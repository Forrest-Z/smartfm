#include "vehicle_tracking_box.h"

	
vehicle_tracking_box::vehicle_tracking_box():
			private_nh_("~")
	{
		ros::NodeHandle nh;
		segpose_batch_sub_  = nh.subscribe("segment_pose_batches", 1, &vehicle_tracking_box::measurement_callback, this);
		anchor_point_pub_   = nh.advertise<sensor_msgs::PointCloud>("anchor_pcl", 2);
		filtered_anchor_point_pub_ = nh.advertise<sensor_msgs::PointCloud>("filtered_anchor_pcl", 2);
		contour_cloud_pub_	= nh.advertise<sensor_msgs::PointCloud>("contour_pcl", 2);
		meas_polygon_pub_  = nh.advertise<geometry_msgs::PolygonStamped>("meas_polygon_debug", 10);
		object_total_id_ = 0;

		private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
	}

	void vehicle_tracking_box::measurement_callback(const MODT::segment_pose_batches& batches)
	{
		if(batches.clusters.size()==0) return;
		cout<<endl;
		cout<<"measurement_callback"<<batches.clusters.back().segments.back().header.seq<<endl;
		MODT::segment_pose_batches batches_copy = batches;

		register_cluster2history(batches_copy);
		tracks_visualization();
	}

	//Here just use a brutal-and-naive method to do data association;
	//later will improve it with a more efficient and safe method;
	void vehicle_tracking_box::register_cluster2history(MODT::segment_pose_batches& batches)
	{
		//pair.first is track id, second is measure id;
		std::vector<std::pair<size_t, size_t> > track_measure_pairs;
		std::vector<size_t> unregistered_measurements;

		for(size_t i=0; i<batches.clusters.size(); i++)
		{
			MODT::segment_pose_batch &cluster_tmp = batches.clusters[i];
			bool registered_to_history = false;
			size_t associated_track_id = 0;

			//chose the probe point at the last but two batch, which corresponds to the final batch in last measurement;

			//pay attention here;
			if(cluster_tmp.segments[cluster_tmp.segments.size()-2].points.size()==0)continue;

			//this may fail to break a track into 2, when measurements are not able to process in time and dropped;
			geometry_msgs::Point32 probe_point_tmp = cluster_tmp.segments[cluster_tmp.segments.size()-2].points.front();

			for(size_t j=0; j<object_tracks_.size(); j++)
			{
				if(registered_to_history) break;
				sensor_msgs::PointCloud &last_pointcloud = object_tracks_[j].last_measurement.segments.back();
				for(size_t k=0; k<last_pointcloud.points.size(); k++)
				{
					geometry_msgs::Point32 &history_pt_tmp = last_pointcloud.points[k];
					float dist_between_pts = sqrtf((history_pt_tmp.x - probe_point_tmp.x)*(history_pt_tmp.x - probe_point_tmp.x)+(history_pt_tmp.y - probe_point_tmp.y)*(history_pt_tmp.y - probe_point_tmp.y));

					//actually they should be the same point;
					//if(dist_between_pts< 0.01)
					if(dist_between_pts< 0.5)
					{
						registered_to_history = true;
						associated_track_id = j;
						break;
					}
				}
			}

			if(registered_to_history)
			{
				std::pair<size_t, size_t> pair_tmp = std::make_pair(associated_track_id, i);
				track_measure_pairs.push_back(pair_tmp);
			}
			else
			{
				cout<<endl;
				cout<<"segment not registered"<<cluster_tmp.segments.back().header.seq<<endl;
				unregistered_measurements.push_back(i);
			}
		}

		update_motionShape(batches, track_measure_pairs, unregistered_measurements);
	}

	//not consider merge-split issues here;

	void vehicle_tracking_box::update_motionShape(MODT::segment_pose_batches& batches, std::vector<std::pair<size_t, size_t> > &track_measure_pairs, std::vector<size_t> &unregistered_measurements)
	{
		//1st: update the associated tracks;
		for(size_t i=0; i<track_measure_pairs.size(); i++)
		{
			size_t track_serial = track_measure_pairs[i].first;
			size_t meas_serial	= track_measure_pairs[i].second;

			box_model_track &track_tmp = object_tracks_[track_serial];
			MODT::segment_pose_batch &lastbatch_tmp = track_tmp.last_measurement;
			MODT::segment_pose_batch &incoming_meas_tmp = batches.clusters[meas_serial];

			//calculate the transformation between two measurement;

			//remember that object-attached coordinate can be arbitrarily defined, but "object transformation" in the odom frame will be always the same;
			//it is the "object transformation" that really matters;
			tf::Pose oldMeas_poseinOdom, newMeas_poseinOdom;
			ICP_motion2shape(track_tmp, lastbatch_tmp, incoming_meas_tmp, oldMeas_poseinOdom, newMeas_poseinOdom);
			tf::Pose oldMeas_to_newMeas = oldMeas_poseinOdom.inverse()*newMeas_poseinOdom;
			double delt_x = newMeas_poseinOdom.getOrigin().x() - oldMeas_poseinOdom.getOrigin().x();
			double delt_y = newMeas_poseinOdom.getOrigin().y() - oldMeas_poseinOdom.getOrigin().y();
			//double delt_thetha =  tf::getYaw(oldMeas_to_newMeas.getRotation());
			//double current_moving_direction = atan2(delt_y, delt_x)+delt_thetha;
			double current_moving_direction = atan2(delt_y, delt_x);

			box_model measure_box;
			calculate_measurement_box(incoming_meas_tmp, current_moving_direction, measure_box);

			//todo: to update the model box using the measurement box;
			update_box_track(track_tmp, measure_box);


			//transform "anchor points" and "contour points", and to visualize the results;
			geometry_msgs::Point32 old_anchor_point = track_tmp.anchor_points.back();
			geometry_msgs::Point32 new_anchor_point;
			attached_points_transform(old_anchor_point, new_anchor_point, oldMeas_poseinOdom, newMeas_poseinOdom);
			ROS_INFO("old anchor: (%3f, %3f); new anchor: (%3f, %3f)", old_anchor_point.x, old_anchor_point.y, new_anchor_point.x, new_anchor_point.y);
			track_tmp.anchor_points.push_back(new_anchor_point);

			double anchor_pt_dist = sqrt((new_anchor_point.x-old_anchor_point.x)*(new_anchor_point.x-old_anchor_point.x)+(new_anchor_point.y-old_anchor_point.y)*(new_anchor_point.y-old_anchor_point.y));
			double time_difference = (incoming_meas_tmp.segments.back().header.stamp - lastbatch_tmp.segments.back().header.stamp).toSec();
			track_tmp.moving_direction = atan2(new_anchor_point.y-old_anchor_point.y, new_anchor_point.x-old_anchor_point.x);
			track_tmp.velocity = anchor_pt_dist/time_difference;

			sensor_msgs::PointCloud transformed_contour;
			transformed_contour.header =incoming_meas_tmp.segments.back().header;
			for(size_t j=0; j<track_tmp.contour_points.points.size(); j++)
			{
				geometry_msgs::Point32 old_contour_point = track_tmp.contour_points.points[j];
				geometry_msgs::Point32 new_contour_point;
				attached_points_transform(old_contour_point, new_contour_point, oldMeas_poseinOdom, newMeas_poseinOdom);
				transformed_contour.points.push_back(new_contour_point);
			}
			for(size_t j=0; j<incoming_meas_tmp.segments.back().points.size(); j++)
			{
				transformed_contour.points.push_back(incoming_meas_tmp.segments.back().points[j]);
			}
			track_tmp.contour_points = transformed_contour;
			track_tmp.last_measurement = incoming_meas_tmp;
			track_tmp.update_time = incoming_meas_tmp.segments.back().header.stamp;

			geometry_msgs::PoseWithCovarianceStamped filtered_pose;
			track_tmp.tracker->update(new_anchor_point.x, new_anchor_point.y, track_tmp.omega, track_tmp.update_time);
			track_tmp.tracker->getEstimate(track_tmp.update_time, filtered_pose);
			MatrixWrapper::ColumnVector ekf_output_tmp = track_tmp.tracker->filter_->PostGet()->ExpectedValueGet();
			cout<<"ekf_output_tmp: "<<ekf_output_tmp(1)<<","<<ekf_output_tmp(2)<<","<<ekf_output_tmp(3)<<","<<ekf_output_tmp(4)<<","<<ekf_output_tmp(5)<<endl;

			track_tmp.omega = ekf_output_tmp(5);

			geometry_msgs::Point32 filtered_anchor_point;
			filtered_anchor_point.x = (float)filtered_pose.pose.pose.position.x;
			filtered_anchor_point.y = (float)filtered_pose.pose.pose.position.y;
			track_tmp.filtered_anchor_points.push_back(filtered_anchor_point);

			if(incoming_meas_tmp.object_label==1)track_tmp.update_object_belief(true);
			else {track_tmp.update_object_belief(false);}
		}

		//2nd: initiate new unassociated tracks;
		for(size_t i=0; i<unregistered_measurements.size(); i++)
		{
			size_t meas_serial	= unregistered_measurements[i];
			MODT::segment_pose_batch &incoming_meas_tmp = batches.clusters[meas_serial];
			if(incoming_meas_tmp.object_label != 1) continue;

			box_model_track new_track_tmp;
			new_track_tmp.object_id = object_total_id_;
			new_track_tmp.last_measurement = incoming_meas_tmp;

			//calculate the anchor point;
			geometry_msgs::Point32 centroid_tmp;
			centroid_tmp.x = 0.0;
			centroid_tmp.y = 0.0;
			centroid_tmp.z = 0.0;
			for(size_t k=0; k<incoming_meas_tmp.segments.back().points.size(); k++)
			{
				centroid_tmp.x = centroid_tmp.x + incoming_meas_tmp.segments.back().points[k].x;
				centroid_tmp.y = centroid_tmp.y + incoming_meas_tmp.segments.back().points[k].y;
				new_track_tmp.contour_points.points.push_back(incoming_meas_tmp.segments.back().points[k]);
			}
			centroid_tmp.x = centroid_tmp.x/(float)incoming_meas_tmp.segments.back().points.size();
			centroid_tmp.y = centroid_tmp.y/(float)incoming_meas_tmp.segments.back().points.size();

			geometry_msgs::Point32 centroid_tmp2;
			centroid_tmp2.x = 0.0;
			centroid_tmp2.y = 0.0;
			centroid_tmp2.z = 0.0;
			for(size_t k=0; k<incoming_meas_tmp.segments.front().points.size(); k++)
			{
				centroid_tmp2.x = centroid_tmp2.x + incoming_meas_tmp.segments.front().points[k].x;
				centroid_tmp2.y = centroid_tmp2.y + incoming_meas_tmp.segments.front().points[k].y;
				new_track_tmp.contour_points.points.push_back(incoming_meas_tmp.segments.front().points[k]);
			}
			centroid_tmp2.x = centroid_tmp2.x/(float)incoming_meas_tmp.segments.front().points.size();
			centroid_tmp2.y = centroid_tmp2.y/(float)incoming_meas_tmp.segments.front().points.size();

			double distance_tmp = sqrt((centroid_tmp.x-centroid_tmp2.x)*(centroid_tmp.x-centroid_tmp2.x)+(centroid_tmp.y-centroid_tmp2.y)*(centroid_tmp.y-centroid_tmp2.y));
			double angle_init = atan2(centroid_tmp.y-centroid_tmp2.y, centroid_tmp.x-centroid_tmp2.x);
			double time_diff_tmp = (incoming_meas_tmp.segments.back().header.stamp - incoming_meas_tmp.segments.front().header.stamp).toSec();
			assert(time_diff_tmp>0.0);
			double speed_value= distance_tmp/time_diff_tmp;
			new_track_tmp.velocity = speed_value;
			new_track_tmp.moving_direction = angle_init;

			double vx_value = speed_value*cos(angle_init);
			double vy_value = speed_value*sin(angle_init);

			//the first anchor point;
			new_track_tmp.anchor_points.push_back(centroid_tmp);
			new_track_tmp.update_time = incoming_meas_tmp.segments.back().header.stamp;
			object_tracks_.push_back(new_track_tmp);
			object_total_id_++;

			new_track_tmp.tracker->set_params(3.0, 3.0, 3.0, 3.0, M_PI*0.2, 0.3, 0.3, M_PI*100);
			new_track_tmp.tracker->init_filter(centroid_tmp.x, centroid_tmp.y, vx_value, vy_value, 0.0);
			new_track_tmp.update_object_belief(true);
		}

		//3rd: delete old tracks;
		size_t deleted_track_num = 0;
		latest_input_time_ = batches.header.stamp;
		for(size_t i=0; i<object_tracks_.size(); )
		{
			double time_passed = (latest_input_time_ - object_tracks_[i].update_time).toSec();
			if(time_passed > 1.0)
			{
				object_tracks_.erase(object_tracks_.begin()+i);
				deleted_track_num++;
			}
			else i++;
		}

		ROS_INFO("updated tracks: %ld; new tracks: %ld; deleted tracks: %ld; remained tracks: %ld", track_measure_pairs.size(), unregistered_measurements.size(), deleted_track_num, object_tracks_.size());
	}

	void vehicle_tracking_box::ICP_motion2shape(box_model_track& track, MODT::segment_pose_batch& old_meas, MODT::segment_pose_batch& new_meas, tf::Pose& oldMeas_poseinOdom, tf::Pose& newMeas_poseinOdom )
	{
		double time_difference = (new_meas.segments.back().header.stamp - old_meas.segments.back().header.stamp).toSec();

		//1st step: for a single cluster, try to estimate its motion using ICP;
		geometry_msgs::Pose old_pose = old_meas.ego_poses.back();
		geometry_msgs::Pose new_pose = new_meas.ego_poses.back();

		//sensor_msgs::PointCloud old_cloud = track.contour_points;
		sensor_msgs::PointCloud new_cloud, old_cloud;

		for(size_t i=0; i<new_meas.segments.size(); i++)
			for(size_t j=0; j<new_meas.segments[i].points.size(); j++)
			{
				new_cloud.points.push_back(new_meas.segments[i].points[j]);
				geometry_msgs::Point32 deputy_point;
				deputy_point.x = new_meas.segments[i].points[j].x + 10.0;
				deputy_point.y = new_meas.segments[i].points[j].y;
				new_cloud.points.push_back(deputy_point);
			}

		for(size_t i=0; i<old_meas.segments.size(); i++)
			for(size_t j=0; j<old_meas.segments[i].points.size(); j++)
			{
				old_cloud.points.push_back(old_meas.segments[i].points[j]);
				geometry_msgs::Point32 deputy_point;
				deputy_point.x = old_meas.segments[i].points[j].x + 10.0;
				deputy_point.y = old_meas.segments[i].points[j].y;
				old_cloud.points.push_back(deputy_point);
			}

		//predicting using tracking information;
		geometry_msgs::Pose old_to_predict;
		old_to_predict.orientation.x = 0.0;
		old_to_predict.orientation.y = 0.0;
		old_to_predict.orientation.z = 0.0;
		old_to_predict.orientation.w = 1.0;
		old_to_predict.position.x = 0.0;
		old_to_predict.position.y = 0.0;
		old_to_predict.position.z = 0.0;

		old_to_predict.position.x = time_difference*track.velocity*cos(track.moving_direction);
		old_to_predict.position.y = time_difference*track.velocity*sin(track.moving_direction);

		cout<<"old_to_predict: "<<time_difference<<","<<track.velocity<<","<<track.moving_direction<<","<<old_to_predict.position.x<<","<<old_to_predict.position.y<<endl;

		//please refer to my evernote 20140103 for more information;
		//a. calculate the rough estimation for the initial pose to speed up ICP;
		std::vector<geometry_msgs::Point32> centroid_position;
		geometry_msgs::Point32 centroid_tmp;
		centroid_tmp.x = 0.0;
		centroid_tmp.y = 0.0;
		centroid_tmp.z = 0.0;
		for(size_t k=0; k<old_cloud.points.size(); k++)
		{
			centroid_tmp.x = centroid_tmp.x + old_cloud.points[k].x;
			centroid_tmp.y = centroid_tmp.y + old_cloud.points[k].y;
		}
		centroid_tmp.x = centroid_tmp.x/(float)old_cloud.points.size();
		centroid_tmp.y = centroid_tmp.y/(float)old_cloud.points.size();
		centroid_position.push_back(centroid_tmp);

		centroid_tmp.x = centroid_tmp.x + old_to_predict.position.x;
		centroid_tmp.y = centroid_tmp.y + old_to_predict.position.y;
		centroid_position.push_back(centroid_tmp);

		tf::Pose Odom_to_Vtm1 = tf::Transform(tf::Matrix3x3(tf::createIdentityQuaternion()), tf::Vector3(tfScalar(centroid_position.front().x), tfScalar(centroid_position.front().y), tfScalar(0)));
		tf::Pose Odom_to_Vt = tf::Transform(tf::Matrix3x3(tf::createIdentityQuaternion()), tf::Vector3(tfScalar(centroid_position.back().x), tfScalar(centroid_position.back().y), tfScalar(0)));
		tf::Pose Vtm1_to_Vt = Odom_to_Vtm1.inverse()*Odom_to_Vt;

		tf::Pose Odom_to_Etm1, Odom_to_Et, Etm1_to_Et;
		tf::poseMsgToTF(old_pose, Odom_to_Etm1);
		tf::poseMsgToTF(new_pose, Odom_to_Et);
		Etm1_to_Et = Odom_to_Etm1.inverse()*Odom_to_Et;

		tf::Pose Vt_to_Et = Odom_to_Vt.inverse()*Odom_to_Et;
		tf::Pose Vtm1_to_Etvirtual = Vt_to_Et;
		tf::Pose Etvirtual_to_Et = Vtm1_to_Etvirtual.inverse()*Vtm1_to_Vt*Vt_to_Et;
		tf::Pose Et_to_Etvirtual = Etvirtual_to_Et.inverse();
		tf::Pose Etm1_to_Etvirtual = Etm1_to_Et*Et_to_Etvirtual;

		float x_initial = (float)Etm1_to_Etvirtual.getOrigin().getX();
		float y_initial = (float)Etm1_to_Etvirtual.getOrigin().getY();
		float yaw_initial = (float) tf::getYaw(Etm1_to_Etvirtual.getRotation());

		cout<<"other vehicle rough movement:"<<Vtm1_to_Vt.getOrigin().getX()<<","<<Vtm1_to_Vt.getOrigin().getY()<<"0"<<endl;
		cout<<"ego vehicle odom movement:"<<Etm1_to_Et.getOrigin().getX()<<","<<Etm1_to_Et.getOrigin().getY()<<","<<tf::getYaw(Etm1_to_Et.getRotation())<<endl;
		cout<<"ICP initial guess: "<<x_initial<<","<<y_initial<<","<<yaw_initial<<endl;

		CPose2D	initialPose(x_initial, y_initial, yaw_initial);

		//to calculate the deputy cloud;
		sensor_msgs::PointCloud old_cloud_shiftAdded, new_cloud_shiftAdded;
		tf::Pose lidarPose_tf;
		tf::poseMsgToTF(new_pose, lidarPose_tf);

		CSimplePointsMap		m1, m2;
		float					runningTime;
		CICP::TReturnInfo		info;
		CICP					ICP;

		constructPtsMap(old_pose, old_cloud, m1);
		constructPtsMap(new_pose, new_cloud, m2);

		ICP.options.ICP_algorithm =  icpLevenbergMarquardt;
		ICP.options.maxIterations			= 100;
		ICP.options.thresholdAng			= DEG2RAD(10.0f);
		ICP.options.thresholdDist			= 0.75f;
		//ICP.options.thresholdDist			= 2.0f;
		ICP.options.ALFA					= 0.5f;
		ICP.options.smallestThresholdDist	= 0.01f;
		ICP.options.doRANSAC = false;
		ICP.options.onlyClosestCorrespondences = false;
		//ICP.options.dumpToConsole();

		//CPose2D		initialPose(0.0f,0.0f,(float)DEG2RAD(0.0f));
		CPosePDFPtr pdf = ICP.Align(&m1, &m2, initialPose, &runningTime,(void*)&info);
		CPosePDFGaussian  gPdf;
		gPdf.copyFrom(*pdf);

		printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ",
				runningTime*1000,
				info.nIterations,
				runningTime*1000.0f/info.nIterations,
				info.goodness*100 );
		cout << "Mean of estimation: " << pdf->getMeanVal() << endl<< endl;

		//c: to recover the vehicle pose and speed in the global frame;
		double ICP_output_x 	= gPdf.mean.x();
		double ICP_output_y 	= gPdf.mean.y();
		double ICP_output_yaw 	= gPdf.mean.phi();

		cout<<"ICP output: "<<ICP_output_x<<","<<ICP_output_y<<","<<ICP_output_yaw<<endl;

		tf::Pose ICP_Etm1_to_Etvirtual = tf::Transform(tf::Matrix3x3(tf::createQuaternionFromYaw(ICP_output_yaw)), tf::Vector3(tfScalar(ICP_output_x), tfScalar(ICP_output_y), tfScalar(0)));

		tf::Pose ICP_Et_To_Etvirtual = Etm1_to_Et.inverse()*ICP_Etm1_to_Etvirtual;
		tf::Pose ICP_Vtm1_To_Vt = Vtm1_to_Etvirtual*ICP_Et_To_Etvirtual.inverse()*Vt_to_Et.inverse();

		double vehicle_delta_x = ICP_Vtm1_To_Vt.getOrigin().getX();
		double vehicle_delta_y = ICP_Vtm1_To_Vt.getOrigin().getY();
		double vehicle_delta_yaw = tf::getYaw(ICP_Vtm1_To_Vt.getRotation());
		cout<<"vehicle motion: "<<vehicle_delta_x<<","<<vehicle_delta_y<<","<<vehicle_delta_yaw<<endl;
		track.omega = vehicle_delta_yaw/time_difference;

		newMeas_poseinOdom = Odom_to_Vt;
		oldMeas_poseinOdom = Odom_to_Vt*ICP_Vtm1_To_Vt.inverse();
	}

	void vehicle_tracking_box::calculate_measurement_box(MODT::segment_pose_batch& new_meas, double current_moving_direction, box_model &measurement_box)
	{
		geometry_msgs::PolygonStamped meas_polygon;
		meas_polygon.header = new_meas.segments.back().header;

		geometry_msgs::Pose lidar_pose = new_meas.ego_poses.back();
		sensor_msgs::PointCloud meas_cloud = new_meas.segments.back();

		TPoint2D lidar_point;
		lidar_point.x = lidar_pose.position.x;
		lidar_point.y = lidar_pose.position.y;
		double perpendicular_direction = current_moving_direction + M_PI_2;

		TLine2D side1, side2, bottom1, bottom2;

		double side1_dist=DBL_MAX, side2_dist = -DBL_MAX;
		double bottom1_dist=DBL_MAX, bottom2_dist = -DBL_MAX;

		for(size_t i=0; i<meas_cloud.points.size(); i++)
		{
			//Ax+By+C=0;
			//A=sin(thetha), B= -cos(thetha), C=cos(thetha)*y0-sin(thetha)*x0;
			double a_tmp = sin(current_moving_direction);
			double b_tmp = -cos(current_moving_direction);
			double c_tmp = cos(current_moving_direction)*meas_cloud.points[i].y-sin(current_moving_direction)*meas_cloud.points[i].x;
			TLine2D line_tmp(a_tmp, b_tmp, c_tmp);
			double dist_tmp = line_tmp.signedDistance(lidar_point);
			if(dist_tmp<side1_dist)
			{
				side1_dist = dist_tmp;
				side1 = line_tmp;
			}
			if(dist_tmp>side2_dist)
			{
				side2_dist = dist_tmp;
				side2 = line_tmp;
			}

			double a_tmp2 = sin(perpendicular_direction);
			double b_tmp2 = -cos(perpendicular_direction);
			double c_tmp2 = cos(perpendicular_direction)*meas_cloud.points[i].y-sin(perpendicular_direction)*meas_cloud.points[i].x;
			TLine2D line2_tmp(a_tmp2, b_tmp2, c_tmp2);
			double dist_tmp2 = line2_tmp.signedDistance(lidar_point);

			if(dist_tmp2<bottom1_dist)
			{
				bottom1_dist = dist_tmp2;
				bottom1 = line2_tmp;
			}
			if(dist_tmp2>bottom2_dist)
			{
				bottom2_dist = dist_tmp2;
				bottom2 = line2_tmp;
			}
		}

		//cout << "side1:" <<side1.coefs[0]<<","<<side1.coefs[1]<<","<<side1.coefs[2]<<endl;
		//cout << "side2:" <<side2.coefs[0]<<","<<side2.coefs[1]<<","<<side2.coefs[2]<<endl;
		//cout << "bottom1:" <<bottom1.coefs[0]<<","<<bottom1.coefs[1]<<","<<bottom1.coefs[2]<<":"<<bottom1_dist<<endl;
		//cout << "bottom2:" <<bottom2.coefs[0]<<","<<bottom2.coefs[1]<<","<<bottom2.coefs[2]<<":"<<bottom2_dist<<endl;

		TLine2D line_tmp;
		if(fabs(side1_dist)>fabs(side2_dist))
		{
			line_tmp = side1;
			side1 = side2;
			side2 = line_tmp;
		}
		if(fabs(bottom1_dist)>fabs(bottom2_dist))
		{
			line_tmp = bottom1;
			bottom1 = bottom2;
			bottom2 = line_tmp;
		}

		geometry_msgs::Point32 intersect_pt[4];
		TObject2D intersection_obj[4];
		intersect(side1, bottom1, intersection_obj[0]);
		intersect(side2, bottom1, intersection_obj[1]);
		intersect(side2, bottom2, intersection_obj[2]);
		intersect(side1, bottom2, intersection_obj[3]);

		for(size_t i=0; i<4; i++)
		{
			TPoint2D intersection_pt;
			intersection_obj[i].getPoint(intersection_pt);
			intersect_pt[i].x = (float)intersection_pt.x;
			intersect_pt[i].y = (float)intersection_pt.y;
			meas_polygon.polygon.points.push_back(intersect_pt[i]);
		}

		meas_polygon_pub_.publish(meas_polygon);

		//todo: to construct the measurement box;

	}

	void vehicle_tracking_box::update_box_track(box_model_track &track, box_model &measurement_box)
	{

	}

	void vehicle_tracking_box::constructPtsMap(geometry_msgs::Pose &lidar_pose, sensor_msgs::PointCloud &segment_pointcloud, CSimplePointsMap &map)
	{
		//2nd step: input the scan readings with pointcloud readings;
		tf::Pose lidarTFPose;
		tf::poseMsgToTF(lidar_pose, lidarTFPose);

		geometry_msgs::Pose temppose;
		temppose.position.x=0;
		temppose.position.y=0;
		temppose.position.z=0;
		temppose.orientation.x=0;
		temppose.orientation.y=0;
		temppose.orientation.z=0;
		temppose.orientation.w=1;

		for(size_t ip=0; ip<segment_pointcloud.points.size(); ip++)
		{
			temppose.position.x = segment_pointcloud.points[ip].x;
			temppose.position.y = segment_pointcloud.points[ip].y;

			tf::Pose tempTfPose;
			tf::poseMsgToTF(temppose, tempTfPose);

			tf::Pose pointInlidar = lidarTFPose.inverseTimes(tempTfPose);
			geometry_msgs::Point32 pointtemp;
			pointtemp.x=(float)pointInlidar.getOrigin().x();
			pointtemp.y=(float)pointInlidar.getOrigin().y();
			map.insertPoint(pointtemp.x, pointtemp.y, 0.0);
		}
	}

	//points attached to vehicle will be transformed together into new places in the "odom" frame;
	void vehicle_tracking_box::attached_points_transform(geometry_msgs::Point32& pt_input, geometry_msgs::Point32& pt_output, tf::Pose& oldMeas_poseinOdom, tf::Pose& newMeas_poseinOdom)
	{
		tf::Pose ptPose_inOdom, ptPose_inVehicle, ptPose_inOdom_new;
		ptPose_inOdom = tf::Transform(tf::Matrix3x3(tf::createIdentityQuaternion()), tf::Vector3(tfScalar(pt_input.x), tfScalar(pt_input.y), tfScalar(0)));
		ptPose_inVehicle = oldMeas_poseinOdom.inverse()*ptPose_inOdom;
		ptPose_inOdom_new = newMeas_poseinOdom*ptPose_inVehicle;
		pt_output.x = (float)ptPose_inOdom_new.getOrigin().getX();
		pt_output.y = (float)ptPose_inOdom_new.getOrigin().getY();
		pt_output.z = 0.0;
	}

	void vehicle_tracking_box::tracks_visualization()
	{
		sensor_msgs::PointCloud contour_pcl, anchor_pcl, filtered_anchor_pcl;
		contour_pcl.header.frame_id = odom_frame_id_;
		anchor_pcl.header.frame_id = odom_frame_id_;
		filtered_anchor_pcl.header.frame_id = odom_frame_id_;
		contour_pcl.header.stamp = latest_input_time_;
		anchor_pcl.header.stamp = latest_input_time_;
		filtered_anchor_pcl.header.stamp = latest_input_time_;

		for(size_t i=0; i<object_tracks_.size(); i++)
		{
			bool visualize_flag = (latest_input_time_-object_tracks_[i].update_time).toSec()<0.001;
			if(object_tracks_[i].vehicle_evidence<0.0) visualize_flag = false;

			//bool visualize_flag = true;
			if(visualize_flag)
			{
				for(size_t j=0; j<object_tracks_[i].contour_points.points.size(); j++)
				{
					contour_pcl.points.push_back(object_tracks_[i].contour_points.points[j]);
				}

				for(size_t j=0; j<object_tracks_[i].anchor_points.size(); j++)
				{
					anchor_pcl.points.push_back(object_tracks_[i].anchor_points[j]);
				}

				for(size_t j=0; j<object_tracks_[i].filtered_anchor_points.size(); j++)
				{
					filtered_anchor_pcl.points.push_back(object_tracks_[i].filtered_anchor_points[j]);
				}
			}
		}
		contour_cloud_pub_.publish(contour_pcl);
		anchor_point_pub_.publish(anchor_pcl);
		filtered_anchor_point_pub_.publish(filtered_anchor_pcl);
	}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vehicle_tracking_node");
	vehicle_tracking_box vehicle_tracking_node;
	ros::spin();
	return (0);
}

