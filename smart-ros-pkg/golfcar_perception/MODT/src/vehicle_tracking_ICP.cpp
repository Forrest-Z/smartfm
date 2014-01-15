#include "vehicle_tracking_ICP.h"

namespace mrpt{
	
	vehicle_tracking::vehicle_tracking()
	{
		ros::NodeHandle nh;
		segpose_batch_sub_  = nh.subscribe("segment_pose_batches", 1, &vehicle_tracking::measurement_callback, this);
		contour_cloud_pub_	= nh.advertise<sensor_msgs::PointCloud>("contour_pcl", 2);
		anchor_point_pub_   = nh.advertise<sensor_msgs::PointCloud>("anchor_pcl", 2);
		filtered_anchor_point_pub_ = nh.advertise<sensor_msgs::PointCloud>("filtered_anchor_pcl", 2);
		meas_deputy_pub_	= nh.advertise<sensor_msgs::PointCloud>("meas_deputy", 2);
		model_deputy_pub_   = nh.advertise<sensor_msgs::PointCloud>("model_deputy", 2);
		contour_cloud_debug_pub_	= nh.advertise<sensor_msgs::PointCloud>("contour_pcl_debug", 2);
		object_total_id_ = 0;
	}

	void vehicle_tracking::measurement_callback(const MODT::segment_pose_batches& batches)
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
	void vehicle_tracking::register_cluster2history(MODT::segment_pose_batches& batches)
	{
		//pair.first is track id, second is measure id;
		std::vector<std::pair<size_t, size_t> > track_measure_pairs;
		std::vector<size_t> unregistered_measurements;

		for(size_t i=0; i<batches.clusters.size(); i++)
		{
			MODT::segment_pose_batch &cluster_tmp = batches.clusters[i];
			bool registered_to_history = false;
			size_t associated_track_id;
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

	void vehicle_tracking::update_motionShape(MODT::segment_pose_batches& batches, std::vector<std::pair<size_t, size_t> > &track_measure_pairs, std::vector<size_t> &unregistered_measurements)
	{
		//1st: update the associated tracks;
		for(size_t i=0; i<track_measure_pairs.size(); i++)
		{
			size_t track_serial = track_measure_pairs[i].first;
			size_t meas_serial	= track_measure_pairs[i].second;

			model_free_track &track_tmp = object_tracks_[track_serial];
			MODT::segment_pose_batch &lastbatch_tmp = track_tmp.last_measurement;
			MODT::segment_pose_batch &incoming_meas_tmp = batches.clusters[meas_serial];

			//calculate the transformation between two measurement;

			//remember that object-attached coordinate can be arbitrarily defined, but "object transformation" in the odom frame will be always the same;
			//it is the "object transformation" that really matters;
			tf::Pose oldMeas_poseinOdom, newMeas_poseinOdom;
			ICP_motion2shape(track_tmp, lastbatch_tmp, incoming_meas_tmp, oldMeas_poseinOdom, newMeas_poseinOdom);

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
			track_tmp.downsample_model();


			geometry_msgs::PoseWithCovarianceStamped filtered_pose;
			track_tmp.tracker->update(new_anchor_point.x, new_anchor_point.y, track_tmp.omega, track_tmp.update_time);
			track_tmp.tracker->getEstimate(track_tmp.update_time, filtered_pose);
			MatrixWrapper::ColumnVector ekf_output_tmp = track_tmp.tracker->filter_->PostGet()->ExpectedValueGet();
			cout<<"ekf_output_tmp: "<<ekf_output_tmp(1)<<","<<ekf_output_tmp(2)<<","<<ekf_output_tmp(3)<<","<<ekf_output_tmp(4)<<","<<ekf_output_tmp(5)<<endl;

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

			model_free_track new_track_tmp;
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

			new_track_tmp.downsample_model();

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

	void vehicle_tracking::ICP_motion2shape(model_free_track& track, MODT::segment_pose_batch& old_meas, MODT::segment_pose_batch& new_meas, tf::Pose& oldMeas_poseinOdom, tf::Pose& newMeas_poseinOdom )
	{
		double time_difference = (new_meas.segments.back().header.stamp - old_meas.segments.back().header.stamp).toSec();

		//1st step: for a single cluster, try to estimate its motion using ICP;
		geometry_msgs::Pose old_pose = old_meas.ego_poses.back();
		geometry_msgs::Pose new_pose = new_meas.ego_poses.back();

		//sensor_msgs::PointCloud old_segment = old_meas.segments.back();
		sensor_msgs::PointCloud old_cloud = track.contour_points;
		sensor_msgs::PointCloud new_cloud = new_meas.segments.back();

		//predicting using tracking information;
		geometry_msgs::Pose old_to_predict;
		old_to_predict.orientation.x = 0.0;
		old_to_predict.orientation.y = 0.0;
		old_to_predict.orientation.z = 0.0;
		old_to_predict.orientation.w = 1.0;
		old_to_predict.position.x = 0.0;
		old_to_predict.position.y = 0.0;
		old_to_predict.position.z = 0.0;

		if(!track.using_prediction_before_ICP)track.velocity = 0.0;
		old_to_predict.position.x = time_difference*track.velocity*cos(track.moving_direction);
		old_to_predict.position.y = time_difference*track.velocity*sin(track.moving_direction);

		//track.velocity = 0.0;

		cout<<"old_to_predict: "<<time_difference<<","<<track.velocity<<","<<track.moving_direction<<","<<old_to_predict.position.x<<","<<old_to_predict.position.y<<endl;
		double move_distance = sqrt((old_to_predict.position.x*old_to_predict.position.x+old_to_predict.position.y*old_to_predict.position.y));

		if(move_distance>3.0)
		{
			ROS_WARN("moving distance too long");
			track.track_status = 1;
		}

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

		//build the transform from v_(t-1) to v_t;
		//about name convention: transform "Frame_A_To_Frame_B" is equal to "Frame_B_in_Frame_A";
		if(track.track_status==1)
		{
			ROS_WARN("only use tracker");
			tf::Pose Odom_to_Vtm1 = tf::Transform(tf::Matrix3x3(tf::createIdentityQuaternion()), tf::Vector3(tfScalar(centroid_position.front().x), tfScalar(centroid_position.front().y), tfScalar(0)));
			tf::Pose Odom_to_Vt = tf::Transform(tf::Matrix3x3(tf::createQuaternionFromYaw(0.0)), tf::Vector3(tfScalar(centroid_position.back().x), tfScalar(centroid_position.back().y), tfScalar(0)));
			tf::Pose Vtm1_to_Vt = Odom_to_Vtm1.inverse()*Odom_to_Vt;
			newMeas_poseinOdom = Odom_to_Vt;
			oldMeas_poseinOdom = Odom_to_Vt*Vtm1_to_Vt.inverse();
			return;
		}

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
		ICP_deputy_cloud(track, lidarPose_tf, new_cloud, old_cloud,   new_cloud_shiftAdded, old_cloud_shiftAdded);


		CSimplePointsMap		m1, m2, m1_shiftAdded, m2_shiftAdded;
		float					runningTime;
		CICP::TReturnInfo		info;
		CICP					ICP;
		constructPtsMap(old_pose, old_cloud_shiftAdded, m1_shiftAdded);
		constructPtsMap(new_pose, new_cloud_shiftAdded, m2_shiftAdded);

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
		CPosePDFPtr pdf = ICP.Align(&m1_shiftAdded, &m2_shiftAdded, initialPose, &runningTime,(void*)&info);
		CPosePDFGaussian  gPdf;
		gPdf.copyFrom(*pdf);

		printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ",
				runningTime*1000,
				info.nIterations,
				runningTime*1000.0f/info.nIterations,
				info.goodness*100 );
		cout << "Mean of estimation: " << pdf->getMeanVal() << endl<< endl;

		CPose2D	initialPose_step2(gPdf.mean.x(), gPdf.mean.y(), gPdf.mean.phi());
		ICP.options.onlyClosestCorrespondences = true;
		ICP.options.maxIterations			= 1;
		CPosePDFPtr pdf2 = ICP.Align(&m1, &m2, initialPose_step2, &runningTime,(void*)&info);

		CPosePDFGaussian  gPdf2;
		gPdf2.copyFrom(*pdf2);
		//c: to recover the vehicle pose and speed in the global frame;
		double ICP_output_x 	= gPdf2.mean.x();
		double ICP_output_y 	= gPdf2.mean.y();
		double ICP_output_yaw 	= gPdf2.mean.phi();

		/*
		if(track.track_status==1)
		{
			ICP_output_x 	= gPdf.mean.x();
			ICP_output_y 	= gPdf.mean.y();
			ICP_output_yaw 	= gPdf.mean.phi();
		}
		*/

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

		/*
		double x_in_odom = oldMeas_poseinOdom.getOrigin().x() - old_to_predict.position.x;
		double y_in_odom = oldMeas_poseinOdom.getOrigin().y() - old_to_predict.position.y;
		oldMeas_poseinOdom.setOrigin(tf::Vector3(x_in_odom, y_in_odom, 0.0));
		*/
	}

	//generate deputy points for ICP;
	//two purposes: 1, to take into account beam model;
	//				2, to allow only shift, and no orientation;
	void vehicle_tracking::ICP_deputy_cloud(model_free_track& track, tf::Pose lidar_pose, sensor_msgs::PointCloud &meas_cloud, sensor_msgs::PointCloud &model_cloud,  sensor_msgs::PointCloud &deputy_meas_cloud, sensor_msgs::PointCloud &deputy_model_cloud)
	{
		deputy_meas_cloud = meas_cloud;
		deputy_model_cloud = model_cloud;

		//1: add constraint points to take into account beam model while doing ICP;
		int constraint_pts_num = track.beam_constraint_number*2;
		int constraint_pts_num2 = track.beam_constraint_number*2*2;

		float constraint_pts_interval = 0.05;
		double scan_angle_max = M_PI*0.75, scan_angle_min = -M_PI*0.75;
		bool add_pts_angleMax = false, add_pts_angleMin = false;

		double meas_angle_max = -M_PI, meas_angle_min = M_PI;
		double angle_tolerance = M_PI/180.0*45.0;
		geometry_msgs::Point32 meas_angleMax_pt, meas_angleMin_pt;

		geometry_msgs::Pose temppose;
		temppose.position.x=0;
		temppose.position.y=0;
		temppose.position.z=0;
		temppose.orientation.x=0;
		temppose.orientation.y=0;
		temppose.orientation.z=0;
		temppose.orientation.w=1;
		for(size_t ip=0; ip<meas_cloud.points.size(); ip++)
		{
			temppose.position.x = meas_cloud.points[ip].x;
			temppose.position.y = meas_cloud.points[ip].y;

			tf::Pose tempTfPose;
			tf::poseMsgToTF(temppose, tempTfPose);

			tf::Pose pointInlidar = lidar_pose.inverseTimes(tempTfPose);
			geometry_msgs::Point32 pointtemp;
			pointtemp.x=(float)pointInlidar.getOrigin().x();
			pointtemp.y=(float)pointInlidar.getOrigin().y();
			float angle_tmp = atan2f(pointtemp.y, pointtemp.x);

			if(angle_tmp>meas_angle_max)
			{
				meas_angle_max = angle_tmp;
				meas_angleMax_pt = meas_cloud.points[ip];
			}

			if(angle_tmp<meas_angle_min)
			{
				meas_angle_min = angle_tmp;
				meas_angleMin_pt = meas_cloud.points[ip];
			}
		}

		//if object exceed scan angle boundary, add deputy points to the other side of the objects, to convey the scenario of beam model;
		//pay attention that it is "the other side";
		ROS_INFO("meas_angle_min, meas_angle_max %lf, %lf", meas_angle_min, meas_angle_max);

		if(meas_angle_max+angle_tolerance>scan_angle_max)
		{
			ROS_WARN("angle too large");
			add_pts_angleMin = true;
			track.track_status = 1;
		}
		else
		{
			add_pts_angleMin = false;
			track.track_status = 0;
		}

		if(meas_angle_min-angle_tolerance<scan_angle_min)
		{
			ROS_WARN("angle too small");
			add_pts_angleMax = true;
			track.track_status = 1;
		}
		else
		{
			if(!add_pts_angleMin) track.track_status = 0;
			add_pts_angleMax = false;
		}

		add_pts_angleMax = true;
		add_pts_angleMin = true;

		geometry_msgs::Point32 lidar_origin;
		lidar_origin.x = lidar_pose.getOrigin().getX();
		lidar_origin.y = lidar_pose.getOrigin().getY();

		if(add_pts_angleMax)
		{
			ROS_INFO("add deputy points around angle max for ICP");
			//add points to deputy_meas_cloud;
			float lidar_pt_dist = sqrtf((meas_angleMax_pt.x-lidar_origin.x)*(meas_angleMax_pt.x-lidar_origin.x)+(meas_angleMax_pt.y-lidar_origin.y)*(meas_angleMax_pt.y-lidar_origin.y));
			geometry_msgs::Point32 delt_point;
			delt_point.x = meas_angleMax_pt.x-lidar_origin.x;
			delt_point.y = meas_angleMax_pt.y-lidar_origin.y;
			float meas_angleMax_inodom = (float)atan2(delt_point.y, delt_point.x);

			for(int i=1; i<=constraint_pts_num/2; i++)
			{
				geometry_msgs::Point32 constraint_pt_tmp;
				constraint_pt_tmp.x = meas_angleMax_pt.x + (constraint_pts_interval*i)/lidar_pt_dist*delt_point.x;
				constraint_pt_tmp.y = meas_angleMax_pt.y + (constraint_pts_interval*i)/lidar_pt_dist*delt_point.y;
				deputy_meas_cloud.points.push_back(constraint_pt_tmp);
				constraint_pt_tmp.x = meas_angleMax_pt.x - (constraint_pts_interval*i)/lidar_pt_dist*delt_point.x;
				constraint_pt_tmp.y = meas_angleMax_pt.y - (constraint_pts_interval*i)/lidar_pt_dist*delt_point.y;
				deputy_meas_cloud.points.push_back(constraint_pt_tmp);
			}

			//add points to deputy_model_cloud;

			//find the corresponding boundary point on the model cloud;
			geometry_msgs::Point32 model_boundary_point;
			double odom_origin_dist_min = DBL_MAX;
			double odom_origin_dist_max = DBL_MIN;
			geometry_msgs::Point32 max_dist_pt, min_dist_pt;

			//Ax+By+C=0;
			//A=sin(thetha), B= -cos(thetha), C=cos(thetha)*y-sin(thetha)*x;
			//origin to line dist: fabs(c)/sqrtf(A^2+B^2);
			cout<<"max angle"<<endl;
			for(size_t ip=0;ip<model_cloud.points.size(); ip++)
			{
				double a_tmp = sin(meas_angleMax_inodom);
				double b_tmp = -cos(meas_angleMax_inodom);
				double c_tmp = cos(meas_angleMax_inodom)*model_cloud.points[ip].y-sin(meas_angleMax_inodom)*model_cloud.points[ip].x;
				double origin_to_line_dist_tmp = (c_tmp)/sqrt(a_tmp*a_tmp+b_tmp*b_tmp);

				//cout<<"("<<model_cloud.points[ip].x<<","<<model_cloud.points[ip].y<<","<<origin_to_line_dist_tmp<<")"<<"\t";
				if(origin_to_line_dist_tmp>odom_origin_dist_max)
				{
					odom_origin_dist_max = origin_to_line_dist_tmp;
					max_dist_pt = model_cloud.points[ip];
				}

				if(origin_to_line_dist_tmp<odom_origin_dist_min)
				{
					odom_origin_dist_min = origin_to_line_dist_tmp;
					min_dist_pt = model_cloud.points[ip];
				}
			}
			cout<<endl;

			temppose.position.x = max_dist_pt.x;
			temppose.position.y = max_dist_pt.y;
			tf::Pose tempTfPose;
			tf::poseMsgToTF(temppose, tempTfPose);
			tf::Pose pointInlidar = lidar_pose.inverseTimes(tempTfPose);
			geometry_msgs::Point32 pointtemp;
			pointtemp.x=(float)pointInlidar.getOrigin().x();
			pointtemp.y=(float)pointInlidar.getOrigin().y();
			float max_dist_angle_tmp = atan2f(pointtemp.y, pointtemp.x);

			temppose.position.x = min_dist_pt.x;
			temppose.position.y = min_dist_pt.y;
			tf::poseMsgToTF(temppose, tempTfPose);
			pointInlidar = lidar_pose.inverseTimes(tempTfPose);
			pointtemp.x=(float)pointInlidar.getOrigin().x();
			pointtemp.y=(float)pointInlidar.getOrigin().y();
			float min_dist_angle_tmp = atan2f(pointtemp.y, pointtemp.x);

			//find the point in the lidar coordinate with bigger angle;
			model_boundary_point = max_dist_angle_tmp>min_dist_angle_tmp?max_dist_pt:min_dist_pt;

			for(int i=1; i<=constraint_pts_num2 /2; i++)
			{
				geometry_msgs::Point32 constraint_pt_tmp;
				constraint_pt_tmp.x = model_boundary_point.x + (constraint_pts_interval*i)*cos(meas_angleMax_inodom);
				constraint_pt_tmp.y = model_boundary_point.y + (constraint_pts_interval*i)*sin(meas_angleMax_inodom);
				deputy_model_cloud.points.push_back(constraint_pt_tmp);
				constraint_pt_tmp.x = model_boundary_point.x - (constraint_pts_interval*i)*cos(meas_angleMax_inodom);
				constraint_pt_tmp.y = model_boundary_point.y - (constraint_pts_interval*i)*sin(meas_angleMax_inodom);
				deputy_model_cloud.points.push_back(constraint_pt_tmp);
			}

		}

		if(add_pts_angleMin)
		{
			ROS_INFO("add deputy points around angle min for ICP");
			//add points to deputy_meas_cloud;
			float lidar_pt_dist = sqrtf((meas_angleMin_pt.x-lidar_origin.x)*(meas_angleMin_pt.x-lidar_origin.x)+(meas_angleMin_pt.y-lidar_origin.y)*(meas_angleMin_pt.y-lidar_origin.y));
			geometry_msgs::Point32 delt_point;
			delt_point.x = meas_angleMin_pt.x-lidar_origin.x;
			delt_point.y = meas_angleMin_pt.y-lidar_origin.y;
			float meas_angleMin_inodom = (float)atan2(delt_point.y, delt_point.x);

			for(int i=1; i<=constraint_pts_num/2; i++)
			{
				geometry_msgs::Point32 constraint_pt_tmp;
				constraint_pt_tmp.x = meas_angleMin_pt.x + (constraint_pts_interval*i)/lidar_pt_dist*delt_point.x;
				constraint_pt_tmp.y = meas_angleMin_pt.y + (constraint_pts_interval*i)/lidar_pt_dist*delt_point.y;
				deputy_meas_cloud.points.push_back(constraint_pt_tmp);
				constraint_pt_tmp.x = meas_angleMin_pt.x - (constraint_pts_interval*i)/lidar_pt_dist*delt_point.x;
				constraint_pt_tmp.y = meas_angleMin_pt.y - (constraint_pts_interval*i)/lidar_pt_dist*delt_point.y;
				deputy_meas_cloud.points.push_back(constraint_pt_tmp);
			}

			//add points to deputy_model_cloud;

			//find the corresponding boundary point on the model cloud;
			geometry_msgs::Point32 model_boundary_point;
			double odom_origin_dist_min = DBL_MAX;
			double odom_origin_dist_max = DBL_MIN;
			geometry_msgs::Point32 max_dist_pt, min_dist_pt;

			//Ax+By+C=0;
			//A=sin(thetha), B= -cos(thetha), C=cos(thetha)*y-sin(thetha)*x;
			//origin to line dist: fabs(c)/sqrtf(A^2+B^2);
			for(size_t ip=0;ip<model_cloud.points.size(); ip++)
			{
				double a_tmp = sin(meas_angleMin_inodom);
				double b_tmp = -cos(meas_angleMin_inodom);
				double c_tmp = cos(meas_angleMin_inodom)*model_cloud.points[ip].y-sin(meas_angleMin_inodom)*model_cloud.points[ip].x;
				double origin_to_line_dist_tmp = (c_tmp)/sqrt(a_tmp*a_tmp+b_tmp*b_tmp);

				if(origin_to_line_dist_tmp>odom_origin_dist_max)
				{
					odom_origin_dist_max = origin_to_line_dist_tmp;
					max_dist_pt = model_cloud.points[ip];
				}

				if(origin_to_line_dist_tmp<odom_origin_dist_min)
				{
					odom_origin_dist_min = origin_to_line_dist_tmp;
					min_dist_pt = model_cloud.points[ip];
				}
			}

			temppose.position.x = max_dist_pt.x;
			temppose.position.y = max_dist_pt.y;
			tf::Pose tempTfPose;
			tf::poseMsgToTF(temppose, tempTfPose);
			tf::Pose pointInlidar = lidar_pose.inverseTimes(tempTfPose);
			geometry_msgs::Point32 pointtemp;
			pointtemp.x=(float)pointInlidar.getOrigin().x();
			pointtemp.y=(float)pointInlidar.getOrigin().y();
			float max_dist_angle_tmp = atan2f(pointtemp.y, pointtemp.x);

			temppose.position.x = min_dist_pt.x;
			temppose.position.y = min_dist_pt.y;
			tf::poseMsgToTF(temppose, tempTfPose);
			pointInlidar = lidar_pose.inverseTimes(tempTfPose);
			pointtemp.x=(float)pointInlidar.getOrigin().x();
			pointtemp.y=(float)pointInlidar.getOrigin().y();
			float min_dist_angle_tmp = atan2f(pointtemp.y, pointtemp.x);

			//find the point in the lidar coordinate with smaller angle;
			model_boundary_point = max_dist_angle_tmp<min_dist_angle_tmp ? max_dist_pt:min_dist_pt;

			for(int i=1; i<=constraint_pts_num2 /2; i++)
			{
				geometry_msgs::Point32 constraint_pt_tmp;
				constraint_pt_tmp.x = model_boundary_point.x + (constraint_pts_interval*i)*cos(meas_angleMin_inodom);
				constraint_pt_tmp.y = model_boundary_point.y + (constraint_pts_interval*i)*sin(meas_angleMin_inodom);
				deputy_model_cloud.points.push_back(constraint_pt_tmp);
				constraint_pt_tmp.x = model_boundary_point.x - (constraint_pts_interval*i)*cos(meas_angleMin_inodom);
				constraint_pt_tmp.y = model_boundary_point.y - (constraint_pts_interval*i)*sin(meas_angleMin_inodom);
				deputy_model_cloud.points.push_back(constraint_pt_tmp);
			}
		}

		//2: futher add points to allow only shift of the object;
		sensor_msgs::PointCloud meas_deputy_tmp, model_deputy_tmp;
		for(size_t i=0; i<deputy_meas_cloud.points.size(); i++)
		{
			geometry_msgs::Point32 pt_shift = deputy_meas_cloud.points[i];
			meas_deputy_tmp.points.push_back(pt_shift);
			pt_shift.x = pt_shift.x + 10.0;
			meas_deputy_tmp.points.push_back(pt_shift);
		}
		for(size_t i=0; i<deputy_model_cloud.points.size(); i++)
		{
			geometry_msgs::Point32 pt_shift = deputy_model_cloud.points[i];
			model_deputy_tmp.points.push_back(pt_shift);
			pt_shift.x = pt_shift.x + 10.0;
			model_deputy_tmp.points.push_back(pt_shift);
		}

		deputy_meas_cloud.points  = meas_deputy_tmp.points;
		deputy_model_cloud.points = model_deputy_tmp.points;

		meas_deputy_pub_.publish(deputy_meas_cloud);
		model_deputy_pub_.publish(deputy_model_cloud);
		contour_cloud_debug_pub_.publish(model_cloud);
	}


	void vehicle_tracking::constructPtsMap(geometry_msgs::Pose &lidar_pose, sensor_msgs::PointCloud &segment_pointcloud, CSimplePointsMap &map)
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
	void vehicle_tracking::attached_points_transform(geometry_msgs::Point32& pt_input, geometry_msgs::Point32& pt_output, tf::Pose& oldMeas_poseinOdom, tf::Pose& newMeas_poseinOdom)
	{
		tf::Pose ptPose_inOdom, ptPose_inVehicle, ptPose_inOdom_new;
		ptPose_inOdom = tf::Transform(tf::Matrix3x3(tf::createIdentityQuaternion()), tf::Vector3(tfScalar(pt_input.x), tfScalar(pt_input.y), tfScalar(0)));
		ptPose_inVehicle = oldMeas_poseinOdom.inverse()*ptPose_inOdom;
		ptPose_inOdom_new = newMeas_poseinOdom*ptPose_inVehicle;
		pt_output.x = (float)ptPose_inOdom_new.getOrigin().getX();
		pt_output.y = (float)ptPose_inOdom_new.getOrigin().getY();
		pt_output.z = 0.0;
	}

	void vehicle_tracking::tracks_visualization()
	{
		sensor_msgs::PointCloud contour_pcl, anchor_pcl, filtered_anchor_pcl;
		contour_pcl.header.frame_id = "odom";
		anchor_pcl.header.frame_id = "odom";
		filtered_anchor_pcl.header.frame_id = "odom";
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
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vehicle_tracking_node");
	mrpt::vehicle_tracking vehicle_tracking_node;
	ros::spin();
	return (0);
}

