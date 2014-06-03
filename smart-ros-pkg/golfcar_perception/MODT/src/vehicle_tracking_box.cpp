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
		ROS_INFO("deleted track num %ld", deleted_track_num);

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

		sensor_msgs::PointCloud current_centroids, prev_centroids;
		cout<<"current_centroids:";
		for(size_t i=0; i<batches.clusters.size(); i++)
		{
			MODT::segment_pose_batch &cluster_tmp = batches.clusters[i];
			geometry_msgs::Point32 current_centroid;
			current_centroid.x = 0.0;
			current_centroid.y = 0.0;
			current_centroid.z = 0.0;
			int current_total_point_num = 0;
			for(size_t j=0; j<cluster_tmp.segments.size(); j++)
			{
				for(size_t k=0; k<cluster_tmp.segments[j].points.size(); k++)
				{
					current_centroid.x = current_centroid.x + cluster_tmp.segments[j].points[k].x;
					current_centroid.y = current_centroid.y + cluster_tmp.segments[j].points[k].y;
					current_total_point_num ++;
				}
			}
			current_centroid.x = current_centroid.x/(float)current_total_point_num;
			current_centroid.y = current_centroid.y/(float)current_total_point_num;
			current_centroids.points.push_back(current_centroid);
			cout<<"("<<current_centroid.x<<","<<current_centroid.y<<")\t";
		}
		cout<<endl;

		cout<<"prev_centroids:";
		for(size_t j=0; j<object_tracks_.size(); j++)
		{
			geometry_msgs::Point32 current_centroid;
			current_centroid.x = 0.0;
			current_centroid.y = 0.0;
			current_centroid.z = 0.0;
			int current_total_point_num = 0;
			for(size_t a = 0; a<object_tracks_[j].last_measurement.segments.size(); a++)
				for(size_t b=0; b<object_tracks_[j].last_measurement.segments[a].points.size(); b++)
				{
					current_centroid.x = current_centroid.x + object_tracks_[j].last_measurement.segments[a].points[b].x;
					current_centroid.y = current_centroid.y + object_tracks_[j].last_measurement.segments[a].points[b].y;
					current_total_point_num ++;
				}
			current_centroid.x = current_centroid.x/(float)current_total_point_num;
			current_centroid.y = current_centroid.y/(float)current_total_point_num;
			prev_centroids.points.push_back(current_centroid);
			cout<<"("<<current_centroid.x<<","<<current_centroid.y<<")\t";
		}
		cout<<endl;

		for(size_t i=0; i<batches.clusters.size(); i++)
		{
			geometry_msgs::Point32 &curr_centroid_tmp = current_centroids.points[i];
			bool registered_to_history = false;
			size_t associated_track_id = 0;
			double nearest_distance = DBL_MAX;

			for(size_t j=0; j<object_tracks_.size(); j++)
			{
				geometry_msgs::Point32 &prev_centroid_tmp = prev_centroids.points[j];

				float dist_between_pts = sqrtf((prev_centroid_tmp.x - curr_centroid_tmp.x)*(prev_centroid_tmp.x - curr_centroid_tmp.x)+(prev_centroid_tmp.y - curr_centroid_tmp.y)*(prev_centroid_tmp.y - curr_centroid_tmp.y));
				if(dist_between_pts<nearest_distance){nearest_distance = dist_between_pts; associated_track_id = j;}

				if(nearest_distance< 1.0)
				{
					registered_to_history = true;
					break;
				}
			}
			cout<<nearest_distance<<","<<i<<","<<associated_track_id<<"\n";

			if(registered_to_history)
			{
				std::pair<size_t, size_t> pair_tmp = std::make_pair(associated_track_id, i);
				track_measure_pairs.push_back(pair_tmp);
			}
			else
			{
				cout<<"segment not registered"<<i<<endl;
				unregistered_measurements.push_back(i);
			}
		}

		maintain_tracks(batches, track_measure_pairs, unregistered_measurements);
	}

	//not consider merge-split issues here;
	void vehicle_tracking_box::maintain_tracks(MODT::segment_pose_batches& batches, std::vector<std::pair<size_t, size_t> > &track_measure_pairs, std::vector<size_t> &unregistered_measurements)
	{
		//1st: update the associated tracks;
		for(size_t i=0; i<track_measure_pairs.size(); i++)
		{
			size_t track_serial = track_measure_pairs[i].first;
			size_t meas_serial	= track_measure_pairs[i].second;

			box_model_track &track_tmp = object_tracks_[track_serial];
			MODT::segment_pose_batch &lastbatch_tmp = track_tmp.last_measurement;
			MODT::segment_pose_batch &incoming_meas_tmp = batches.clusters[meas_serial];

			double delt_x, delt_y;
			motion_estimation(incoming_meas_tmp, delt_x, delt_y);
			double current_moving_direction = atan2(delt_y, delt_x);

			box_model measure_box;
			calculate_measurement_box(incoming_meas_tmp, current_moving_direction, measure_box);

			//to update the model box using the measurement box;
			update_box_track(track_tmp, measure_box);

			/*
			//transform "anchor points" and "contour points", and to visualize the results;
			geometry_msgs::Point32 old_anchor_point = track_tmp.anchor_points.back();
			geometry_msgs::Point32 new_anchor_point;
			attached_points_transform(old_anchor_point, new_anchor_point, oldMeas_poseinOdom, newMeas_poseinOdom);
			ROS_INFO("old anchor: (%3f, %3f); new anchor: (%3f, %3f)", old_anchor_point.x, old_anchor_point.y, new_anchor_point.x, new_anchor_point.y);
			track_tmp.anchor_points.push_back(new_anchor_point);
			*/
			geometry_msgs::Point32 new_anchor_point = track_tmp.anchor_points.back();
			geometry_msgs::Point32 old_anchor_point = track_tmp.anchor_points[track_tmp.anchor_points.size()-2];

			double anchor_pt_dist = sqrt((new_anchor_point.x-old_anchor_point.x)*(new_anchor_point.x-old_anchor_point.x)+(new_anchor_point.y-old_anchor_point.y)*(new_anchor_point.y-old_anchor_point.y));
			double time_difference = (incoming_meas_tmp.segments.back().header.stamp - lastbatch_tmp.segments.back().header.stamp).toSec();
			track_tmp.moving_direction = atan2(new_anchor_point.y-old_anchor_point.y, new_anchor_point.x-old_anchor_point.x);
			track_tmp.velocity = anchor_pt_dist/time_difference;

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

			track_tmp.last_measurement = incoming_meas_tmp;
			track_tmp.update_time = incoming_meas_tmp.segments.back().header.stamp;
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

			new_track_tmp.tracker->set_params(3.0, 3.0, 3.0, 3.0, M_PI*0.2, 0.3, 0.3, M_PI*100);
			new_track_tmp.tracker->init_filter(centroid_tmp.x, centroid_tmp.y, vx_value, vy_value, 0.0);
			new_track_tmp.update_object_belief(true);

			double delt_x, delt_y;
			motion_estimation(incoming_meas_tmp, delt_x, delt_y);
			double current_moving_direction = atan2(delt_y, delt_x);
			box_model measure_box;
			calculate_measurement_box(incoming_meas_tmp, current_moving_direction, measure_box);
			update_box_track(new_track_tmp, measure_box);

			object_tracks_.push_back(new_track_tmp);
			object_total_id_++;
		}

		ROS_INFO("updated tracks: %ld; new tracks: %ld; remained tracks: %ld", track_measure_pairs.size(), unregistered_measurements.size(), object_tracks_.size());
	}

	void vehicle_tracking_box::motion_estimation(MODT::segment_pose_batch& new_meas, double &delt_x, double &delt_y)
	{
		//sensor_msgs::PointCloud old_cloud = track.contour_points;
		sensor_msgs::PointCloud new_cloud, old_cloud;

		int batch_number_tmp = int(new_meas.segments.size())/(int)2;

		int batch_serial=0;
		for(size_t i=0; batch_serial<batch_number_tmp; i=i+2, batch_serial++)
			for(size_t j=0; j<new_meas.segments[i].points.size(); j++)
			{
				new_cloud.points.push_back(new_meas.segments[i].points[j]);
			}

		batch_serial=0;
		for(size_t i=1; batch_serial<batch_number_tmp; i=i+2, batch_serial++)
			for(size_t j=0; j<new_meas.segments[i].points.size(); j++)
			{
				old_cloud.points.push_back(new_meas.segments[i].points[j]);
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

		centroid_tmp.x = 0.0;
		centroid_tmp.y = 0.0;
		for(size_t k=0; k<new_cloud.points.size(); k++)
		{
			centroid_tmp.x = centroid_tmp.x + new_cloud.points[k].x;
			centroid_tmp.y = centroid_tmp.y + new_cloud.points[k].y;
		}
		centroid_tmp.x = centroid_tmp.x/(float)new_cloud.points.size();
		centroid_tmp.y = centroid_tmp.y/(float)new_cloud.points.size();
		centroid_position.push_back(centroid_tmp);

		delt_x = centroid_position.back().x - centroid_position.front().x;
		delt_y = centroid_position.back().y - centroid_position.front().y;
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

			measurement_box.corner_points[i] = intersect_pt[i];
			measPt_lidarAngle(lidar_pose, measurement_box.corner_points[i], measurement_box.lidar_angle[i]);
		}

		measurement_box.moving_direction = current_moving_direction;
		measurement_box.refPt_seial = 0;
		measurement_box.width = sqrtf((intersect_pt[0].x-intersect_pt[1].x)*(intersect_pt[0].x-intersect_pt[1].x)+(intersect_pt[0].y-intersect_pt[1].y)*(intersect_pt[0].y-intersect_pt[1].y));
		measurement_box.length = sqrtf((intersect_pt[0].x-intersect_pt[3].x)*(intersect_pt[0].x-intersect_pt[3].x)+(intersect_pt[0].y-intersect_pt[3].y)*(intersect_pt[0].y-intersect_pt[3].y));
		meas_polygon_pub_.publish(meas_polygon);
	}

	//bugs inside;
	void vehicle_tracking_box::update_box_track(box_model_track &track, box_model &measurement_box)
	{
		ROS_WARN("update box track %d", track.object_id);

		if(measurement_box.length<0.001 || measurement_box.width<0.001)
		{
			ROS_WARN("measurement box too small, unreliable");
		}

		if(track.shape.init==false)
		{
			track.shape = measurement_box;
			track.shape.init=true;
		}
		else
		{
			//1st: update the shape model with new measurement box;
			geometry_msgs::Point32 new_points[4];
			for(size_t i=0; i<4; i++) new_points[i]=measurement_box.corner_points[i];

			//take into account limited FOV;
			/*
			if(fabs(measurement_box.lidar_angle[0])> M_PI*(130.0/180.0) && fabs(measurement_box.lidar_angle[3])< M_PI*(130.0/180.0))
			{
				geometry_msgs::Point32 point_tmp;
				point_tmp = new_points[0];
				new_points[0] = new_points[3];
				new_points[3] = point_tmp;
				point_tmp = new_points[1];
				new_points[1] = new_points[2];
				new_points[2] = point_tmp;
			}
			*/

			geometry_msgs::Point32 shape_points[4];
			shape_points[0]=new_points[0];

			double scale_ratio_width = track.shape.width/measurement_box.width > 1.0 ? track.shape.width/measurement_box.width: 1.0;
			double scale_ratio_length = track.shape.length/measurement_box.length > 1.0 ? track.shape.length/measurement_box.length:1.0;

			cout<<"track shape"<<" "<<track.shape.width<<" "<<track.shape.length<<" "<<endl;
			cout<<"measurement_box"<<" "<<measurement_box.width<<" "<<measurement_box.length<<" "<<endl;
			cout<<"scale ration"<<" "<<scale_ratio_width<<" "<<scale_ratio_length<<endl;

			track.shape.width = measurement_box.width*scale_ratio_width;
			track.shape.length = measurement_box.length*scale_ratio_length;

			shape_points[1].x = shape_points[0].x + scale_ratio_width *(new_points[1].x-new_points[0].x);
			shape_points[1].y = shape_points[0].y + scale_ratio_width *(new_points[1].y-new_points[0].y);
			shape_points[2].x = shape_points[1].x + scale_ratio_length *(new_points[2].x-new_points[1].x);
			shape_points[2].y = shape_points[1].y + scale_ratio_length *(new_points[2].y-new_points[1].y);
			shape_points[3].x = shape_points[0].x + scale_ratio_length *(new_points[3].x-new_points[0].x);
			shape_points[3].y = shape_points[0].y + scale_ratio_length *(new_points[3].y-new_points[0].y);
			for(size_t i=0; i<4; i++) track.shape.corner_points[i]=shape_points[i];

			//2nd: find the nearest point to the old anchor point;
			/*
			geometry_msgs::Point32 prev_anchorPt = track.anchor_points.back();
			size_t nearest_serial = 0;
			double shortest_dist = DBL_MAX;
			for(size_t i=0; i<4; i++)
			{
				geometry_msgs::Point32 &shape_pt_tmp = shape_points[i];
				double dist_tmp = sqrt((shape_pt_tmp.x-prev_anchorPt.x)*(shape_pt_tmp.x-prev_anchorPt.x)+(shape_pt_tmp.y-prev_anchorPt.y)*(shape_pt_tmp.y-prev_anchorPt.y));
				nearest_serial = dist_tmp<shortest_dist? i:nearest_serial;
			}
			track.anchor_points.push_back(shape_points[nearest_serial]);
			*/

			geometry_msgs::Point32 shape_centroid;
			for(size_t i=0; i<4; i++)
			{
				shape_centroid.x = shape_centroid.x + shape_points[i].x;
				shape_centroid.y = shape_centroid.y + shape_points[i].y;
			}
			shape_centroid.x = shape_centroid.x/4.0;
			shape_centroid.y = shape_centroid.y/4.0;
			track.anchor_points.push_back(shape_centroid);
		}
	}

	void vehicle_tracking_box::measPt_lidarAngle(geometry_msgs::Pose &lidar_pose, geometry_msgs::Point32 measPt, float &lidarAngle)
	{
		tf::Pose lidarTFPose;
		tf::poseMsgToTF(lidar_pose, lidarTFPose);
		geometry_msgs::Pose temppose;
		temppose.position.x = measPt.x;
		temppose.position.y = measPt.y;
		temppose.position.z = measPt.z;
		temppose.orientation.x=0;
		temppose.orientation.y=0;
		temppose.orientation.z=0;
		temppose.orientation.w=1;
		tf::Pose tempTfPose;
		tf::poseMsgToTF(temppose, tempTfPose);

		tf::Pose pointInlidar = lidarTFPose.inverseTimes(tempTfPose);
		geometry_msgs::Point32 pointtemp;
		pointtemp.x=(float)pointInlidar.getOrigin().x();
		pointtemp.y=(float)pointInlidar.getOrigin().y();
		float pt_angle = (float)atan2(pointtemp.y, pointtemp.x);
		lidarAngle = pt_angle;
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
				for(size_t j=0; j<4; j++)
				{
					contour_pcl.points.push_back(object_tracks_[i].shape.corner_points[j]);
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

