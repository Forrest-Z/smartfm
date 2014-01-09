#include "vehicle_tracking_ICP.h"

namespace mrpt{
	
	vehicle_tracking::vehicle_tracking()
	{
		ros::NodeHandle nh;
		segpose_batch_sub_  = nh.subscribe("segment_pose_batches", 1, &vehicle_tracking::measurement_callback, this);
		contour_cloud_pub_	= nh.advertise<sensor_msgs::PointCloud>("contour_pcl", 2);
		anchor_point_pub_   = nh.advertise<sensor_msgs::PointCloud>("anchor_pcl", 2);
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
		}

		//2nd: initiate new unassociated tracks;
		for(size_t i=0; i<unregistered_measurements.size(); i++)
		{
			size_t meas_serial	= unregistered_measurements[i];
			MODT::segment_pose_batch &incoming_meas_tmp = batches.clusters[meas_serial];

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

			//the first anchor point;
			new_track_tmp.anchor_points.push_back(centroid_tmp);
			new_track_tmp.update_time = incoming_meas_tmp.segments.back().header.stamp;
			object_tracks_.push_back(new_track_tmp);
			object_total_id_++;
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

		/*
		for(size_t i=0; i<object_tracks_.size(); i++)
		{
			cout<<"object tracks last measurements "<<object_tracks_[i].last_measurement.segments.back().header.seq<<":";
			for(size_t j=0; j<object_tracks_[i].last_measurement.segments.back().points.size(); j++)
			{
				cout<<"("<<object_tracks_[i].last_measurement.segments.back().points[j].x<<","<<object_tracks_[i].last_measurement.segments.back().points[j].y<<")\t";
			}
			cout<<endl;
		}
		*/
	}

	void vehicle_tracking::ICP_motion2shape(model_free_track& track, MODT::segment_pose_batch& old_meas, MODT::segment_pose_batch& new_meas, tf::Pose& oldMeas_poseinOdom, tf::Pose& newMeas_poseinOdom )
	{
		//1st step: for a single cluster, try to estimate its motion using ICP;
		geometry_msgs::Pose old_pose = old_meas.ego_poses.back();
		geometry_msgs::Pose new_pose = new_meas.ego_poses.back();

		//sensor_msgs::PointCloud old_segment = old_meas.segments.back();
		sensor_msgs::PointCloud old_cloud = track.contour_points;
		sensor_msgs::PointCloud new_cloud = new_meas.segments.back();

		/*
		sensor_msgs::PointCloud old_cloud, new_cloud;
		for(int i= (int)old_meas.segments.size()-1, count=0; i>=0; i--, count++)
		{
			if(count%2!=0)continue;
			for(size_t j=0; j<old_meas.segments[i].points.size(); j++)
			{
				geometry_msgs::Point32 old_shifted;
				old_shifted = old_meas.segments[i].points[j];
				old_shifted.x = old_shifted.x + 10.0*count;
				old_cloud.points.push_back(old_shifted);
			}
		}

		for(int i= (int)new_meas.segments.size()-1, count = 0; i>=0; i--, count++)
		{
			if(count%2!=0)continue;
			for(size_t j=0; j<new_meas.segments[i].points.size(); j++)
			{
				geometry_msgs::Point32 new_shifted;
				new_shifted = new_meas.segments[i].points[j];
				new_shifted.x = new_shifted.x + 10.0*count;
				new_cloud.points.push_back(new_shifted);
			}
		}
		 */

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
		centroid_tmp.z = 0.0;
		for(size_t k=0; k<new_cloud.points.size(); k++)
		{
			centroid_tmp.x = centroid_tmp.x + new_cloud.points[k].x;
			centroid_tmp.y = centroid_tmp.y + new_cloud.points[k].y;
		}
		centroid_tmp.x = centroid_tmp.x/(float)new_cloud.points.size();
		centroid_tmp.y = centroid_tmp.y/(float)new_cloud.points.size();
		centroid_position.push_back(centroid_tmp);

		//build the transform from v_(t-1) to v_t;
		//about name convention: transform "Frame_A_To_Frame_B" is equal to "Frame_B_in_Frame_A";
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

		//b. construct two scans based on the "pose + pointcloud";
		//CObservation2DRangeScan	scan1, scan2;
		//construct_ICP_scans(old_pose, old_segment, scan1);
		//construct_ICP_scans(new_pose, new_segment, scan2);

		// a trick to use ICP to get zero-degree rotation :-)
		sensor_msgs::PointCloud old_cloud_shiftAdded, new_cloud_shiftAdded;
		for(size_t i=0; i<old_cloud.points.size(); i++)
		{
			geometry_msgs::Point32 pt_shift = old_cloud.points[i];
			old_cloud_shiftAdded.points.push_back(pt_shift);
			pt_shift.x = pt_shift.x + 10.0;
			old_cloud_shiftAdded.points.push_back(pt_shift);
		}

		for(size_t i=0; i<new_cloud.points.size(); i++)
		{
			geometry_msgs::Point32 pt_shift = new_cloud.points[i];
			new_cloud_shiftAdded.points.push_back(pt_shift);
			pt_shift.x = pt_shift.x + 10.0;
			new_cloud_shiftAdded.points.push_back(pt_shift);
		}


		CSimplePointsMap		m1, m2, m1_shiftAdded, m2_shiftAdded;
		float					runningTime;
		CICP::TReturnInfo		info;
		CICP					ICP;
		constructPtsMap(old_pose, old_cloud_shiftAdded, m1_shiftAdded);
		constructPtsMap(new_pose, new_cloud_shiftAdded, m2_shiftAdded);

		constructPtsMap(old_pose, old_cloud, m1);
		constructPtsMap(new_pose, new_cloud, m2);

		//m1.insertObservation( &scan1 );
		//m2.insertObservation( &scan2 );

		ICP.options.ICP_algorithm = icpLevenbergMarquardt;
		ICP.options.maxIterations			= 100;
		ICP.options.thresholdAng			= DEG2RAD(10.0f);
		ICP.options.thresholdDist			= 0.75f;
		ICP.options.ALFA					= 0.5f;
		ICP.options.smallestThresholdDist	= 0.01f;
		ICP.options.doRANSAC = false;
		ICP.options.onlyClosestCorrespondences = false;
		ICP.options.dumpToConsole();

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
		cout<<"ICP output: "<<ICP_output_x<<","<<ICP_output_y<<","<<ICP_output_yaw<<endl;

		tf::Pose ICP_Etm1_to_Etvirtual = tf::Transform(tf::Matrix3x3(tf::createQuaternionFromYaw(ICP_output_yaw)), tf::Vector3(tfScalar(ICP_output_x), tfScalar(ICP_output_y), tfScalar(0)));

		tf::Pose ICP_Et_To_Etvirtual = Etm1_to_Et.inverse()*ICP_Etm1_to_Etvirtual;
		tf::Pose ICP_Vtm1_To_Vt = Vtm1_to_Etvirtual*ICP_Et_To_Etvirtual.inverse()*Vt_to_Et.inverse();

		double vehicle_delta_x = ICP_Vtm1_To_Vt.getOrigin().getX();
		double vehicle_delta_y = ICP_Vtm1_To_Vt.getOrigin().getY();
		double vehicle_delta_yaw = tf::getYaw(ICP_Vtm1_To_Vt.getRotation());
		cout<<"vehicle motion: "<<vehicle_delta_x<<","<<vehicle_delta_y<<","<<vehicle_delta_yaw<<endl;

		newMeas_poseinOdom = Odom_to_Vt;
		oldMeas_poseinOdom = Odom_to_Vt*ICP_Vtm1_To_Vt.inverse();
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

	void vehicle_tracking::construct_ICP_scans(geometry_msgs::Pose &lidar_pose, sensor_msgs::PointCloud &segment_pointcloud, CObservation2DRangeScan& scan)
	{
		//1st step: define the basic information of the scan reading, in the format of MRPT datatype;
		//give lagest possible aperture to deal with all kinds of possible readings;
		scan.aperture = 2*M_PIf;
		scan.rightToLeft = true;
		//assuming finest possible accuracy is 0.5 degree (remember to change to rad);
		float resolution = 0.5/180.0*M_PIf;
		int scan_size =721;
		scan.validRange.resize( scan_size, 0);
		scan.scan.resize(scan_size, 0.0);

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

			float range_tmp = sqrtf(pointtemp.x*pointtemp.x+pointtemp.y*pointtemp.y);
			float angle_tmp = atan2f(pointtemp.y, pointtemp.x);
			float float_serial = (angle_tmp+scan.aperture/2.0 + resolution/2.0)/resolution;
			int serial_tmp = (int)floor(float_serial);

			//to deal with possible rounding issue;
			if(serial_tmp<0)serial_tmp=0;
			if(serial_tmp>scan_size-1)serial_tmp=scan_size-1;

			scan.validRange[serial_tmp] = 1;
			scan.scan[serial_tmp] = range_tmp;
			//cout<<"[x, y, angle, serial, range]"<<pointtemp.x<<","<<pointtemp.y<<","<<angle_tmp<<","<<serial_tmp<<","<<range_tmp<<"\t";
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
		sensor_msgs::PointCloud contour_pcl, anchor_pcl;
		contour_pcl.header.frame_id = "odom";
		anchor_pcl.header.frame_id = "odom";
		contour_pcl.header.stamp = latest_input_time_;
		anchor_pcl.header.stamp = latest_input_time_;

		for(size_t i=0; i<object_tracks_.size(); i++)
		{
			bool visualize_flag = (latest_input_time_-object_tracks_[i].update_time).toSec()<0.001;

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
			}
		}

		contour_cloud_pub_.publish(contour_pcl);
		anchor_point_pub_.publish(anchor_pcl);
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vehicle_tracking_node");
	mrpt::vehicle_tracking vehicle_tracking_node;
	ros::spin();
	return (0);
}

