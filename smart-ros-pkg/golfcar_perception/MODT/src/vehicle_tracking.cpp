#include "vehicle_tracking.h"

namespace mrpt{
	
	vehicle_tracking::vehicle_tracking()
	{
		ros::NodeHandle nh;
		segpose_batch_sub_  = nh.subscribe("segment_pose_batches", 1, &vehicle_tracking::measurement_callback, this);
	}

	void vehicle_tracking::measurement_callback(const MODT::segment_pose_batches& batches)
	{
		ROS_INFO("measurement_callback");
		if(batches.clusters.size()==0) return;

		register_cluster2history(batches);
	}

	//Here just use a brutal-and-naive method to do data association;
	//later will improve it with a more efficient and safe method;
	void vehicle_tracking::register_cluster2history(const MODT::segment_pose_batches& batches)
	{
		//pair.first is track id, second is measure id;
		std::vector<std::pair<size_t, size_t> > track_measure_pairs;
		std::vector<size_t> unregistered_measurements;

		for(size_t i=0; i<batches.clusters.size(); i++)
		{
			MODT::segment_pose_batch &cluster_tmp = batches.clusters[i];
			bool registered_to_history = false;

			//chose the probe point at the last but two batch, which corresponds to the final batch in last measurement;
			geometry_msgs::Point32 probe_point_tmp = cluster_tmp.segments[cluster_tmp.segments.end()-1];
			for(size_t j=0; j<object_tracks_.size(); j++)
			{
				if(registered_to_history) break;
				sensor_msgs::PointCloud &last_pointcloud = object_tracks_[j].last_measurement.segments.back();
				for(size_t k=0; k<last_pointcloud.points.size(); k++)
				{
					geometry_msgs::Point32 &history_pt_tmp = last_pointcloud.points[k];
					float dist_between_pts = sqrtf((history_pt_tmp.x - probe_point_tmp.x)*(history_pt_tmp.x - probe_point_tmp.x)+(history_pt_tmp.y - probe_point_tmp.y)*(history_pt_tmp.y - probe_point_tmp.y));

					//actually they should be the same point;
					if(dist_between_pts< 0.01)
					{
						registered_to_history = true; break;
					}
				}
			}

			if(registered_to_history)
			{
				std::pair<size_t, size_t> pair_tmp = std::make_pair(j, i);
				track_measure_pairs.push_back(pair_tmp);
			}
			else
			{
				unregistered_measurements.push_back(j);
			}
		}
	}

	//not consider merge-split issues here;

	void vehicle_tracking::update_motionShape(const MODT::segment_pose_batches& batches, std::vector<std::pair<size_t, size_t> > &track_measure_pairs, std::vector<size_t> &unregistered_measurements)
	{
		//1st: update the associated tracks;
		for(size_t i=0; i<track_measure_pairs.size(); i++)
		{
			size_t track_serial = track_measure_pairs[i].first;
			size_t meas_serial	= track_measure_pairs[i].second;

			model_free_track &track_tmp = object_tracks_[track_serial];
			MODT::segment_pose_batch &lastbatch_tmp = track_tmp.last_measurement;
			MODT::segment_pose_batch &incoming_meas_tmp = batches[meas_serial];


		}

		//2nd: initiate new unassociated tracks;


		//3rd: delete old tracks;


	}

	void vehicle_tracking::ICP_motion2shape()
	{
		for(size_t i=0; i<batches.clusters.size(); i++)
		{
			//1st step: for a single cluster, try to estimate its motion using ICP;
			geometry_msgs::Pose oldest_pose = batches.clusters[i].ego_poses.front();
			geometry_msgs::Pose newest_pose = batches.clusters[i].ego_poses.back();
			sensor_msgs::PointCloud oldest_segment = batches.clusters[i].segments.front();
			sensor_msgs::PointCloud newest_segment = batches.clusters[i].segments.back();

			//please refer to my evernote 20140103 for more information;
			//a. calculate the rough estimation for the initial pose to speed up ICP;

			std::vector<geometry_msgs::Point32> centroid_position;
			for(size_t j=0; j<batches.clusters[i].segments.size(); j++)
			{
				geometry_msgs::Point32 centroid_tmp;
				centroid_tmp.x = 0.0;
				centroid_tmp.y = 0.0;
				centroid_tmp.z = 0.0;
				for(size_t k=0; k<batches.clusters[i].segments[j].points.size(); k++)
				{
					centroid_tmp.x = centroid_tmp.x + batches.clusters[i].segments[j].points[k].x;
					centroid_tmp.y = centroid_tmp.y + batches.clusters[i].segments[j].points[k].y;
				}
				centroid_tmp.x = centroid_tmp.x/(float)batches.clusters[i].segments[j].points.size();
				centroid_tmp.y = centroid_tmp.y/(float)batches.clusters[i].segments[j].points.size();
				centroid_position.push_back(centroid_tmp);
			}
			//build the transform from v_(t-1) to v_t;
			//about name convention: transform "Frame_A_To_Frame_B" is equal to "Frame_B_in_Frame_A";
			tf::Pose Odom_to_Vtm1 = tf::Transform(tf::Matrix3x3(tf::createIdentityQuaternion()), tf::Vector3(tfScalar(centroid_position.front().x), tfScalar(centroid_position.front().y), tfScalar(0)));
			tf::Pose Odom_to_Vt = tf::Transform(tf::Matrix3x3(tf::createIdentityQuaternion()), tf::Vector3(tfScalar(centroid_position.back().x), tfScalar(centroid_position.back().y), tfScalar(0)));
			tf::Pose Vtm1_to_Vt = Odom_to_Vtm1.inverse()*Odom_to_Vt;

			tf::Pose Odom_to_Etm1, Odom_to_Et, Etm1_to_Et;
			tf::poseMsgToTF(oldest_pose, Odom_to_Etm1);
			tf::poseMsgToTF(newest_pose, Odom_to_Et);
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
			CObservation2DRangeScan	scan1, scan2;
			construct_ICP_scans(oldest_pose, oldest_segment, scan1);
			construct_ICP_scans(newest_pose, newest_segment, scan2);

			CSimplePointsMap		m1,m2;
			float					runningTime;
			CICP::TReturnInfo		info;
			CICP					ICP;

			m1.insertObservation( &scan1 );
			m2.insertObservation( &scan2 );

			ICP.options.ICP_algorithm = icpClassic;
			ICP.options.maxIterations			= 100;
			ICP.options.thresholdAng			= DEG2RAD(10.0f);
			ICP.options.thresholdDist			= 0.75f;
			ICP.options.ALFA					= 0.5f;
			ICP.options.smallestThresholdDist	= 0.05f;
			ICP.options.doRANSAC = false;
			ICP.options.dumpToConsole();

			//CPose2D		initialPose(0.0f,0.0f,(float)DEG2RAD(0.0f));
			CPosePDFPtr pdf = ICP.Align(&m1, &m2,initialPose,&runningTime,(void*)&info);

			printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ",
					runningTime*1000,
					info.nIterations,
					runningTime*1000.0f/info.nIterations,
					info.goodness*100 );

			cout << "Mean of estimation: " << pdf->getMeanVal() << endl<< endl;

			CPosePDFGaussian  gPdf;
			gPdf.copyFrom(*pdf);

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
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vehicle_tracking_node");
	mrpt::vehicle_tracking vehicle_tracking_node;
	ros::spin();
	return (0);
}

