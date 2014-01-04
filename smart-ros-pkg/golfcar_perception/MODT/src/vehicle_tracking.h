#ifndef MODT_VEHICLE_TRACKING_H
#define MODT_VEHICLE_TRACKING_H

#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include "MODT/segment_pose_batches.h"
#include <mrpt/slam.h>
#include <tf/transform_broadcaster.h>
/*
 * Pay attention here to remove irrelavant header files from ros;
 * It turns out that some header files will mess up the mrpt compilation;
 */

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;

namespace mrpt{
	
    class vehicle_tracking
    {
	     public:

    	//need a datatype to serve as the track history;


    	ros::Subscriber segpose_batch_sub_;
    	vehicle_tracking();
		~vehicle_tracking(){};

		void measurement_callback(const MODT::segment_pose_batches& batches);

		//basically data association step, but it can be removed in the futher with a smarter segmentation code;
		//since data association is already solved implicitly in the pcl segmentation part;
		void register_cluster2history(const MODT::segment_pose_batches& batches);

		//perform some tracker's basic operations;
		//1st: initiate new tracks; 2nd: merge and split; 3rd: update with measurements; 4th: delete obsolete tracks;
		void update_motionShape();

		//update object's position and shape using ICP;
		void ICP_motion2shape();
		void construct_ICP_scans(geometry_msgs::Pose &lidar_pose, sensor_msgs::PointCloud &segment_pointcloud, CObservation2DRangeScan& scan);
    };
};

#endif
