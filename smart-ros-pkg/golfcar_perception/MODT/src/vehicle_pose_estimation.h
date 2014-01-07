#ifndef MODT_VEHICLE_POSE_ESTIMATION_H
#define MODT_VEHICLE_POSE_ESTIMATION_H

#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include "RANSAC_model/ransac_Lshape.hpp"
#include "MODT/segment_pose_batches.h"

#include <mrpt/slam.h>

#include <tf/transform_broadcaster.h>
/*
 * Pay attention here to remove irrelavant header files from ros;
 * It turns out that some header files will mess up the mrpt compilation;
 */
#define SIDE_LENGTH_THRESHOLD 0.5

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;

namespace mrpt{
	
    class vehicle_pose_estimation
    {
	     public:
    	ros::Subscriber segpose_batch_sub_;
    	ros::Publisher shape_pcl_pub_, polygon_pub_;
    	vehicle_pose_estimation();
		~vehicle_pose_estimation(){};
		void estimate_pose(const MODT::segment_pose_batches& batches);
		void RANSAC_shape2motion(sensor_msgs::PointCloud &input_cloud);

		void ICP_motion2shape(const MODT::segment_pose_batches& batches);
		void construct_ICP_scans(geometry_msgs::Pose &lidar_pose, sensor_msgs::PointCloud &segment_pointcloud, CObservation2DRangeScan& scan);
    };
};

#endif
