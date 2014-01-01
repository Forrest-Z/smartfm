#ifndef MODT_VEHICLE_POSE_ESTIMATION_H
#define MODT_VEHICLE_POSE_ESTIMATION_H

#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include "RANSAC_model/ransac_Lshape.hpp"
#include "MODT/segment_pose_batches.h"
/*
 * Pay attention here to remove irrelavant header files from ros;
 * It turns out that some header files will mess up the mrpt compilation;
 */

namespace mrpt{
	
    class vehicle_pose_estimation
    {
	     public:
    	ros::Subscriber segpose_batch_sub_;
    	vehicle_pose_estimation();
		~vehicle_pose_estimation(){};
		void estimate_pose(const MODT::segment_pose_batches& batches);
		void RANSAC_shape(sensor_msgs::PointCloud &input_cloud);
    };
};

#endif
