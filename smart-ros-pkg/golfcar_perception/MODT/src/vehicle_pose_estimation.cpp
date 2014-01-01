#include "vehicle_pose_estimation.h"

namespace mrpt{
	
	vehicle_pose_estimation::vehicle_pose_estimation()
	{
		ros::NodeHandle nh;
		segpose_batch_sub_  = nh.subscribe("segment_pose_batches", 1, &vehicle_pose_estimation::estimate_pose, this);
	}

	void vehicle_pose_estimation::estimate_pose(const MODT::segment_pose_batches& batches)
	{
		ROS_INFO("receive batches");
		sensor_msgs::PointCloud cloud_to_process;
		if(batches.clusters.size()==0) return;
		cloud_to_process = batches.clusters.back().segments.back();
		RANSAC_shape(cloud_to_process);
	}

	void vehicle_pose_estimation::RANSAC_shape(sensor_msgs::PointCloud &input_cloud)
	{
		ROS_INFO("point number: %ld", input_cloud.points.size());
		mrpt::dynamicsize_vector<double> xs, ys;
		for(size_t k=0; k<input_cloud.points.size(); k++)
		{
			xs.push_back( input_cloud.points[k].x);
			ys.push_back( input_cloud.points[k].y);
		}
		std::vector<std::pair<mrpt::vector_size_t, Lshape> >  Lshape_models;
		ransac_detect_Lshape(xs, ys, Lshape_models, 0.3, 5);
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vehicle_pose_estimation_node");
	mrpt::vehicle_pose_estimation vehicle_pose_estimation_node;
	ros::spin();
	return (0);
}

