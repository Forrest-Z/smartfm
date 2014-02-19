#ifndef MODT_OGM_MODEL_H
#define MODT_OGM_MODEL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

class OGM_model
{
	public:
	//this anchor point is a fixed point in the image;
	cv::Point 	anchor_point;
	cv::Mat		grid_image;

	//this is the extracted map points in the space coordinate attached to the anchor point;
	std::vector<geometry_msgs::Point32> map_points;
	double		resolution;

	OGM_model(std::string param_file)
	{


	}

	void load_parameters()
	{

	}

	void init_model(std::vector<geometry_msgs::Point32> initial_cloud)
	{

	}

	double measurement_score(geometry_msgs::Pose anchor_particle, sensor_msgs::PointCloud meas_input)
	{
		//1st: convert the "map_points" into the LIDAR coordinate;


		//2nd: calculate the measurement score for "meas_input";

	}

	void update_model(geometry_msgs::Pose anchor_particle, sensor_msgs::PointCloud meas_input)
	{
		//1st: transform "meas_input" into anchor-point coordinate;

		//2nd: update the "grid_image", and extract new map_points;

	}

	//extract map points from the "grid_image", by thresholding the image value;
	void extract_map_points()
	{

	}

};

#endif
