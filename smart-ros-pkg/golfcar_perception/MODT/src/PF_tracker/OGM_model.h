#ifndef MODT_OGM_MODEL_H
#define MODT_OGM_MODEL_H

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

class OGM_model
{
	public:
	cv::Point 	anchor_point;
	cv::Mat		grid_image;
	double		resolution;

	OGM_model()
	{
		tracker = new constspeed_ekf_tracker();
		track_status = 0;
		vehicle_evidence = 0;
	}

	double measurement_score()
	{

	}

	void update_model()
	{

	}

};

#endif
