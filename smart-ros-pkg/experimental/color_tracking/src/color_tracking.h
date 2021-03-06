
#ifndef COLOR_TRACKING_H
#define COLOR_TRACKING_H

//******************************************************************************
//******************************************************************************

//=============================================================================
// System Includes
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>

//=============================================================================
// OPENCV Includes
//=============================================================================
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>

//=============================================================================
// Other Includes
//=============================================================================
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>

//******************************************************************************
//******************************************************************************

// Some defines
using namespace std;
using namespace cv;

//******************************************************************************
//******************************************************************************


//Class declaration

class color_tracking {
  
public:
  color_tracking(ros::NodeHandle &n);
  ~color_tracking();
  
private:
  ros::NodeHandle private_nh_, n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  void imageCallback(const sensor_msgs::ImageConstPtr& image);
  cv::Mat GetThresholdedImage(Mat imgHSV, cv::Scalar lowerBound, cv::Scalar upperBound);
  cv::Point trackObject(Mat imgThresh);
  cv_bridge::CvImagePtr cv_image_;
  
  Mat frame0_, frame1_hsv_;
  Mat frame1_rgb_, frame2_rgb_;
  Mat imgThresh1_, imgThresh2_;
  
  cv::Point trackingPoint1, trackingPoint2;
  cv::Point lastPoint1, lastPoint2;
  geometry_msgs::Point trackingPoint1_ros, trackingPoint2_ros;
  
  ros::Publisher tracker_red, tracker_green;
};

#endif // COLOR_TRACKING_H