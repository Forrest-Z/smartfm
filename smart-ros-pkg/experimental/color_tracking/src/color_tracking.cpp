#include "color_tracking.h"

color_tracking::color_tracking(ros::NodeHandle &n) : private_nh_("~"), n_(n), it_(n)
{
  image_sub_ = it_.subscribe("/camera_sys76/image_raw", 1, &color_tracking::imageCallback, this);
  image_pub_ = it_.advertise("/camera_sys76/tracking_result", 1);
  
  tracker_red = n.advertise<geometry_msgs::Point>("/tracker_red", 1);
  tracker_green = n.advertise<geometry_msgs::Point>("/tracker_green", 1);
  
  cv::namedWindow("CameraOutput",WINDOW_AUTOSIZE);
  cv::namedWindow("Object_Red",WINDOW_AUTOSIZE);
  cv::namedWindow("Object_Green",WINDOW_AUTOSIZE);
  
  trackingPoint1 = cv::Point(-1.0,-1.0);
  trackingPoint2 = cv::Point(-1.0,-1.0);
  lastPoint1 = cv::Point(-1.0,-1.0);
  lastPoint2 = cv::Point(-1.0,-1.0);
}

color_tracking::~color_tracking()
{
  destroyAllWindows();
}

Mat color_tracking::GetThresholdedImage(Mat imgHSV, cv::Scalar lowerBound, cv::Scalar upperBound)
{       
  cv::Mat imgThresh = cv::Mat(imgHSV.size(),CV_8UC1);
  cv::inRange(imgHSV, lowerBound, upperBound, imgThresh);
  return imgThresh;
}

cv::Point color_tracking::trackObject(Mat imgThresh){
    // Calculate the moments of 'imgThresh'
    cv::Moments img_moments = cv::moments(imgThresh, true);
    double moment10 = img_moments.m10;
    double moment01 = img_moments.m01;
    double area = img_moments.m00;

    cv::Point pos(0.0,0.0);
     // if the area<1000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
    if(area>500){
        // calculate the position of the ball
        pos.x = moment10/area;
        pos.y = moment01/area;       
    }
     
     return pos;
}

void color_tracking::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
  cv::Mat flow_rgb, imgThresh1, imgThresh2, img_hsv;
  
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image, "bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  cv_image_->image.copyTo(frame1_rgb_);
  cv::cvtColor(frame1_rgb_, frame1_hsv_, CV_BGR2HSV);
  frame1_hsv_.copyTo(img_hsv);
  
  //Do thresholding and track :: Red
  imgThresh1_ = color_tracking::GetThresholdedImage(img_hsv, cv::Scalar(15,37,121), cv::Scalar(24,256,256));
  trackingPoint1 = color_tracking::trackObject(imgThresh1_);
  trackingPoint1_ros.x = trackingPoint1.x;
  trackingPoint1_ros.y = trackingPoint1.y;
  //Draw line
  if(trackingPoint1.x>0 && trackingPoint1.y>0 && lastPoint1.x>0 && lastPoint1.y>0)
  {
    // Draw a line from the previous point to the current point
    line(frame1_rgb_, trackingPoint1, lastPoint1, cv::Scalar(0,0,255), 1);
  }
  
  //Do thresholding and track :: Green
  imgThresh2_ = color_tracking::GetThresholdedImage(img_hsv, cv::Scalar(28,16,100), cv::Scalar(75,51,256));
  trackingPoint2 = color_tracking::trackObject(imgThresh2_);
  trackingPoint2_ros.x = trackingPoint2.x;
  trackingPoint2_ros.y = trackingPoint2.y;
  //Draw line
  if(trackingPoint2.x>0 && trackingPoint2.y>0 && lastPoint2.x>0 && lastPoint2.y>0)
  {
    // Draw a line from the previous point to the current point
    line(frame1_rgb_, trackingPoint2, lastPoint2, cv::Scalar(0,255,0), 1);
  }
  
  imgThresh1_.copyTo(imgThresh1);
  cv::imshow("Object_Red",imgThresh1);
  
  imgThresh2_.copyTo(imgThresh2);
  cv::imshow("Object_Green",imgThresh2);
  
  frame1_rgb_.copyTo(flow_rgb);
  cv::imshow("CameraOutput",flow_rgb);
  
  cv_image_->image = flow_rgb;
  image_pub_.publish(cv_image_->toImageMsg());
  
  tracker_red.publish(trackingPoint1_ros);
  tracker_green.publish(trackingPoint2_ros);
  lastPoint1 = trackingPoint1;
  lastPoint2 = trackingPoint2;
  cv:: waitKey(3);
}

int main(int argc, char** argv)
{
  std::cout << "Hello there!" << std::endl;
  ros::init(argc, argv, "color_tracking");
  ros::NodeHandle n;
  color_tracking color_tracking_node(n);
  ros::spin();
  return 0;
}