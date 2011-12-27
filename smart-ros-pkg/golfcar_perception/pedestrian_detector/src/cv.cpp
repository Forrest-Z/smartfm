#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

class ImageConverter {

public:

ImageConverter(ros::NodeHandle &n) :
  n_(n), it_(n_)
{
  image_pub_ = it_.advertise("image_topic_2",1);

  cvNamedWindow("Image window");
  image_sub_ = it_.subscribe(
    "image_topic", 1, &ImageConverter::imageCallback, this);
}

~ImageConverter()
{
  cvDestroyWindow("Image window");
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{

  IplImage *cv_image = NULL;
  try
  {
    cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("error");
  }
	cv::Mat img(cv_image,false);/*
  cv::HOGDescriptor hog;
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    cv::vector<cv::Rect> found;
    double t = (double)cv::getTickCount();
    // run the detector with default parameters. to get a higher hit-rate
    // (and more false alarms, respectively), decrease the hitThreshold and
    // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
    hog.detectMultiScale(img, found, 0, cv::Size(8,8), cv::Size(24,16), 1.05, 2);
    t = (double)cv::getTickCount() - t;
    printf("Detection time = %gms\n", t*1000./cv::getTickFrequency());
    for( int i = 0; i < (int)found.size(); i++ )
    {
        cv::Rect r = found[i];
        // the HOG detector returns slightly larger rectangles than the real objects.
        // so we slightly shrink the rectangles to get a nicer output.
        r.x += cvRound(r.width*0.1);
        r.y += cvRound(r.height*0.1);
        r.width = cvRound(r.width*0.8);
        r.height = cvRound(r.height*0.8);
        cv::rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 1);
    }
*/
  imshow("Image window", img);
 // cvWaitKey(3);

  try
  {
    image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8"));
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("error");
  }

}

protected:

ros::NodeHandle n_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
sensor_msgs::CvBridge bridge_;
image_transport::Publisher image_pub_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;
  ImageConverter ic(n);
  ros::spin();
  return 0;
}
