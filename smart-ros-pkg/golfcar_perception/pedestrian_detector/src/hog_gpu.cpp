#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"

#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"
class HOGClissifier {

public:
std::vector<float> detector;
cv::gpu::HOGDescriptor gpu_hog;
cv::HOGDescriptor hog;
HOGClissifier(ros::NodeHandle &n) :
  n_(n), it_(n_)
{
  image_pub_ = it_.advertise("pedestrian_detector",1);

//  cvNamedWindow("Image window");

  image_sub_ = it_.subscribe(
    "image_raw", 1, &HOGClissifier::imageCallback, this);
    //initializing classifier
    detector = cv::gpu::HOGDescriptor::getDefaultPeopleDetector();
    //cv::gpu::HOGDescriptor gh(cv::Size(64,128), cv::Size(16,16), cv::Size(8,8), cv::Size(8,8),9, cv::gpu::HOGDescriptor::DEFAULT_WIN_SIGMA,
    //0.2, true, cv::gpu::HOGDescriptor::DEFAULT_NLEVELS);
    //gpu_hog = gh;
    gpu_hog.setSVMDetector(detector);
    gpu_hog.nlevels=18;
    //cv::HOGDescriptor ch(cv::Size(64,128), cv::Size(16,16), cv::Size(8,8), cv::Size(8,8),9, cv::gpu::HOGDescriptor::DEFAULT_WIN_SIGMA,
    //0.2, true, cv::gpu::HOGDescriptor::DEFAULT_NLEVELS);
    //hog=ch;
    hog.setSVMDetector(detector);
}

~HOGClissifier()
{
//  cvDestroyWindow("Image window");
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{

  IplImage *cv_image = NULL;
  try
  {
    cv_image = bridge_.imgMsgToCv(msg_ptr, "bgra8");
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("error");
  }
	cv::Mat img(cv_image);
	cv::resize(img, img, cv::Size(320, 240));
	
	
    
    cv::vector<cv::Rect> found;
    cv::gpu::GpuMat gpu_img(img);
    double t = (double)cv::getTickCount();
    // run the detector with default parameters. to get a higher hit-rate
    // (and more false alarms, respectively), decrease the hitThreshold and
    // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
    gpu_hog.detectMultiScale(gpu_img, found, 0, cv::Size(8,8), cv::Size(0,0), 1.05, 1);
    t = (double)cv::getTickCount() - t;
    printf("Detection time = %gms %d %d\n", t*1000./cv::getTickFrequency(), msg_ptr->width, msg_ptr->height);
    for( int i = 0; i < (int)found.size(); i++ )
    {
        cv::Rect r = found[i];
        cv::rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 1);
    }

  imshow("Image window", img);
  cvWaitKey(3);

  try
  {
    image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgra8"));
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
  ros::init(argc, argv, "hog_gpu");
  ros::NodeHandle n;
  HOGClissifier ic(n);
  ros::spin();
  return 0;
}
