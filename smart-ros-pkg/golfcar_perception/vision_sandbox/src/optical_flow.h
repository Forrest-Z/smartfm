#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "image_transport/subscriber_filter.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv/cv.h>
#include "opencv2/gpu/gpu.hpp"
#include <opencv/highgui.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PolygonStamped.h>

#include "farneback_flow.h"

using namespace std;
using namespace message_filters;
using namespace cv;
using namespace gpu;

class optical_flower {

public:
	optical_flower(ros::NodeHandle &n);
    ~optical_flower();

private:

    ros::NodeHandle private_nh_, n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    cv_bridge::CvImagePtr cv_image_;

    cv::gpu::FarnebackOpticalFlow dflow;

    Mat frame0_, frame1_;
    Mat frame0_rgb_, frame1_rgb_;

    GpuMat frame0GPU, frame1GPU, uGPU, vGPU;
    Mat imgU, imgV;

};

