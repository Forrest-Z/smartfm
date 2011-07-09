#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include "people_detector/people_rects.h"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace HOG_Classifier{
class HOGClassifier {

public:


	HOGClassifier(ros::NodeHandle &n);
	~HOGClassifier();



private:
	std::vector<float> detector;
	cv::gpu::HOGDescriptor gpu_hog;
	cv::HOGDescriptor cpu_hog;
	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber people_rects_sub_;
	ros::Publisher people_roi_pub_;
	ros::Publisher people_detect_pub_;
	ros::Publisher people_ver_pub_;
	sensor_msgs::CvBridge bridge_;
	image_transport::Publisher image_pub_;
	void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr);
	void peopleRectsCallback(people_detector::people_rects pr);
	cv::Mat img;
	IplImage *cv_image;
	
	bool new_image;
};
};
