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
	ros::Subscriber people_roi_sub_;
	ros::Subscriber people_detect_sub_;
	ros::Subscriber people_verified_sub_;
	sensor_msgs::CvBridge bridge_;
	image_transport::Publisher image_pub_;
	void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr);
	void peopleRoiCallback(people_detector::people_rects pr);
	void peopleDetectCallback(people_detector::people_rects pr);
	void peopleVerifiedCallback(people_detector::people_rects pr);
	people_detector::people_rects roi_rects_;
	people_detector::people_rects detect_rects_;
	people_detector::people_rects verified_rects_;
	
	IplImage *cv_image;
	bool started;
};
};
