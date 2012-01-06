#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensing_on_road/pedestrian_vision_batch.h"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"

class HOGVisualizer {

public:


	HOGVisualizer(ros::NodeHandle &n);
	~HOGVisualizer();



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
	//sensor_msgs::cv_bridge bridge_;
	image_transport::Publisher image_pub_;
	void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr);
	void peopleRoiCallback(sensing_on_road::pedestrian_vision_batch pr);
	void peopleDetectCallback(sensing_on_road::pedestrian_vision_batch pr);
	void peopleVerifiedCallback(sensing_on_road::pedestrian_vision_batch pr);
	sensing_on_road::pedestrian_vision_batch roi_rects_;
	sensing_on_road::pedestrian_vision_batch detect_rects_;
	sensing_on_road::pedestrian_vision_batch verified_rects_;
	
	
	bool started;
};

