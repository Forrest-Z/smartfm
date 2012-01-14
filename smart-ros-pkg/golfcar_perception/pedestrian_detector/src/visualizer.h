#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensing_on_road/pedestrian_vision_batch.h"
#include "sensing_on_road/pedestrian_vision.h"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "classifier.h"
#include "image_transport/subscriber_filter.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensing_on_road;
using namespace message_filters;
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
	image_transport::SubscriberFilter image_sub_;
	message_filters::Subscriber<sensing_on_road::pedestrian_vision_batch> people_roi_sub_;
	message_filters::Subscriber<sensing_on_road::pedestrian_vision_batch> people_detect_sub_;
	message_filters::Subscriber<sensing_on_road::pedestrian_vision_batch> people_verified_sub_;
	//sensor_msgs::cv_bridge bridge_;
	image_transport::Publisher image_pub_;
	void peopleCallback(const sensor_msgs::ImageConstPtr image, const pedestrian_vision_batchConstPtr laser_detect, const pedestrian_vision_batchConstPtr vision_roi, const pedestrian_vision_batchConstPtr verified);
	void peopleRoiCallback(sensing_on_road::pedestrian_vision_batch pr);
	void peopleDetectCallback(sensing_on_road::pedestrian_vision_batch pr);
	void peopleVerifiedCallback(sensing_on_road::pedestrian_vision_batch pr);
	void drawIDandConfidence(cv::Mat& img, sensing_on_road::pedestrian_vision& pv);
	
	bool started, ROI_text, verified_text, vision_rect, verified_rect, ROI_rect;
};

