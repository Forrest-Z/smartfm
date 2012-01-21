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

#include <ped_momdp_sarsop/peds_believes.h>

using namespace sensing_on_road;
using namespace message_filters;

struct pedBelife_vis
{
    double left_side, right_side;
    int decision;
    int id;
};

class HOGVisualizer {

public:


	HOGVisualizer(ros::NodeHandle &n);
	~HOGVisualizer();



private:
	std::vector<float> detector;
	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::SubscriberFilter image_sub_;
	message_filters::Subscriber<sensing_on_road::pedestrian_vision_batch> people_roi_sub_;
	ros::Subscriber ped_bel_sub_;
	image_transport::Publisher image_pub_;
	void peopleCallback(const sensor_msgs::ImageConstPtr image, const pedestrian_vision_batchConstPtr vision_roi);
	void drawIDandConfidence(cv::Mat& img, sensing_on_road::pedestrian_vision& pv);
	void pedBeliefCallback(ped_momdp_sarsop::peds_believes ped_bel);
	bool started, ROI_text, verified_text, vision_rect, verified_rect, ROI_rect;
	vector<pedBelife_vis>  ped_bel;
};

