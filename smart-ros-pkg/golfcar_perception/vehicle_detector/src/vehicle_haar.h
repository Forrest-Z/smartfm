#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "image_transport/subscriber_filter.h"
#include <cv_bridge/CvBridge.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <feature_detection/clusters.h>
#include <geometry_msgs/PolygonStamped.h>

using namespace std;

using namespace message_filters;

class HAARClassifier {

public:
	HAARClassifier(ros::NodeHandle &n);
    ~HAARClassifier();

private:

    ros::NodeHandle private_nh_, n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    sensor_msgs::CvBridge bridge_;
    ros::Publisher vehicle_roi_pub_;
    ros::Publisher polygon_pub_;

    //sensor_msgs::CvBridge bridge_;
    image_transport::Publisher image_pub_;

    double getAngularDistance(double x);
    void detectVehicle(IplImage* frame, vector<CvRect> & vehicle_rects);
    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    void updateParameter();
    template <class T>
    void checkParamChanged(T &a, T &b);
    bool parameter_changed, show_processed_image;

    int count;
    int						input_resize_percent;
    std::string				cascade_path;
    CvHaarClassifierCascade *cascade;
    CvMemStorage            *storage;
};

