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


#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

using namespace std;
using namespace message_filters;
using namespace cv;
using namespace gpu;

class vehicle_detection {

public:
	vehicle_detection(ros::NodeHandle &n);
    ~vehicle_detection();

private:

    ros::NodeHandle private_nh_, n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_, edge_pub_;
    ros::Publisher vehicle_roi_pub_;
    ros::Publisher polygon_pub_;
    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    cv_bridge::CvImagePtr cv_image_;

    void detectAndDrawObjects( Mat& image, LatentSvmDetector& detector, const vector<Scalar>& colors, float overlapThreshold, int numThreads );
    LatentSvmDetector *LatentSVMdetector_;
    vector<Scalar> colors_;
    double overlap_threshold_;
    int threadNum_;
};

