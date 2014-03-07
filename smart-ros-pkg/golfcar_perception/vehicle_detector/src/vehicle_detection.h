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
#include <sensor_msgs/PointCloud.h>
#include <image_geometry/pinhole_camera_model.h>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "MODT/segment_pose_batches.h"


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
    image_transport::CameraSubscriber cam_sub_;
    image_transport::Publisher image_pub_, edge_pub_;
    ros::Publisher vehicle_roi_pub_;
    ros::Publisher polygon_pub_;
    void imageCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info_msg);
    cv_bridge::CvImagePtr cv_image_;

    void detectAndDrawObjects( Mat& image, LatentSvmDetector& detector, const vector<Scalar>& colors, float overlapThreshold, int numThreads );
    LatentSvmDetector *LatentSVMdetector_;
    vector<Scalar> colors_;
    double overlap_threshold_;
    int threadNum_;

    tf::TransformListener tf_;
    ros::Subscriber segpose_batch_sub_;
    void lidarMeas_callback(const MODT::segment_pose_batches& batches);
    std::string camera_frame_id_;
    std::string laser_frame_id_;
	std::string base_frame_id_;
	std::string odom_frame_id_;
	std::string map_frame_id_;
	image_geometry::PinholeCameraModel cam_model_;
	bool camera_initialized_;

};

