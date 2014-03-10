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
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

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
    cv_bridge::CvImagePtr cv_image_, cv_image_copy_;

    void imageCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info_msg);
    void lidarMeas_callback(const MODT::segment_pose_batches& batches);
    void calcROIs(std::vector<sensor_msgs::PointCloud> & object_clusters, std::vector<Rect> & object_ROIs);
    double x_inflat_dist_, y_inflat_dist_;
    double z_low_bound_, z_high_bound_;

    void detectAndDrawObjects( Mat& image, LatentSvmDetector& detector, const vector<Scalar>& colors, float overlapThreshold, int numThreads );

    LatentSvmDetector *LatentSVMdetector_;
    vector<Scalar> colors_;
    double overlap_threshold_;
    int threadNum_;

    tf::TransformListener tf_;
    ros::Subscriber segpose_batch_sub_;

    std::string camera_frame_id_;
    std::string laser_frame_id_;
	std::string base_frame_id_;
	std::string odom_frame_id_;
	std::string map_frame_id_;
	image_geometry::PinholeCameraModel cam_model_;
	bool camera_initialized_;
	tf::MessageFilter<sensor_msgs::CameraInfo>				*tf_filter_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> 	*camera_info_sub_;
	void cameraInfo_callback(const sensor_msgs::CameraInfoConstPtr& camera_info_ptr);

	boost::recursive_mutex configuration_mutex_;
	bool new_image_;
};

