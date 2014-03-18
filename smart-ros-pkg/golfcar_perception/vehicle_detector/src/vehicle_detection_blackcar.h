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

#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace message_filters;
using namespace cv;
using namespace gpu;


struct vehicle_ROI
{
	int long_side;
	Rect original_ROI, ROI;
};

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
    void filterROIs(std::vector<Rect> & object_ROIs, std::vector<vehicle_ROI> & filtered_ROIs);

    void detectAndDrawObjects(std::vector<vehicle_ROI> & filtered_ROIs, LatentSvmDetector& detector, const vector<Scalar>& colors, float overlapThreshold, int numThreads );
    double detection_threshold_;

    LatentSvmDetector *LatentSVMdetector_;
    vector<Scalar> colors_;
    double overlap_threshold_;
    int threadNum_;

    tf::TransformListener tf_;

	tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub_;
	laser_geometry::LaserProjection                         projector_;
	ros::Publisher											beam_pub_, endPt_pub_;
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
	double angle_aperture_thresh_, vehicle_width_, vehicle_height_, max_detection_range_;
	int x_buff_, y_buff_;

    std::string camera_frame_id_;
    std::string laser_frame_id_;
	std::string base_frame_id_;
	std::string odom_frame_id_;
	std::string map_frame_id_;
	image_geometry::PinholeCameraModel cam_model_;
	bool camera_initialized_;
	boost::recursive_mutex configuration_mutex_;
	bool new_image_;
};

