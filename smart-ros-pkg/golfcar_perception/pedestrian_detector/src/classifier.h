#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "image_transport/subscriber_filter.h"
#include "cv_bridge/cv_bridge.h"
#include "sensing_on_road/pedestrian_vision_batch.h"
#include "sensing_on_road/pedestrian_vision.h"
#include "sensing_on_road/camera_project.h"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <feature_detection/clusters.h>
#include "cv_helper.h"
#include <geometry_msgs/PolygonStamped.h>
#define WIN_SIZE Size(48,96)

using namespace std;
using namespace cv;
using namespace gpu;

using namespace message_filters;
class HOGClassifier {

public:


    HOGClassifier(ros::NodeHandle &n);
    //HOGClassifier(){};
    ~HOGClassifier();
    
    bool fillRoiRectangle(Size img_size, Size* roi_size, Point* roi_point, sensing_on_road::pedestrian_vision& pd);
    void sensormsgsToCv(const sensor_msgs::ImageConstPtr& msg_ptr, Mat &img);
private:
    cv::HOGDescriptor cpu_hog;
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    message_filters::Subscriber<feature_detection::clusters> people_rects_sub_;
    ros::Publisher people_roi_pub_;
    ros::Publisher people_detect_pub_;
    ros::Publisher people_ver_pub_;
    ros::Publisher polygon_pub_;

    //sensor_msgs::CvBridge bridge_;
    image_transport::Publisher image_pub_;
    double getAngularDistance(double x);
    void ScaleWithDistanceRatio(Mat *img, double disz, double norm_distance, Size img_size, Size smallest_size, double *ratio);
    void detectPedestrian(Point offset, double ratio, gpu::GpuMat& gpu_img, sensing_on_road::pedestrian_vision_batch *detect_rects);
    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    void updateParameter();
    template <class T>
    void checkParamChanged(T &a, T &b);
    bool parameter_changed;
    cv::Mat img;
    int count;
    bool new_image, show_processed_image, black_front_roi,write_image;
    bool scaleWithDistance;
    double hit_threshold, scale, group_threshold, norm_dist;
    std::string path;
    std::string camera_base_id;

    //project camera
    sensing_on_road::pedestrian_vision_batch transformClusterToPedVisionBatch(const feature_detection::clustersConstPtr cl_ptr);
    camera_project::camera_projector camera_project;
    void setRectMsg(const camera_project::CvRectangle & rect, sensing_on_road::pedestrian_vision * msg)
    {
        msg->x          = rect.upper_left.x;
        msg->y          = rect.upper_left.y;
        msg->width      = rect.lower_right.x - rect.upper_left.x;
        msg->height     = rect.lower_right.y - rect.upper_left.y;

        msg->cvRect_x1  = rect.upper_left.x;
        msg->cvRect_y1  = rect.upper_left.y;
        msg->cvRect_x2  = rect.lower_right.x;
        msg->cvRect_y2  = rect.lower_right.y;
    }
};

