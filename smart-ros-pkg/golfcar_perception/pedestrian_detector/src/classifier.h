#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "image_transport/subscriber_filter.h"
#include "cv_bridge/cv_bridge.h"
#include "sensing_on_road/pedestrian_vision_batch.h"
#include "sensing_on_road/pedestrian_vision.h"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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
    image_transport::SubscriberFilter image_sub_;
    message_filters::Subscriber<sensing_on_road::pedestrian_vision_batch> people_rects_sub_;
    ros::Publisher people_roi_pub_;
    ros::Publisher people_detect_pub_;
    ros::Publisher people_ver_pub_;
    //sensor_msgs::CvBridge bridge_;
    image_transport::Publisher image_pub_;

    void ScaleWithDistanceRatio(Mat *img, double disz, double norm_distance, Size img_size, Size smallest_size, double *ratio);
    void detectPedestrian(Point offset, double ratio, gpu::GpuMat& gpu_img, sensing_on_road::pedestrian_vision_batch *detect_rects);
    void syncCallback(const sensing_on_road::pedestrian_vision_batchConstPtr pr, const sensor_msgs::ImageConstPtr);
    void updateParameter();
    template <class T>
    void checkParamChanged(T &a, T &b);
    bool parameter_changed;
    cv::Mat img;
    int count;
    bool new_image, show_processed_image, black_front_roi,write_image;
    double hit_threshold, scale, group_threshold, norm_dist;
    std::string path;
};

