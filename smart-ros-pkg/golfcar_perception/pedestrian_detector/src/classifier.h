#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensing_on_road/pedestrian_vision_batch.h"
#include "sensing_on_road/pedestrian_vision.h"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;
using namespace gpu;


class HOGClassifier {

public:


    HOGClassifier(ros::NodeHandle &n);
    //HOGClassifier(){};
    ~HOGClassifier();
    
    bool fillRoiRectangle(Size img_size, Size* roi_size, Point* roi_point, sensing_on_road::pedestrian_vision& pd);

private:
    cv::HOGDescriptor cpu_hog;
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber people_rects_sub_;
    ros::Publisher people_roi_pub_;
    ros::Publisher people_detect_pub_;
    ros::Publisher people_ver_pub_;
    //sensor_msgs::CvBridge bridge_;
    image_transport::Publisher image_pub_;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr);
    void peopleRectsCallback(sensing_on_road::pedestrian_vision_batch pr);
    void ScaleWithDistanceRatio(Mat *img, double disz, double norm_distance, Size img_size, Size smallest_size, double *ratio);
    void detectPedestrian(Point offset, double ratio, gpu::GpuMat& gpu_img, sensing_on_road::pedestrian_vision_batch *detect_rects);

    cv::Mat img;
    int count;
    bool new_image, show_processed_image, black_front_roi,write_image;
    double hit_threshold, scale, group_threshold, norm_dist;
    std::string path;
};

