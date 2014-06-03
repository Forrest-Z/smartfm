#include <ros/ros.h>
#include <oculus_msgs/HMDInfo.h>
#include <oculus_viewer/distort.h>
#include <oculus_viewer/viewer.h>

using namespace std;

namespace oculus_viewer {

class ImageDistortViewer {
 public:
  ImageDistortViewer();
  void init();
  void HMDInfoCallback(const oculus_msgs::HMDInfoPtr& info);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void show();
  Viewer viewer_;
 private:
  DistortImage left_;
  DistortImage right_;
  ros::Subscriber sub_;
  bool use_display_;
  ros::NodeHandle nh_;
  image_transport::Subscriber img_sub_;
  image_transport::ImageTransport it_;
};

ImageDistortViewer::ImageDistortViewer()
  : viewer_("oculus camera view")
  , use_display_(true)
  , nh_()
  , it_(nh_){
}

void ImageDistortViewer::imageCallback(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr ptr;
  try{
    ptr = cv_bridge::toCvCopy(msg, "bgr8");
  }
  catch(ros::Exception& e) {
    ROS_ERROR("init ROS error: %s", e.what());
  } catch(image_transport::TransportLoadException& e) {
    ROS_ERROR("image_transport error: %s", e.what());
  } catch(...) {
    ROS_ERROR("some error");
  }
  
  cv::Mat raw_img = ptr->image;
  int img_width = ptr->image.cols/2, img_height = ptr->image.rows;
  cv::Mat left_img = raw_img(cv::Rect(img_width, 0, img_width, img_height));
  cv::Mat right_img = raw_img(cv::Rect(0, 0, img_width, img_height));
  cv::imshow("distort img", raw_img);
  cv::imshow("left img", left_img);
  cv::imshow("right img", right_img);
  left_.process(left_img);
  right_.process(right_img);
  show();
  cvWaitKey(1);
}
void ImageDistortViewer::init() {
	ros::NodeHandle node;
  left_.init("minoru_left/image_raw");
  right_.init("minoru_right/image_raw");
  sub_ = node.subscribe("/oculus/hmd_info",
                        1,
                        &ImageDistortViewer::HMDInfoCallback,
                        this);
  img_sub_ = it_.subscribe("gscam/image_raw", 1, &ImageDistortViewer::imageCallback, 
			   this, image_transport::TransportHints("compressed"));
  
  ros::NodeHandle private_node("~");	
  int32_t offset_x = 0;
  private_node.param<int32_t>("display_offset_x", offset_x, 60);
  int32_t offset_y = 0;
  private_node.param<int32_t>("display_offset_y", offset_y, 17);
  viewer_.setDisplayOffset(offset_x, offset_y);
  private_node.param<bool>("use_display", use_display_, true);
}

void ImageDistortViewer::show() {
  if (use_display_) {
    if ((!right_.getImage().empty()) &&
        (!left_.getImage().empty())) {
      viewer_.show(left_.getImage(), right_.getImage());
    }
  }
}

void ImageDistortViewer::HMDInfoCallback(
    const oculus_msgs::HMDInfoPtr& info) {
  if (info->horizontal_screen_size > 0) {
    double lens_center = 1 - 2 * info->lens_separation_distance / info->horizontal_screen_size;
    double scale = 1 + lens_center;
    left_.setK(info->distortion_K);
    left_.setScale(scale);
    left_.setOffset(-lens_center);
    right_.setK(info->distortion_K);
    right_.setScale(scale);
    right_.setOffset(lens_center);
  }
}

}  // namespace oculus_viewer


int main(int argc, char** argv) {
	ros::init(argc, argv, "image_distort_viewer");
  try {
    oculus_viewer::ImageDistortViewer dis;
    dis.init();
    ros::spin();
   
  } catch(ros::Exception& e) {
    ROS_ERROR("ros error: %s", e.what());
  } catch(...) {
    ROS_FATAL("unexpected error");
  }
}

