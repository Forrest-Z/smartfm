#include <oculus_viewer/distort.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp> // tmp
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

namespace oculus_viewer {

IplImage* barrel_dist(IplImage* img, double Cx, double Cy,
                      double k0, double k1, double k2) {
  IplImage* mapx = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
  IplImage* mapy = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
  
  int w= img->width;
  int h= img->height;
  
  float* pbuf = (float*)mapx->imageData;
  const float unit_xr2 = (w - Cx) * (w - Cx);
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      float r2 = (x-Cx)*(x-Cx)+(y-Cy)*(y-Cy);
      r2 /= unit_xr2;
      const float r4 = r2 * r2;
      *pbuf = Cx + (x - Cx) * (k0 + k1 * r2 + k2 * r4);
      ++pbuf;
    }
  }
  
  pbuf = (float*)mapy->imageData;
  const float unit_yr2 = (h -Cy) * (h -Cy);
  for (int y = 0;y < h; y++) {
    for (int x = 0; x < w; x++) {
      float r2 = (x-Cx)*(x-Cx)+(y-Cy)*(y-Cy);
      r2 /= unit_yr2;
      const float r4 = r2 * r2;
      *pbuf = Cy + (y - Cy) * (k0 + k1 * r2 + k2 * r4);
      ++pbuf;
    }
  }

  IplImage* temp = cvCloneImage(img);
  cvRemap( temp, img, mapx, mapy ); 
  cvReleaseImage(&temp);
  cvReleaseImage(&mapx);
  cvReleaseImage(&mapy);
  
  return img;
}

DistortImage::DistortImage()
  : nh_()
  , it_(nh_)
  , offset_(0.0)
  , scale_(1.0) {
}
  
void DistortImage::init(const std::string& topic_name) {
  ros::NodeHandle node;
  try {
    pub_ = it_.advertise(topic_name + "/distorted", 1);
  } catch(ros::Exception& e) {
    ROS_ERROR("init ROS error: %s", e.what());
  } catch(image_transport::TransportLoadException& e) {
    ROS_ERROR("image_transport error: %s", e.what());
  } catch(...) {
    ROS_ERROR("some error");
  }
}

void DistortImage::process(cv::Mat &image) {
  IplImage img = image;
  if (K_.size() > 2) {
    double cx = (image.cols) / 2 + offset_;
    double cy = image.rows / 2;
//     std::cout<<cx<<" "<<cy<<" "<<offset_<<" "<<K_[0]<<" "<<K_[1]<<" "<<K_[2]<<std::endl;
    barrel_dist(&img, cx, cy, 
                K_[0], K_[1], K_[2]);
    cv::Mat resized_image;
    cv::resize(image, resized_image, cv::Size(), scale_, scale_);
    image = resized_image(
        cv::Rect((resized_image.cols - image.cols) / 2,
                 (resized_image.rows - image.rows) /2,
                 image.cols,
                 image.rows));
    img_ = image;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    cv_bridge::CvImage cv_img(header, "bgr8", image);
    pub_.publish(cv_img.toImageMsg());
  }
}

}  // namespace oculus_viewer
