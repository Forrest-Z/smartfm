#include "ros/ros.h"
#include <cstdlib>
#include <cv.h>
#include <highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include "ocr_server/TextRecognition.h"


class OcrClientNode {
public:
  ros::ServiceClient ocr_client_;
  sensor_msgs::CvBridge bridge_;

  OcrClientNode() {
    ros::NodeHandle private_nh("~");
    ocr_client_ = private_nh.serviceClient<ocr_server::TextRecognition>("/ocr_server/ocr");
    sensor_msgs::CvBridge bridge_;
  }
  virtual ~OcrClientNode() {
  }
  bool recognize(IplImage *src, char letter);
};
