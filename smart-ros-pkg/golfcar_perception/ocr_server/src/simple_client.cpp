#include "ros/ros.h"
#include <cstdlib>
#include <cv.h>
#include <highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include "ocr_server/TextRecognition.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ocr_client");
  if (argc != 2)
  {
    ROS_INFO("please specify the image");
    return 1;
  }
  IplImage *src=0;
  if((src = cvLoadImage( argv[1], CV_LOAD_IMAGE_GRAYSCALE)) == 0)
  {
      return -1;
  }

  sensor_msgs::CvBridge bridge_;
  sensor_msgs::Image::Ptr load_image =  bridge_.cvToImgMsg(src, "mono8");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ocr_server::TextRecognition>("/ocr_server/ocr");
  ocr_server::TextRecognition srv;
  srv.request.image = *load_image;

  if (client.call(srv))
  {
    ROS_INFO("successfully call the service");

    for (size_t i=0; i<srv.response.lines.size(); i++) {
	 printf("%i. %s\n", (int)i, srv.response.lines[i].c_str());
   }

    const char * firstletter = srv.response.lines[0].c_str();
    printf("%c", firstletter[0]);

  }
  else
  {
    ROS_ERROR("service failure");
    return 1;
  }

  return 0;
}
