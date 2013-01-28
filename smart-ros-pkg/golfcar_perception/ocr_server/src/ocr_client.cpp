#include "ros/ros.h"
#include <cstdlib>
#include <cv.h>
#include <highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include "ocr_server/TextRecognition.h"
#include "ocr_client.h"

  bool OcrClientNode::recognize(IplImage *src, char letter)
  {
	ocr_server::TextRecognition srv;
	sensor_msgs::Image::Ptr load_image =  bridge_.cvToImgMsg(src, "mono8");
	srv.request.image = *load_image;

	if (ocr_client_.call(srv))
	{
		ROS_INFO("successfully call the service");

		for (size_t i=0; i<srv.response.lines.size(); i++) {
		printf("%i. %s\n", (int)i, srv.response.lines[i].c_str());
		}
		const char * firstletter = srv.response.lines[0].c_str();
		letter = firstletter[0];
		return true;
	}
	else
	{
	ROS_ERROR("service failure");
	return false;
	}
  }
