#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdexcept>
class ImageConverter {

public:

std::string video_file;
cv::VideoCapture vc;
bool running;
ImageConverter(ros::NodeHandle &n, std::string argcv) :
  n_(n), it_(n_)
{
	image_pub_ = it_.advertise("image_raw",1);
	video_file = argcv;
	convert_video_to_ros();
}

~ImageConverter()
{
}

void convert_video_to_ros()
{
	while(running)
	{
		vc.open(video_file);
		if (!vc.isOpened())
		throw std::runtime_error(std::string("can't open video file: " + video_file));
		cv::Mat frame;
		IplImage dst_img;
		vc >> frame;
		
		 while (running && !frame.empty())
        {
			dst_img = frame;

			try
			{
				image_pub_.publish(bridge_.cvToImgMsg(&dst_img, "passthrough"));
			}
			catch (sensor_msgs::CvBridgeException error)
			{
				ROS_ERROR("error");
			}
			vc >> frame;
			handleKey((char)cv::waitKey(20));
		}
	}
	exit(0);
}

void handleKey(char key)
{
	std::cout<<key;
	switch(key){
	case 'q':
    case 'Q':running=false;
		break;
	}
}


protected:

ros::NodeHandle n_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
sensor_msgs::CvBridge bridge_;
image_transport::Publisher image_pub_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;
  ImageConverter ic(n, argv[1]);
  ros::spin();
  return 0;
}
