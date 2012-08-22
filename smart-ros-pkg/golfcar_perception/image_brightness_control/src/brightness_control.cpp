#include "brightness_control.h"


namespace golfcar_vision{
  
  brightness_control::brightness_control():
    private_nh_("~"),
    it_(nh_)
  {
	  control_command_ = 1000;
      cam_sub_ = it_.subscribeCamera("/camera_front/image_raw", 1, &brightness_control::ImageCallBack, this);
      image_brightness_pub = nh_.advertise<image_brightness_control::image_brightness>("image_brightness",2);
      control_command_pub = nh_.advertise<image_brightness_control::control_command>("control_command",2);
  }
  
  void brightness_control::ImageCallBack( const sensor_msgs::ImageConstPtr& image_msg,
														const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
        ROS_INFO("ImageCallBack");

        IplImage* color_image, *gray_image;
        //get image in OpenCV format;
        try {
            color_image = bridge_.imgMsgToCv(image_msg, "bgr8");
            }
        catch (sensor_msgs::CvBridgeException& ex) {
            ROS_ERROR("Failed to convert image");
            return;
            }
            
        gray_image = cvCreateImage(cvGetSize(color_image),8,1);
        cvCvtColor(color_image, gray_image, CV_BGR2GRAY);
        
        int numBins = 16;
		  float range[] = {0, 255};
        float *ranges[] = { range };
        CvHistogram *hist = cvCreateHist(1, &numBins, CV_HIST_ARRAY, ranges, 1);
		  cvClearHist(hist);
        cvCalcHist(&gray_image, hist, 0, 0);
        cvNormalizeHist(hist,1);
        image_brightness_control::image_brightness brightness_indicator;
        brightness_indicator.header = info_msg->header;
        for(int i = 0; i < 16; i++)
        {
			  float hist_element_value = cvQueryHistValue_1D(hist, i);
			  brightness_indicator.hist_elements.push_back(hist_element_value);
		  }
        image_brightness_pub.publish(brightness_indicator);
        cvReleaseImage(&gray_image);
        
        //simple control policy;
        if(brightness_indicator.hist_elements[15] > 0.20)
        {
			  if(control_command_ >1)
           control_command_--;
        }
        else if (brightness_indicator.hist_elements[15] < 0.10) 
        {
			  if(control_command_ <1000)
			  control_command_++;
		  }
        image_brightness_control::control_command control_tmp;
        control_tmp.header = image_msg->header;
        control_tmp.shutter = control_command_;
        control_command_pub.publish(control_tmp);
  }
  
  brightness_control::~brightness_control()
  {
  }
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "brightness_control");
	 ros::NodeHandle n;
	 golfcar_vision::brightness_control brightness_control_node;
    ros::spin();
    return 0;
}
