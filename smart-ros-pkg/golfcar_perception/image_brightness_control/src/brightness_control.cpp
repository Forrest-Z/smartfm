#include "brightness_control.h"


namespace golfcar_vision{
  
  brightness_control::brightness_control():
    private_nh_("~"),
    it_(nh_)
  {
	  private_nh_.param("control_command_", control_command_, 100);
	  private_nh_.param("control_command_", expected_centroid_, 115);
	  private_nh_.param("visualization_flag", visualization_flag_, true);
     cam_sub_ = it_.subscribeCamera("/camera_front/image_raw", 1, &brightness_control::ImageCallBack, this);
     image_brightness_pub = nh_.advertise<image_brightness_control::image_brightness>("image_brightness",2);
     control_command_pub = nh_.advertise<image_brightness_control::control_command>("control_command",2);
     
     if(visualization_flag_)
     {
		  cvNamedWindow("histogram",1);
	  }
  }
  
	void brightness_control::DrawHistogram(IplImage* imgHist, CvHistogram *hist)
	{
		 float histMax = 1.0;
		 for(int i=0;i<16;i++)
		 {
			  int scaleX = 256/16;
			  float histValue = cvQueryHistValue_1D(hist, i);
			  //float nextValue = cvQueryHistValue_1D(hist, i+1);
			  CvPoint pt1 = cvPoint(i*scaleX, 64);
			  CvPoint pt2 = cvPoint(i*scaleX+scaleX, 64);
			  CvPoint pt3 = cvPoint(i*scaleX+scaleX, 64-histValue*64/histMax);
			  CvPoint pt4 = cvPoint(i*scaleX, 64-histValue*64/histMax);
			  int numPts = 5;
			  CvPoint pts[] = {pt1, pt2, pt3, pt4, pt1};
			  cvFillConvexPoly(imgHist, pts, numPts, cvScalar(255));
		 }
		 return;
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
        
        if(visualization_flag_)
        {
			   IplImage* imgHist = cvCreateImage(cvSize(256, 64),8,1);
			   cvZero(imgHist);
				DrawHistogram(imgHist, hist);
				cvShowImage("histogram", imgHist);
				cvWaitKey(10);
				cvReleaseImage(&imgHist);
        }
        
        generate_ctrl_cmd(brightness_indicator);
  }
  
  void brightness_control::generate_ctrl_cmd(image_brightness_control::image_brightness & brightness_indicator)
  {
	  //simple control policy;
	  
	  double distance_weighted_pixel = 0;
	  for(int i=0; i<16; i++)
	  {
		  int bin_center = 256/32 + 256/16*i;
		  distance_weighted_pixel = distance_weighted_pixel + double(bin_center) * brightness_indicator.hist_elements[i];
	  }
	  
	  int brightness_centroid = (int)distance_weighted_pixel;
	  int control_step = (expected_centroid_ - brightness_centroid)/5;
	  
	  if(control_step==0)
	  {
		  if(expected_centroid_ - brightness_centroid >= 0) control_step = 1;
		  else control_step = -1;
	  }
	  if(control_step >  10) control_step  =   10;
	  if(control_step < -10) control_step = -10;
	  
	  control_command_ = control_command_ + control_step;
	  
	  if(control_command_<=1) control_command_=1;
	  if(control_command_>=100) control_command_ = 100;
	  
	  if(visualization_flag_) ROS_INFO("brightness_centroid %d", brightness_centroid);
	  
	  image_brightness_control::control_command control_tmp;
	  control_tmp.header = brightness_indicator.header;
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
