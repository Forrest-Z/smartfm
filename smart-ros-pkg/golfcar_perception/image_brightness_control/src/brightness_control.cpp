#include "brightness_control.h"


namespace golfcar_vision{
  
  brightness_control::brightness_control():
    private_nh_("~"),
    it_(nh_)
  {
	  visualization_ = true;
	  
	srv = new dynamic_reconfigure::Server<image_brightness_control::brightCTRConfig> (ros::NodeHandle("~"));
	dynamic_reconfigure::Server<image_brightness_control::brightCTRConfig>::CallbackType f = boost::bind(&brightness_control::reconfig, this, _1, _2);
	srv->setCallback(f);
		
	 cam_sub_ = it_.subscribeCamera("/camera_front/image_raw", 1, &brightness_control::ImageCallBack, this);
	 image_brightness_pub = nh_.advertise<image_brightness_control::image_brightness>("image_brightness",2);
	 control_command_pub = nh_.advertise<image_brightness_control::control_command>("/camera_front/control_command",2);

	 if(visualization_)
	 {
		  cvNamedWindow("histogram",1);
	 }
	 
  }
  
  void brightness_control::reconfig(image_brightness_control::brightCTRConfig &config, uint32_t level) 
  {
	shutter_value_ 		= 	config.shutter_value;
	gain_value_ 		= 	config.gain_value;
	expected_centroid_ 	= 	config.expected_centroid;
	shutter_control_ 	= 	config.shutter_control;
	gain_control_ 		= 	config.gain_control;
	visualization_ 		= 	config.visualization;
	p_shutter_ 			=	config.p_shutter;
	i_shutter_ 			=	config.i_shutter;
	d_shutter_ 			=	config.d_shutter;
	p_gain_ 			=	config.p_gain;
	i_gain_ 			=	config.i_gain;
	d_gain_ 			=	config.d_gain;
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

        cvSetImageROI(color_image, cvRect(24, 306, 601, 156));

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
        for(int i = 0; i < numBins; i++)
        {
			  float hist_element_value = cvQueryHistValue_1D(hist, i);
			  brightness_indicator.hist_elements.push_back(hist_element_value);
		  }
        image_brightness_pub.publish(brightness_indicator);
        cvReleaseImage(&gray_image);
        
        if(visualization_)
        {
			IplImage* imgHist = cvCreateImage(cvSize(256, 64),8,1);
			cvZero(imgHist);
			DrawHistogram(imgHist, hist);
			cvShowImage("histogram", imgHist);
			cvWaitKey(1);
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
		  int bin_center = 256/32 + i*256/16;
		  distance_weighted_pixel = distance_weighted_pixel + double(bin_center) * brightness_indicator.hist_elements[i];
	  }
	  int brightness_centroid = (int)distance_weighted_pixel;
	  if(visualization_) ROS_INFO("brightness_centroid %d", brightness_centroid);
	  
	  control_command_.header = brightness_indicator.header;
	  shutter_control(brightness_centroid);
	  gain_control(brightness_centroid);
	  control_command_pub.publish(control_command_);
  }
  
  void brightness_control::shutter_control(int brightness_centroid)
  {
	  float error = expected_centroid_ - brightness_centroid;
	  float i_err  = error;

	  if(fabs(error) > 50)
	  {
	    i_err = error *50/fabs(error);
	  }
	  
	  float k_err = (error - last_err);
	  if(fabs(k_err) > 20)
	  {
		  k_err = k_err*20/fabs(k_err);
	  }

	  float d_err = (error - 2*last_err+last_err2);
	  float control_step = k_p* k_err+ k_i*i_err+k_d*d_err;

	  last_err2 = last_err;
	  last_err = error;

	  if(fabs(control_step) > 10)
	  {
		   control_step = 10 * control_step / fabs(control_step);
	  }
	  
	  if(fabs(error) < 20)
	  {
		  control_step = 0;
	  }
	  cmd = cmd + control_step;

	  if(cmd < 10)
	  {
		  cmd = 10;
	  }
	  else if (cmd > 300)
	  {
		  cmd = 300;
	  }

	  control_command_.gain = cmd;
  }
  
    void brightness_control::gain_control(int brightness_centroid)
  {
	  	  
	  float error = expected_centroid_ - brightness_centroid;
	  int tmp_gain = 0;
	  if(fabs(error) > 80)
	  {
	    tmp_gain = error * 20.0/fabs(error);
	  }
	  
	  gain_value_ += tmp_gain;
	  if(gain_value_ < 40)
	  {
		  gain_value_=40;
	  }
	  else if(gain_value_> 250)
	  {
		  gain_value_=250;
	  }
	 
	  control_command_.gain = gain_value_;
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
