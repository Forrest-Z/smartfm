#include "ipm.h"

namespace golfcar_vision{
  
  ipm::ipm():
    private_nh_("~"),
    it_(nh_),
    fixedTf_inited_(false)
  {
      training_frame_serial_ = 0;
      private_nh_.param("destination_frame_id", dest_frame_id_, std::string("base_link"));
      cam_sub_ = it_.subscribeCamera("/usb_cam/image_raw", 1, &ipm::ImageCallBack, this);
      image_pub_ = it_.advertise("/usb_cam/image_ipm", 1);
      
      markers_info_pub = nh_.advertise<vision_lane_detection::markers_info>("markers_info",2);
      
      image_processor_ = new image_proc();
      
      //Four base points on the ground in the "base_link" coordinate; "base_link" is at the back wheel.
      gndQuad_[0].x = RECT_P0_X+DIS_CAM_BASE_X;
      gndQuad_[0].y = RECT_P0_Y;
      gndQuad_[1].x = RECT_P1_X+DIS_CAM_BASE_X;
      gndQuad_[1].y = RECT_P1_Y;
      gndQuad_[2].x = RECT_P2_X+DIS_CAM_BASE_X;
      gndQuad_[2].y = RECT_P2_Y;
      gndQuad_[3].x = RECT_P3_X+DIS_CAM_BASE_X;
      gndQuad_[3].y = RECT_P3_Y;
      
      cvNamedWindow("src_window");
      //cvNamedWindow("gray_window");
      cvNamedWindow("ipm_window");
  }
  
  void ipm::ImageCallBack( const sensor_msgs::ImageConstPtr& image_msg,
                           const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
        ROS_INFO("ImageCallBack");
        IplImage* color_image, *gray_image, *ipm_image;
        
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
        
        //assign camera informtion to "cam_model_";
        cam_model_.fromCameraInfo(info_msg);
        CameraStaticInfo_ = *info_msg;
        scale_ = float(CameraStaticInfo_.height)/GND_HEIGHT;
      
        //check if pinhole camera parameter is right;
        //ROS_DEBUG("cam_model_ parameters:");
        //ROS_DEBUG("fx %5f, fy %5f",cam_model_.fx(), cam_model_.fy());
        //ROS_DEBUG("Tx %5f, Ty %5f",cam_model_.Tx(), cam_model_.Ty());
        //ROS_DEBUG("cx %5f, cy %5f",cam_model_.cx(), cam_model_.cy());
        
        //get fixed transform information from "baselink" (dest) to "camera" (src);
        //pay attention to that this time the roles are interchanged between two frames;
        //right now is "camera" centered;
        if(!fixedTf_inited_)
        {
            tf::StampedTransform transform;
            try {
                ros::Time acquisition_time = info_msg->header.stamp;
                ros::Duration timeout(5.0 / 30);
                tf_.waitForTransform(cam_model_.tfFrame(), dest_frame_id_,
                                                acquisition_time, timeout);
                tf_.lookupTransform(cam_model_.tfFrame(), dest_frame_id_,
                                                acquisition_time, transform);
            
                tf::Transform *transform_pointer = &transform;
                src_dest_tf_ = *transform_pointer;
                
                fixedTf_inited_ = true;
                }
            catch (tf::TransformException& ex) {
                ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
                return;
            }
        }
        
        ////////////////////////////////////////////////
        //main functional part;
        ////////////////////////////////////////////////
        
        //1. from "gndQuad_[4]" to get "srcQuad_[4]";
        GndPt_to_Src(gndQuad_, srcQuad_);
        
        //2. from "gndQuad_[4]" to get "dstQuad_[4]";
        //this depends on how you represent your image;
        GndPt_to_Dst(gndQuad_, dstQuad_);
        
        //3. wrap the image;
        CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);
        cvGetPerspectiveTransform(srcQuad_,dstQuad_,warp_matrix);
        
        ipm_image = cvCreateImage(cvGetSize(gray_image),8,1);
        cvWarpPerspective( gray_image, ipm_image, warp_matrix);
        
        ////////////////////////////////////////////////
        //visualization part;
        ////////////////////////////////////////////////
        cvCircle( color_image, cvPointFrom32f(srcQuad_[0]), 6, CV_RGB(0,255,0), 2);
        cvCircle( color_image, cvPointFrom32f(srcQuad_[1]), 6, CV_RGB(0,255,0), 2);
        cvCircle( color_image, cvPointFrom32f(srcQuad_[2]), 6, CV_RGB(0,255,0), 2);
        cvCircle( color_image, cvPointFrom32f(srcQuad_[3]), 6, CV_RGB(0,255,0), 2);
        
        cvLine( color_image, cvPointFrom32f(srcQuad_[0]), cvPointFrom32f(srcQuad_[1]), CV_RGB(0,0,255), 1);
        cvLine( color_image, cvPointFrom32f(srcQuad_[1]), cvPointFrom32f(srcQuad_[2]), CV_RGB(0,0,255), 1);
        cvLine( color_image, cvPointFrom32f(srcQuad_[2]), cvPointFrom32f(srcQuad_[3]), CV_RGB(0,0,255), 1);
        cvLine( color_image, cvPointFrom32f(srcQuad_[3]), cvPointFrom32f(srcQuad_[0]), CV_RGB(0,0,255), 1);
        
        //from "gndPt_[4]" to get "dstQuad_[4]";    
        cvShowImage("src_window", color_image);
        //cvShowImage("gray_window", gray_image);
        cvShowImage("ipm_window", ipm_image);
        
        //cvSaveImage("/home/baoxing/src_window.png", color_image);
        //cvSaveImage("/home/baoxing/ipm_window.png", ipm_image);
        
        try
          {
            image_pub_.publish(bridge_.cvToImgMsg(ipm_image, "mono8"));
          }
        catch (sensor_msgs::CvBridgeException error)
          {
            ROS_ERROR("error");
          }
          
        //this scentence is necessary;
        cvWaitKey(10);
        
        image_processor_->Extract_Markers(ipm_image, scale_, markers_, training_frame_serial_);
        markers_.header = info_msg -> header;
        markers_info_pub.publish(markers_);
        
        ROS_INFO("ImageCallBack finished");
        
        //Attention: 
        //color_image is not allocated memory as normal;
        //it points to some memory space in "cv_bridge" object handled by ros;
        //so there is no need and it is also not permitted to release its memory space as normal;
        //cvReleaseImage(&color_image);
        
        cvReleaseImage(&gray_image);
        cvReleaseImage(&ipm_image);
        cvReleaseMat(&warp_matrix);
  }
  
  //Function "GndPt_to_Src": project ground point in baselink coordinate into camera image;
  //input: pointer to ground points in "baselink coordinate"; output: pointer to pixels in "camera image";
  //steps: a. use "tf" relationship; b. use cam_model to project;
  void ipm::GndPt_to_Src(CvPoint2D32f * gnd_pointer, CvPoint2D32f* src_pointer)
  {
        geometry_msgs::Pose temppose;
		temppose.position.x=0;
		temppose.position.y=0;
		temppose.position.z=0;
		temppose.orientation.x=1;
		temppose.orientation.y=0;
		temppose.orientation.z=0;
		temppose.orientation.w=0;
        tf::Pose tempTfPose;
        tf::Pose PoseInCamera;
        for(unsigned int i = 0; i < 4; i++)
        {
            temppose.position.x= gnd_pointer[i].x;
            temppose.position.y= gnd_pointer[i].y;
			tf::poseMsgToTF(temppose, tempTfPose);
            PoseInCamera = src_dest_tf_ * tempTfPose;
            tf::Point pt = PoseInCamera.getOrigin();
            //ROS_DEBUG("%5f,%5f,%5f", pt.x(), pt.y(), pt.z());
            cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
            cv::Point2d uv;
            cam_model_.project3dToPixel(pt_cv, uv);
            src_pointer[i].x = uv.x;
            src_pointer[i].y = uv.y;
            //ROS_DEBUG("%5f, %5f", src_pointer[i].x, src_pointer[i].y);
        }
  }
  
  void ipm::GndPt_to_Dst(CvPoint2D32f * gnd_pointer, CvPoint2D32f* dst_pointer)
  {
      //this point corresponds to the center of the image;
      float center_x = (RECT_P0_X + RECT_P2_X)/2.0 + DIS_CAM_BASE_X;
      float center_y = 0.0;
      
      for(unsigned int i = 0; i < 4; i++)
      {
          float x_tmp = - (gnd_pointer[i].y-center_y);
          float y_tmp = - (gnd_pointer[i].x-center_x);
          dst_pointer[i].x =  x_tmp * scale_ + CameraStaticInfo_.width/2.0;
          dst_pointer[i].y =  y_tmp * scale_ + CameraStaticInfo_.height/2.0;
          ROS_DEBUG("%5f, %5f, %5f, %5f", x_tmp, y_tmp, dst_pointer[i].x, dst_pointer[i].y);
      }
  }
  
  ipm::~ipm()
  {
    cvDestroyWindow("src_window");
    //cvDestroyWindow("gray_window");
    cvDestroyWindow("ipm_window");
    delete image_processor_;
  }
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "ipm");
	 ros::NodeHandle n;
	 golfcar_vision::ipm ipm_node;
     ros::spin();
     return 0;
}
