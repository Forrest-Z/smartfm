#include "ipm.h"


namespace golfcar_vision{
  
  ipm::ipm():
    private_nh_("~"),
    it_(nh_),
    fixedTf_inited_(false),
    visualization_flag_(false)
  {
	  string svm_model_path;
	  string svm_scale_path;
	  private_nh_.param("svm_model_path", svm_model_path, std::string("/home/baoxing/workspace/data_and_model/scaled_20120726.model"));
	  private_nh_.param("svm_scale_path", svm_scale_path, std::string("/home/baoxing/workspace/data_and_model/range_20120726"));
	  
	  private_nh_.param("publish_dis_thresh",     publish_dis_thresh_,    0.05);
	  private_nh_.param("publish_angle_thresh",   publish_angle_thresh_,  5.0/180.0*M_PI);
      private_nh_.param("visualization_flag",     visualization_flag_,    false);     
      training_frame_serial_ = 0;
      private_nh_.param("destination_frame_id", dest_frame_id_, std::string("base_link"));
      cam_sub_ = it_.subscribeCamera("/camera_front/image_raw", 1, &ipm::ImageCallBack, this);
      image_pub_ = it_.advertise("/camera_front/image_ipm", 1);
      
      markers_info_pub = nh_.advertise<vision_lane_detection::markers_info>("markers_info",2);
      markers_info_2nd_pub = nh_.advertise<vision_lane_detection::markers_info>("markers_2nd_info",2);
      
      planeCoef_sub_ = nh_.subscribe("plane_coef", 10, &ipm::planeCoefCallback, this);
      
      
      
      image_processor_ = new image_proc(svm_model_path, svm_scale_path);
      
      //Four base points on the ground in the "base_link" coordinate; "base_link" is at the back wheel.
      gndQuad_[0].x = RECT_P0_X+DIS_CAM_BASE_X;
      gndQuad_[0].y = RECT_P0_Y;
      gndQuad_[1].x = RECT_P1_X+DIS_CAM_BASE_X;
      gndQuad_[1].y = RECT_P1_Y;
      gndQuad_[2].x = RECT_P2_X+DIS_CAM_BASE_X;
      gndQuad_[2].y = RECT_P2_Y;
      gndQuad_[3].x = RECT_P3_X+DIS_CAM_BASE_X;
      gndQuad_[3].y = RECT_P3_Y;
		
      pub_init_ = false;
      
      if(visualization_flag_)
      {
		cvNamedWindow("src_window");
		cvNamedWindow("ipm_window");
	  }
	  				
		warp_matrix_ = cvCreateMat(3,3,CV_32FC1);
		projection_matrix_ = cvCreateMat(3,3,CV_32FC1);
		
		
		//to accumulate the curb points (road_boundary);
		odom_frame_ = "odom";
		base_frame_ = "base_link";
		curb_point_sub_.subscribe(nh_, "hybrid_pt", 10);
		curb_point_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(curb_point_sub_, tf_, odom_frame_, 10);
		curb_point_filter_->registerCallback(boost::bind(&ipm::curbCallback, this, _1));
		curb_point_filter_->setTolerance(ros::Duration(0.05));
		left_accumulated_.header.frame_id  = odom_frame_;
		right_accumulated_.header.frame_id = odom_frame_;
		curb_num_limit_ = 500;
		
		left_pub_   = nh_.advertise<sensor_msgs::PointCloud>("vision_curb_left", 10);
		right_pub_   = nh_.advertise<sensor_msgs::PointCloud>("vision_curb_right", 10);
  }
  
  void ipm::curbCallback(const sensor_msgs::PointCloudConstPtr  curb_in)
  {
	   sensor_msgs::PointCloud left_tmp;
	   sensor_msgs::PointCloud right_tmp;
	   left_tmp.header = curb_in->header;
	   right_tmp.header = curb_in->header;
	   for(size_t i=0;i< curb_in->points.size(); i ++)
	   {
			if(curb_in->points[i].y >= 0.0)left_tmp.points.push_back(curb_in->points[i]);
			else {right_tmp.points.push_back(curb_in->points[i]);}
		}
		try {tf_.transformPointCloud(odom_frame_, left_tmp, left_tmp);}
		catch (tf::TransformException &ex){printf ("Failure %s\n", ex.what());return;}
		try {tf_.transformPointCloud(odom_frame_, right_tmp, right_tmp);}
		catch (tf::TransformException &ex){printf ("Failure %s\n", ex.what());return;}
		 
		for(size_t i=0; i<left_tmp.points.size(); i++){left_accumulated_.points.push_back(left_tmp.points[i]);}
		
		left_tmp.points.clear();
		if(left_accumulated_.points.size() > curb_num_limit_)
		{
			size_t obsolete_num = left_accumulated_.points.size()- curb_num_limit_;
			for(size_t j= obsolete_num; j <left_accumulated_.points.size(); j ++)
			{
				left_tmp.points.push_back(left_accumulated_.points[j]);
			}
			left_accumulated_.points = left_tmp.points;
		}
		left_accumulated_.header.stamp = curb_in->header.stamp;
		
		for(size_t i=0; i<right_tmp.points.size(); i++){right_accumulated_.points.push_back(right_tmp.points[i]);}
		right_tmp.points.clear();
		if(right_accumulated_.points.size() > curb_num_limit_)
		{
			size_t obsolete_num = right_accumulated_.points.size()- curb_num_limit_;
			for(size_t j= obsolete_num; j <right_accumulated_.points.size(); j ++)
			{
				right_tmp.points.push_back(right_accumulated_.points[j]);
			}
			right_accumulated_.points = right_tmp.points;
		}
		right_accumulated_.header.stamp = curb_in->header.stamp;
		
		left_pub_.publish(left_accumulated_);
		right_pub_.publish(right_accumulated_);
  }
  
  void ipm::ImageCallBack( const sensor_msgs::ImageConstPtr& image_msg,
                           const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
        ROS_INFO("ImageCallBack");
        
        ros::Time meas_time = info_msg->header.stamp;
        process_control(meas_time);
        
        //disable it temporarily;
        publish_flag_ = true;
        
        if(!publish_flag_)
        {
			ROS_INFO("image not processing since moving distance small");
			return;
			}
			else ROS_INFO("-----------to process image------");
        

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
        ROS_DEBUG("cam_model_ parameters:");
        ROS_DEBUG("fx %5f, fy %5f",cam_model_.fx(), cam_model_.fy());
        ROS_DEBUG("Tx %5f, Ty %5f",cam_model_.Tx(), cam_model_.Ty());
        ROS_DEBUG("cx %5f, cy %5f",cam_model_.cx(), cam_model_.cy());
        
        //get fixed transform information from "baselink" (dest) to "camera" (src);
        //pay attention to that this time the roles are interchanged between two frames;
        //right now is "camera" centered;
        if(!fixedTf_inited_)
        {
            tf::StampedTransform transform;
            try {
                ros::Time acquisition_time = info_msg->header.stamp;
                ros::Duration timeout(5.0 / 30);
                tf_.waitForTransform(cam_model_.tfFrame(), dest_frame_id_, acquisition_time, timeout);
                tf_.lookupTransform(cam_model_.tfFrame(), dest_frame_id_, acquisition_time, transform);
                fixedTf_inited_ = true;
                }
				catch (tf::TransformException& ex){
                ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
                return;
					 }
					 
            tf::Transform *transform_pointer = &transform;
            src_dest_tf_ = *transform_pointer;
        }
        
         //To take into account the uneven of the road surface, the matrix is to be calculated every time;
         //1. from "gndQuad_[4]" to get "srcQuad_[4]";
			GndPt_to_Src(gndQuad_, srcQuad_);
			//2. from "gndQuad_[4]" to get "dstQuad_[4]";
			//this depends on how you represent your image;
			GndPt_to_Dst(gndQuad_, dstQuad_);
			
			ROS_DEBUG("srcQuad points: (%5f, %5f); (%5f, %5f); (%5f, %5f); (%5f, %5f)", 
					srcQuad_[0].x, srcQuad_[0].y,
					srcQuad_[1].x, srcQuad_[1].y,
					srcQuad_[2].x, srcQuad_[2].y,
					srcQuad_[3].x, srcQuad_[3].y
			);
			ROS_DEBUG("dstQuad points: (%5f, %5f); (%5f, %5f); (%5f, %5f); (%5f, %5f)",
					dstQuad_[0].x, dstQuad_[0].y,
					dstQuad_[1].x, dstQuad_[1].y,
					dstQuad_[2].x, dstQuad_[2].y,
					dstQuad_[3].x, dstQuad_[3].y
			);

			cvGetPerspectiveTransform(srcQuad_,dstQuad_,  warp_matrix_);
			cvGetPerspectiveTransform(dstQuad_, srcQuad_, projection_matrix_);
			
			////////////////////////////////////////////////
			//main functional part;
			////////////////////////////////////////////////
        ipm_image = cvCreateImage(cvGetSize(gray_image),8,1);
        cvWarpPerspective( gray_image, ipm_image, warp_matrix_);

        //---------------------------------------------------------------------
        //this helps to reduce the artificial contours in adaptiveThreshold;
        int ipm_height 		= ipm_image -> height;
		  int ipm_width  		= ipm_image -> width;
		  int ipm_step	 		= ipm_image -> widthStep/sizeof(uchar);
		  uchar * ipm_data 	= (uchar*)ipm_image ->imageData;
		  for(int ih=0; ih < ipm_height; ih++)
		  {
			 for(int iw=0; iw < ipm_width; iw++)
			 {
				 if(ipm_data[ih*ipm_step+iw] == 0)
				 {
					 ipm_data[ih*ipm_step+iw]=150;
				 }
			 }
		  }
		  //---------------------------------------------------------------------
		         
        //----------------------project the curb points into image-----------------------
        sensor_msgs::PointCloud left_tmp;
	     sensor_msgs::PointCloud right_tmp;
        left_accumulated_.header.stamp = meas_time;
        right_accumulated_.header.stamp = meas_time;
        try {tf_.transformPointCloud(base_frame_, left_accumulated_, left_tmp);}
		  catch (tf::TransformException &ex){printf ("Failure %s\n", ex.what());return;}
		  try {tf_.transformPointCloud(base_frame_, right_accumulated_, right_tmp);}
		  catch (tf::TransformException &ex){printf ("Failure %s\n", ex.what());return;}
		  
		  std::vector <CvPoint2D32f> left_curb_image;
		  std::vector <CvPoint2D32f> right_curb_image;
        curbPts_to_image(left_tmp, left_curb_image);
        curbPts_to_image(right_tmp, right_curb_image);
        
        for(size_t p=0; p<left_curb_image.size(); p++)
        {
				cvCircle( color_image, cvPointFrom32f(left_curb_image[p]), 6, CV_RGB(0,255,0), 2);
		  }
		  for(size_t p=0; p<right_curb_image.size(); p++)
        {
				cvCircle( color_image, cvPointFrom32f(right_curb_image[p]), 6, CV_RGB(0,0,255), 2);
		  }
		  
		  
        ////////////////////////////////////////////////
        //visualization part;
        ////////////////////////////////////////////////
        if(visualization_flag_)
        {
			cvCircle( color_image, cvPointFrom32f(srcQuad_[0]), 6, CV_RGB(0,255,0), 2);
			cvCircle( color_image, cvPointFrom32f(srcQuad_[1]), 6, CV_RGB(0,255,0), 2);
			cvCircle( color_image, cvPointFrom32f(srcQuad_[2]), 6, CV_RGB(0,255,0), 2);
			cvCircle( color_image, cvPointFrom32f(srcQuad_[3]), 6, CV_RGB(0,255,0), 2);
			cvLine( color_image, cvPointFrom32f(srcQuad_[0]), cvPointFrom32f(srcQuad_[1]), CV_RGB(0,0,255), 1);
			cvLine( color_image, cvPointFrom32f(srcQuad_[1]), cvPointFrom32f(srcQuad_[2]), CV_RGB(0,0,255), 1);
			cvLine( color_image, cvPointFrom32f(srcQuad_[2]), cvPointFrom32f(srcQuad_[3]), CV_RGB(0,0,255), 1);
			cvLine( color_image, cvPointFrom32f(srcQuad_[3]), cvPointFrom32f(srcQuad_[0]), CV_RGB(0,0,255), 1);
			cvShowImage("src_window", color_image);
			
			/*
			cvCircle( ipm_image, cvPointFrom32f(dstQuad_[0]), 6, CV_RGB(0,255,0), 2);
			cvCircle( ipm_image, cvPointFrom32f(dstQuad_[1]), 6, CV_RGB(0,255,0), 2);
			cvCircle( ipm_image, cvPointFrom32f(dstQuad_[2]), 6, CV_RGB(0,255,0), 2);
			cvCircle( ipm_image, cvPointFrom32f(dstQuad_[3]), 6, CV_RGB(0,255,0), 2);
			cvLine( ipm_image, cvPointFrom32f(dstQuad_[0]), cvPointFrom32f(dstQuad_[1]), cvScalar(255), 1);
			cvLine( ipm_image, cvPointFrom32f(dstQuad_[1]), cvPointFrom32f(dstQuad_[2]), cvScalar(255), 1);
			cvLine( ipm_image, cvPointFrom32f(dstQuad_[2]), cvPointFrom32f(dstQuad_[3]), cvScalar(255), 1);
			cvLine( ipm_image, cvPointFrom32f(dstQuad_[3]), cvPointFrom32f(dstQuad_[0]), cvScalar(255), 1);
			*/ 
			cvShowImage("ipm_window", ipm_image);
	    }
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
        
        image_processor_->Extract_Markers(ipm_image, scale_, markers_, training_frame_serial_, dstQuad_, projection_matrix_, markers_2nd_);
        markers_.header = info_msg -> header;
        markers_info_pub.publish(markers_);
        //
        markers_2nd_.header = info_msg -> header;
        markers_info_2nd_pub.publish(markers_2nd_);
        
        ROS_INFO("ImageCallBack finished");
        
        //Attention: 
        //color_image is not allocated memory as normal;
        //it points to some memory space in "cv_bridge" object handled by ros;
        //so there is no need and it is also not permitted to release its memory space as normal;
        //cvReleaseImage(&color_image);
        
        cvReleaseImage(&gray_image);
        cvReleaseImage(&ipm_image);

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
            temppose.position.z= 0.0;
            
            //this part takes into account that the road is not flat;
            //use the plane coefficient from "rolling_window";
            if(plane_ROI_.coefs.size() == 4)
            {
					ROS_INFO("compensate road unflatness;");
					if(plane_ROI_.coefs[2]!=0.0)
					{
						double nominator_tmp = (plane_ROI_.coefs[0]*temppose.position.x + plane_ROI_.coefs[1]*temppose.position.y 
														+ plane_ROI_.coefs[3]);
												
						temppose.position.z = - nominator_tmp/ plane_ROI_.coefs[2];
					}
					ROS_INFO("temppose.position.z %3f", temppose.position.z);
				}
            
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
          ROS_INFO("%5f, %5f, %5f, %5f", x_tmp, y_tmp, dst_pointer[i].x, dst_pointer[i].y);
      }
  }
   void ipm::curbPts_to_image(sensor_msgs::PointCloud &pts_3d, std::vector <CvPoint2D32f> & pts_image)
   {
		pts_image.clear();
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
		CvPoint2D32f pt_image;
		for(size_t i=0; i<pts_3d.points.size(); i++)
		{	
			temppose.position.x=pts_3d.points[i].x;
			temppose.position.y=pts_3d.points[i].y;
			temppose.position.z=pts_3d.points[i].z;
			
			tf::poseMsgToTF(temppose, tempTfPose);
			PoseInCamera = src_dest_tf_ * tempTfPose;
         tf::Point pt = PoseInCamera.getOrigin();
			cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
			if(pt.z()<0.0) continue;
         cv::Point2d uv;
         cam_model_.project3dToPixel(pt_cv, uv);
         pt_image.x = uv.x;
			pt_image.y = uv.y;
			pts_image.push_back(pt_image);
		}
	}
  
   void ipm::process_control(ros::Time meas_time)
   {
		publish_flag_ = false;
		
		try {
			ros::Duration timeout(1.0 / 30);
			tf_.waitForTransform("odom", "base_link", meas_time, timeout);
			tf_.lookupTransform("odom", "base_link",  meas_time, odom_meas_);
			}
		catch (tf::TransformException& ex) {
			ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
			return;
			}
		
		if(pub_init_)
		{
			tf::Transform odom_old_new  = odom_meas_old_.inverse() * odom_meas_;
			float tx, ty;
			tx = -odom_old_new.getOrigin().y();
			ty =  odom_old_new.getOrigin().x();
			float mov_dis = sqrtf(tx*tx + ty*ty);
			ROS_INFO("moving distance %3f", mov_dis);

			double yaw_dis, ttemp;
			odom_old_new.getBasis().getEulerYPR(yaw_dis, ttemp, ttemp);

			if(mov_dis > publish_dis_thresh_ || yaw_dis > publish_angle_thresh_)
			{
				publish_flag_ = true;
				odom_meas_old_ = odom_meas_;
			}
		}
		else
		{
			pub_init_ = true;
			publish_flag_ = true;
			odom_meas_old_ = odom_meas_;
		}
   }
   
   void ipm::planeCoefCallback(const rolling_window::plane_coef::ConstPtr& coef_in)
   {
		plane_ROI_ = *coef_in;
	}
  
  ipm::~ipm()
  {
	if(visualization_flag_)
	{
	   cvDestroyWindow("src_window");
	   cvDestroyWindow("ipm_window");
	}
    delete image_processor_;
    cvReleaseMat(&warp_matrix_);
    cvReleaseMat(&projection_matrix_);
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
