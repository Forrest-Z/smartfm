#include "ipm_pcl.h"


namespace golfcar_vision{
  
  ipm_pcl::ipm_pcl():
    private_nh_("~"),
    it_(nh_),
    fixedTf_inited_(false)
  {
      private_nh_.param("destination_frame_id", dest_frame_id_, std::string("base_link"));
      cam_sub_ = it_.subscribeCamera("/camera_front/image_raw", 1, &ipm_pcl::ImageCallBack, this);
      
      //Four base points on the ground in the "base_link" coordinate; "base_link" is at the back wheel.
      gndQuad_[0].x = RECT_P0_X+DIS_CAM_BASE_X;
      gndQuad_[0].y = RECT_P0_Y;
      gndQuad_[1].x = RECT_P1_X+DIS_CAM_BASE_X;
      gndQuad_[1].y = RECT_P1_Y;
      gndQuad_[2].x = RECT_P2_X+DIS_CAM_BASE_X;
      gndQuad_[2].y = RECT_P2_Y;
      gndQuad_[3].x = RECT_P3_X+DIS_CAM_BASE_X;
      gndQuad_[3].y = RECT_P3_Y;
		frame_serial_ = 0;
		warp_matrix_ = cvCreateMat(3,3,CV_32FC1);
		rbg_pub_ = nh_.advertise<PointCloudRGB>("pts_rgb", 10);
  }
 
  void ipm_pcl::ImageCallBack( const sensor_msgs::ImageConstPtr& image_msg,
										const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
        ROS_INFO("ImageCallBack");
			frame_serial_++;
			if(frame_serial_%50 !=0) return;
			
        ros::Time meas_time = info_msg->header.stamp;

        IplImage* color_image, *ipm_image;
        //get image in OpenCV format;
        try {
            color_image = bridge_.imgMsgToCv(image_msg, "bgr8");
            }
        catch (sensor_msgs::CvBridgeException& ex) {
            ROS_ERROR("Failed to convert image");
            return;
            }
            
        ipm_image = cvCreateImage(cvGetSize(color_image), 8, 3);
     
        //assign camera informtion to "cam_model_";
        cam_model_.fromCameraInfo(info_msg);
        CameraStaticInfo_ = *info_msg;
        scale_ = float(CameraStaticInfo_.height)/GND_HEIGHT;
      
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

			cvGetPerspectiveTransform(srcQuad_,dstQuad_,  warp_matrix_);
			
			////////////////////////////////////////////////
			//main functional part;
			////////////////////////////////////////////////
        cvWarpPerspective( color_image, ipm_image, warp_matrix_);
        
		  PointCloudRGB rgb_pts;
		  rgb_pts.header.stamp = info_msg->header.stamp;
		  rgb_pts.header.frame_id = "base_link";
		  IpmImage_to_pclrgb (ipm_image, rgb_pts);
		  rbg_pub_.publish(rgb_pts);       
        cvReleaseImage(&ipm_image);
  }
  
  //Function "GndPt_to_Src": project ground point in baselink coordinate into camera image;
  //input: pointer to ground points in "baselink coordinate"; output: pointer to pixels in "camera image";
  //steps: a. use "tf" relationship; b. use cam_model to project;
  void ipm_pcl::GndPt_to_Src(CvPoint2D32f * gnd_pointer, CvPoint2D32f* src_pointer)
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
  
  void ipm_pcl::GndPt_to_Dst(CvPoint2D32f * gnd_pointer, CvPoint2D32f* dst_pointer)
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
  
	void ipm_pcl::IpmImage_to_pclrgb(IplImage* pts_image, PointCloudRGB &pts_rgb)
	{
		geometry_msgs::Point32 pttmp;
		float center_x = (RECT_P0_X + RECT_P2_X)/2.0 + DIS_CAM_BASE_X;
		float center_y = 0.0;
		float center_pix_x = 320;
		float center_pix_y = 180;
		
		CvPoint pixel;
		int ipm_height 		= pts_image -> height;
		int ipm_width  		= pts_image -> width;
		for(int ih=0; ih < ipm_height; ih++)
		{
			for(int iw=0; iw < ipm_width; iw++)
			{
				pixel.x = iw;
				pixel.y = ih;
				CvScalar s=cvGet2D(pts_image, pixel.y, pixel.x);
		
				pcl::PointXYZRGB xyzRGB_pt;
				xyzRGB_pt.x = center_x -(pixel.y- center_pix_y)/scale_;
				xyzRGB_pt.y = center_y -(pixel.x - center_pix_x)/scale_;
				xyzRGB_pt.z = 0.0;
				xyzRGB_pt.r = s.val[0];
				xyzRGB_pt.g = s.val[1];
				xyzRGB_pt.b = s.val[2];
				pts_rgb.points.push_back(xyzRGB_pt);
			}
		}
	}
  
  ipm_pcl::~ipm_pcl()
  {
    cvReleaseMat(&warp_matrix_);
  }
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "ipm_pcl");
	 ros::NodeHandle n;
	 golfcar_vision::ipm_pcl ipm_pcl_node;
    ros::spin();
    return 0;
}
