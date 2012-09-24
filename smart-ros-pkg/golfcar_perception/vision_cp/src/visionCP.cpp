#include "visionCP.h"


namespace golfcar_vision{
  
  visionCP::visionCP():
    private_nh_("~"),
    it_(nh_)
  {
      cam_sub_ = it_.subscribeCamera("camera_front/image_raw", 1, &visionCP::ImageCallBack, this);
		camera_init_ = false;
		intrinsic_matrix_	= cvCreateMat( 3, 3, CV_32FC1 );
		distortion_coeffs_	= cvCreateMat( 5, 1, CV_32FC1 );
		
		//to specify different boards;
		private_nh_.param("rect_height", rect_height_, 0.3);
		private_nh_.param("rect_width",  rect_width_,  0.5);
		private_nh_.param("binary_thresh", binary_thresh_, 50);
		private_nh_.param("cvpoly_thresh", cvpoly_thresh_, 5);
		private_nh_.param("rect_height", rect_height_, 0.185);
		private_nh_.param("rect_width", rect_width_, 0.255);
		private_nh_.param("angle_thresh", angle_thresh_, M_PI*30/180);
		private_nh_.param("ratio_thresh", ratio_thresh_, 0.3);
		private_nh_.param("red_detect",  red_detect_,  true);
		private_nh_.param("blue_detect", blue_detect_, true);
		private_nh_.param("visual_flag", visual_flag_, true);
		
		double tmp_tol;
		private_nh_.param("transform_tolerance", tmp_tol, 0.0);
		transform_tolerance_.fromSec(tmp_tol);
		
		board_pub_ = nh_.advertise<vision_cp::chess_board_poses>("cb_info",2);
		tfb_ = new tf::TransformBroadcaster();
  }
  
  void visionCP::ImageCallBack( const sensor_msgs::ImageConstPtr& image_msg,
										  const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
        ROS_INFO("visionCP: ImageCallBack");
        
        vision_cp::chess_board_poses poses_batch;
        poses_batch.header = info_msg -> header;
        //to initialize the intrinsic matrix of camera;
		  if(!camera_init_)
		  {
			  camera_init_ = true;
			  CameraStaticInfo_ = *info_msg;
			  CV_MAT_ELEM( *intrinsic_matrix_, float, 0, 0 ) = CameraStaticInfo_.K[0];
			  CV_MAT_ELEM( *intrinsic_matrix_, float, 0, 1 ) = CameraStaticInfo_.K[1];
			  CV_MAT_ELEM( *intrinsic_matrix_, float, 0, 2 ) = CameraStaticInfo_.K[2];
			  CV_MAT_ELEM( *intrinsic_matrix_, float, 1, 0 ) = CameraStaticInfo_.K[3];
			  CV_MAT_ELEM( *intrinsic_matrix_, float, 1, 1 ) = CameraStaticInfo_.K[4];
			  CV_MAT_ELEM( *intrinsic_matrix_, float, 1, 2 ) = CameraStaticInfo_.K[5];
			  CV_MAT_ELEM( *intrinsic_matrix_, float, 2, 0 ) = CameraStaticInfo_.K[6];
			  CV_MAT_ELEM( *intrinsic_matrix_, float, 2, 1 ) = CameraStaticInfo_.K[7];
			  CV_MAT_ELEM( *intrinsic_matrix_, float, 2, 2 ) = CameraStaticInfo_.K[8];
		
			  CV_MAT_ELEM( *distortion_coeffs_, float, 0, 0 ) = CameraStaticInfo_.D[0];
			  CV_MAT_ELEM( *distortion_coeffs_, float, 1, 0 ) = CameraStaticInfo_.D[1];		
			  CV_MAT_ELEM( *distortion_coeffs_, float, 2, 0 ) = CameraStaticInfo_.D[2];
			  CV_MAT_ELEM( *distortion_coeffs_, float, 3, 0 ) = CameraStaticInfo_.D[3]; 
			  CV_MAT_ELEM( *distortion_coeffs_, float, 4, 0 ) = CameraStaticInfo_.D[4];
		  }
		  
        IplImage* color_image;
        //get image in OpenCV format;
        try {
            color_image = bridge_.imgMsgToCv(image_msg, "rgb8");
            }
        catch (sensor_msgs::CvBridgeException& ex) {
            ROS_ERROR("Failed to convert image");
            return;
            }
		  //-------------------------------------------------------------------------------------------------------------------------
        //1. thresholding step, combining threshold and adaptive threshold methods, to get binary image;
        //-------------------------------------------------------------------------------------------------------------------------
        //It is short for "image threshold"; Iat "image adaptive threshold"; Itand is operates "and" on these two resulting images; 
        IplImage *R = 0, *G = 0, *B = 0;
        R = cvCreateImage(cvSize(color_image->width,color_image->height),IPL_DEPTH_8U, 1);
        G = cvCreateImage(cvSize(color_image->width,color_image->height),IPL_DEPTH_8U, 1);
        B = cvCreateImage(cvSize(color_image->width,color_image->height),IPL_DEPTH_8U, 1);
        cvSplit(color_image, R, G, B, 0);
        IplImage *Rtmp = 0, *Gtmp=0, *Btmp = 0, *tmp=0;
        tmp = cvCreateImage(cvSize(color_image->width,color_image->height),IPL_DEPTH_8U, 1);
        Rtmp = cvCloneImage(R);
        Gtmp = cvCloneImage(G);
        Btmp = cvCloneImage(B);
        
        //remember to suppress white color;
        cvScale(G, tmp, 0.5);
        cvSub(Rtmp, tmp, Rtmp);
        cvScale(B, tmp, 0.5);
        cvSub(Rtmp, tmp, Rtmp);
        cvScale(R, tmp, 0.5);
        cvSub(Btmp, tmp, Btmp);
        cvScale(G, tmp, 0.5);
        cvSub(Btmp, tmp, Btmp);
        
        if(visual_flag_)
        {
			  cvShowImage("R",R);
			  cvShowImage("G",G);
			  cvShowImage("B",B);
			  cvShowImage("Rtmp",Rtmp);
			  cvShowImage("Btmp",Btmp);
			  cvWaitKey(10);
		  }
        
        vision_cp::cb_pose board_pose;
        
        if(red_detect_)
        {
			  board_pose.label = 1;
			  if(visionCP::Extract_Rects (Rtmp, info_msg, std::string("/robot_1/board"), board_pose.pose )) poses_batch.cb_poses.push_back(board_pose);
		  }
		  
		  if(blue_detect_)
		  {
			  board_pose.label = 2;
			  if(visionCP::Extract_Rects (Btmp, info_msg, std::string("/robot_2/board"), board_pose.pose ))poses_batch.cb_poses.push_back(board_pose);
		  }
		  
		  board_pub_.publish(poses_batch);
		  
		  cvReleaseImage(&R);
		  cvReleaseImage(&G);
		  cvReleaseImage(&B);
		  cvReleaseImage(&tmp);
		  cvReleaseImage(&Rtmp);
		  cvReleaseImage(&Gtmp);
		  cvReleaseImage(&Btmp);
  }
  
  visionCP::~visionCP()
  {
    cvReleaseMat(&intrinsic_matrix_);
    cvReleaseMat(&distortion_coeffs_);
  }
  
    bool visionCP::Extract_Rects (IplImage* src, 
											 const sensor_msgs::CameraInfoConstPtr& info_msg, 
											 std::string board_name,
											 geometry_msgs::Pose & board_pose)
    {
		  bool pose_extracted = false;
		  ROS_INFO("extract rectangles");
        IplImage *It = 0;
        It = cvCreateImage(cvSize(src->width,src->height),IPL_DEPTH_8U, 1);
        //Threshold
        cvThreshold(src,It,binary_thresh_,255,CV_THRESH_BINARY);
        IplImage *contour_img = cvCreateImage(cvSize(src->width,src->height),IPL_DEPTH_8U, 3);
        cvCvtColor(It, contour_img, CV_GRAY2BGR);
        CvScalar ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 );
        
        //cvShowImage("It_image",It);
        
        CvSeq *contours = 0;            //"contours" is a list of contour sequences, which is the core of "image_proc";
        CvSeq *first_contour = 0;       //always keep one copy of the beginning of this list, for further usage;
        CvMemStorage *mem_contours; 
        mem_contours = cvCreateMemStorage(0);
        CvContourScanner scanner = cvStartFindContours(It, mem_contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        
		  CvSeq *contour_poly;
		  CvMemStorage *mem_poly;
		  mem_poly = cvCreateMemStorage(0);
				
        CvSeq* c;
        while((c=cvFindNextContour(scanner))!=NULL)
        {
				contour_poly = cvApproxPoly( c, sizeof(CvContour), mem_poly, CV_POLY_APPROX_DP, cvpoly_thresh_, 0 );
				if(contour_poly->total!=4)
				{
					//ROS_INFO("vertice number is bad : %d", contour_poly->total); 
					cvSubstituteContour(scanner, NULL);
				}
				else
				{
					std::vector <CvPoint> vertices;
					for(unsigned int i=0; i<contour_poly->total; i++)
					{
						 CvPoint* p = (CvPoint*)cvGetSeqElem(contour_poly, i);
						 vertices.push_back(*p);
					}
					bool angle_criteria = check_angles(vertices);
					
					if(!angle_criteria)
					{
						ROS_INFO("line angle is bad;"); 
						cvSubstituteContour(scanner, NULL);
					}
					else
					{
						float distance1 = sqrtf((vertices[1].x-vertices[0].x)*(vertices[1].x-vertices[0].x)+(vertices[1].y-vertices[0].y)*(vertices[1].y-vertices[0].y));
						float distance2 = sqrtf((vertices[2].x-vertices[1].x)*(vertices[2].x-vertices[1].x)+(vertices[2].y-vertices[1].y)*(vertices[2].y-vertices[1].y));
						float distance3 = sqrtf((vertices[3].x-vertices[2].x)*(vertices[3].x-vertices[2].x)+(vertices[3].y-vertices[2].y)*(vertices[3].y-vertices[2].y));
						bool paralell_criterion = true;
						bool intersect_criterion =  true;
						if(distance1>distance3+ratio_thresh_*distance3 || distance1<distance3-ratio_thresh_*distance3) paralell_criterion = false;
						float long_dis = distance1 > distance2 ? distance1:distance2;
						float short_dis = distance1 <= distance2 ? distance1:distance2;
						if(long_dis/short_dis > rect_width_/rect_height_ + ratio_thresh_ || long_dis/short_dis < rect_width_/rect_height_ - ratio_thresh_) intersect_criterion = false;
						if(!paralell_criterion || !intersect_criterion) 
						{
							ROS_INFO("rect length is bad;"); 
							cvSubstituteContour(scanner, NULL);
						}
					}
				}
        }
        contours = cvEndFindContours(&scanner);
        first_contour = contours;
        
        std::vector <CvPoint> vertices;
        CvMat* image_points	= cvCreateMat( 2*2, 2, CV_32FC1 );
        CvMat* object_points	= cvCreateMat( 2*2, 3, CV_32FC1 );
		  for( int i=0, j=0; j < 4; ++i, ++j )
		  {
			  CV_MAT_ELEM( *object_points, float, i, 0 ) = j/2 * rect_height_;
			  CV_MAT_ELEM( *object_points, float, i, 1 ) = j%2 * rect_width_;
			  CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
		  }
		  
		  //find the largest square;
		  CvMoments cvm; 
		  double contour_weight = 0.0;
		  CvSeq *contours_largest = 0;    
        for (; contours != 0; contours = contours->h_next)
        {
			  cvContourMoments(contours, &cvm);
			  if(cvm.m00 > contour_weight)
			  contour_weight = cvm.m00;
			  contours_largest = contours;
		  }
		  
        contours = contours_largest;
        if(contours != 0)
        {
			  cvDrawContours(contour_img, contours, ext_color, CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));
			  contour_poly = cvApproxPoly( contours, sizeof(CvContour), mem_poly, CV_POLY_APPROX_DP, cvpoly_thresh_, 0 );
			  for(unsigned int i=0; i<contour_poly->total; i++)
			  {
					CvPoint* p = (CvPoint*)cvGetSeqElem(contour_poly, i);
				   vertices.push_back(*p);
			  }
			  
			  visionCP::sort_pts(vertices);
			  for( int i=0, j=0; j < 4; ++i, ++j )
			  {
					 CV_MAT_ELEM( *image_points,  float, i, 0 ) = vertices[j].x;
					 CV_MAT_ELEM( *image_points,  float, i, 1 ) = vertices[j].y;
			  }
			  visionCP::calc_cb_pose(info_msg, board_name, object_points, image_points, board_pose);
			  pose_extracted = true;
		  }
		  else
		  {pose_extracted=false;
		  }
		  
		  if(visual_flag_)
		  {
			  cvShowImage("contour_img",contour_img);
			  cvWaitKey(10);
		  }
        
        cvReleaseMemStorage(&mem_contours);
        cvReleaseMemStorage(&mem_poly);
        cvReleaseImage(&It);
        cvReleaseImage(&contour_img);
        return pose_extracted;
    }
    
   //to be continued;
	void visionCP::sort_pts(std::vector <CvPoint> & vertices)
	{
		assert(vertices.size()==4);
	   std::vector <CvPoint> vertices_tmp;
	   std::vector<unsigned int> x_bigs, y_bigs;
	   for(unsigned int i=0; i<vertices.size(); i++)
	   {
			x_bigs.push_back(i);
			y_bigs.push_back(i);
			if(i<2){}
			else
			{
				int small_x, small_y;
				size_t x_serial, y_serial;
				for(size_t j=0; j<x_bigs.size(); j++)
				{
					if(j==0){small_x = vertices[x_bigs[j]].x; x_serial = j;}
					else
					{
						if(vertices[x_bigs[j]].x<small_x){small_x = vertices[x_bigs[j]].x; x_serial = j;}
					}
				}
				x_bigs.erase (x_bigs.begin()+x_serial);
				
				for(size_t j=0; j<y_bigs.size(); j++)
				{
					if(j==0){small_y = vertices[y_bigs[j]].y; y_serial = j;}
					else
					{
						if(vertices[y_bigs[j]].y<small_y){small_y = vertices[y_bigs[j]].y; y_serial = j;}
					}
				}
				y_bigs.erase (y_bigs.begin()+y_serial);
			}
		}
		size_t lower_right, lower_left, upper_right, upper_left;
		bool get_result=false;
		for(size_t ix=0; ix<x_bigs.size(); ix++)
		{
			if(get_result){break;}
			for(size_t iy=0; iy<y_bigs.size(); iy++)
			{
				if(x_bigs[ix]==y_bigs[iy]) 
				{
					lower_right=x_bigs[ix];
					x_bigs.erase(x_bigs.begin()+ix);
					y_bigs.erase(y_bigs.begin()+iy);
					get_result = true;
					break;
				}
			}
		}
		lower_left 	= 	y_bigs[0];
		upper_right = 	x_bigs[0];
		for(size_t i=0; i<4; i++)
		{
			if(i!=lower_right && i!=lower_left && i!=upper_right){upper_left = i; break;}
		}
		vertices_tmp.push_back(vertices[lower_right]);
		vertices_tmp.push_back(vertices[lower_left]);
		vertices_tmp.push_back(vertices[upper_right]);
		vertices_tmp.push_back(vertices[upper_left]);
		vertices = vertices_tmp;
		
		for(size_t i=0; i<vertices.size(); i++)
		{
			ROS_INFO("sorted Point: (%d, %d)", vertices[i].x, vertices[i].y);
		}
	}

  //"cb" is short for "chess board"
  void visionCP::calc_cb_pose( const sensor_msgs::CameraInfoConstPtr& info_msg, std::string board_name,
										CvMat* obj_pts, CvMat* img_pts, geometry_msgs::Pose & board_pose )
  {
	  CvMat* trans_vec = cvCreateMat(3,1,CV_32FC1);
	  CvMat* rot_vec = cvCreateMat(3,1,CV_32FC1);
	  CvMat* rot_matrix = cvCreateMat(3,3,CV_32FC1);
	  cvFindExtrinsicCameraParams2(obj_pts, img_pts, intrinsic_matrix_, distortion_coeffs_, rot_vec, trans_vec);
	  cvRodrigues2(rot_vec, rot_matrix, NULL);

		for( int i=0, j=0; j < 4; ++i, ++j )
		{
			float a = CV_MAT_ELEM( *obj_pts, float, i, 0 );
			float b = CV_MAT_ELEM( *obj_pts, float, i, 1 );
			float c = CV_MAT_ELEM( *img_pts,  float, i, 0 );
			float d = CV_MAT_ELEM( *img_pts,  float, i, 1 );
			ROS_INFO("object points: %3f, %3f", a, b);
			ROS_INFO("image points: %3f, %3f",  c, d);
		}
		  
	  board_pose.position.x = CV_MAT_ELEM( *trans_vec, float, 0, 0 );
	  board_pose.position.y = CV_MAT_ELEM( *trans_vec, float, 1, 0 );
	  board_pose.position.z = CV_MAT_ELEM( *trans_vec, float, 2, 0 );
	  ROS_INFO("%3f, %3f, %3f", board_pose.position.x, board_pose.position.y, board_pose.position.z);
	  
	  float M11_rot = CV_MAT_ELEM( *rot_matrix, float, 0, 0 );
	  float M21_rot = CV_MAT_ELEM( *rot_matrix, float, 1, 0 );
	  float M31_rot = CV_MAT_ELEM( *rot_matrix, float, 2, 0 );
	  float M32_rot = CV_MAT_ELEM( *rot_matrix, float, 2, 1 );
	  float M33_rot = CV_MAT_ELEM( *rot_matrix, float, 2, 2 );
	
	  double yaw, pitch, roll;
	  yaw	  = atan2f(M21_rot, M11_rot);
	  pitch = atan2f(- M31_rot, sqrtf(M32_rot*M32_rot+M33_rot*M33_rot));
	  roll  = atan2f(M32_rot, M33_rot);
	  board_pose.orientation = createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
	  
	  ros::Time transform_expiration = (info_msg->header.stamp + transform_tolerance_);
	  tf::Transform tmp_tf(tf::createQuaternionFromRPY(roll, pitch, yaw),
								  tf::Vector3(board_pose.position.x, board_pose.position.y, board_pose.position.z));	
	  tf::StampedTransform tmp_tf_stamped(tmp_tf, transform_expiration, info_msg->header.frame_id, board_name);	
	  tfb_->sendTransform(tmp_tf_stamped);
	  
	  cvReleaseMat(&trans_vec);
	  cvReleaseMat(&rot_vec);
	  cvReleaseMat(&rot_matrix);
  }

    bool visionCP::check_angles(std::vector <CvPoint> vertices)
    {
		 std::vector < pair<CvPoint, CvPoint> > lines;
		 for(unsigned int i=0; i<vertices.size(); i++)
		 {
			 ROS_INFO("Point: (%d, %d)", vertices[i].x, vertices[i].y);
			 if(i== vertices.size()-1) lines.push_back(make_pair(vertices[i], vertices[0]));
			 else{ lines.push_back(make_pair(vertices[i], vertices[i+1]));}
		 }
		 
		 for(unsigned int i=0; i<lines.size(); i++)
		 {
			 if(i==lines.size()-1) 
			 {
				float dx21 = lines[i].second.x - lines[i].first.x;
				float dx23 = lines[0].first.x - lines[0].second.x;
				float dy21 = lines[i].second.y - lines[i].first.y;
				float dy23 = lines[0].first.y- lines[0].second.y;
				//ROS_INFO("%3f, %3f, %3f, %3f", dx21, dx23, dy21, dy23);
				float m21 = sqrt( dx21*dx21 + dy21*dy21 );
				float m23 = sqrt( dx23*dx23 + dy23*dy23 );
				float theta = acos( (dx21*dx23 + dy21*dy23) / (m21 * m23));
				if(theta>= M_PI_2 + angle_thresh_|| theta<=M_PI_2 -angle_thresh_) 
				{
					//ROS_INFO("thetha: %3f", theta/M_PI*180);
					return false;
				}
				
			 }
			 else
			 {
				// "lines[i+1].first.x" equals "lines[i].second.x'
				float dx21 = lines[i].second.x - lines[i].first.x;
				float dx23 = lines[i+1].first.x - lines[i+1].second.x;
				float dy21 = lines[i].second.y - lines[i].first.y;
				float dy23 = lines[i+1].first.y - lines[i+1].second.y;
				ROS_INFO("%3f, %3f, %3f, %3f", dx21, dx23, dy21, dy23);
				float m21 = sqrt( dx21*dx21 + dy21*dy21 );
				float m23 = sqrt( dx23*dx23 + dy23*dy23 );
				float theta = acos( (dx21*dx23 + dy21*dy23) / (m21 * m23));
				if(theta>= M_PI_2 + angle_thresh_|| theta<=M_PI_2 -angle_thresh_) 
				{
					//ROS_INFO("thetha: %3f", theta/M_PI*180);
					return false;
				}
			 }
		 }
		 return true;
	 }
  
}

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "visionCP_node");
	 ros::NodeHandle n;
	 golfcar_vision::visionCP visionCP_node;
    ros::spin();
    return 0;
}
