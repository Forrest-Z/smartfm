#include "visionCP.h"


namespace golfcar_vision{
  
  visionCP::visionCP():
    private_nh_("~"),
    it_(nh_)
  {
      cam_sub_ = it_.subscribeCamera("/camera_front/image_raw", 1, &visionCP::ImageCallBack, this);
		board_1st_switch_ = true;
		board_2nd_switch_ = true;
		detected_times_1st_ = 0;
		detected_times_2nd_ = 0;
		
		intrinsic_matrix_	= cvCreateMat( 3, 3, CV_32FC1 );
		distortion_coeffs_	= cvCreateMat( 5, 1, CV_32FC1 );
		
		//to specify different boards;
		board_1st_.board_w = 6;
		board_1st_.board_h = 8;
		board_1st_.side_length = 0.0345;
		board_2nd_.board_w = 4;
		board_2nd_.board_h = 5;
		board_2nd_.side_length = 0.20;
		
		board_pub_ = nh_.advertise<vision_cp::chess_board_poses>("cb_info",2);
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

        // Find chessboard corners:
        int board_n_1st = board_1st_.board_w * board_1st_.board_h;
		  CvSize board_sz_1st = cvSize( board_1st_.board_w , board_1st_.board_h );
        CvMat* image_points_1st	=	cvCreateMat( board_n_1st, 2, CV_32FC1 );
	     CvMat* object_points_1st	=	cvCreateMat( board_n_1st, 3, CV_32FC1 );
		  int corner_count_1st;
		  CvPoint2D32f* corners_1st = new CvPoint2D32f[ board_n_1st ];
	 
	     int board_n_2nd = board_2nd_.board_w * board_2nd_.board_h;
	     CvSize board_sz_2nd = cvSize( board_2nd_.board_w, board_2nd_.board_h );
	     CvMat* image_points_2nd	=	cvCreateMat( board_n_2nd, 2, CV_32FC1 );
	     CvMat* object_points_2nd	=	cvCreateMat( board_n_2nd, 3, CV_32FC1 );
	     int corner_count_2nd;
	     CvPoint2D32f* corners_2nd = new CvPoint2D32f[ board_n_2nd ];
	     
		  int found_1st = 0;
		  
		  //small tricks to speed up the program;  
		  if(board_1st_switch_)
		  found_1st = cvFindChessboardCorners( color_image, board_sz_1st, corners_1st,
														   &corner_count_1st, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
		  if(found_1st!=0)
		  {
			   detected_times_1st_ ++;
			   if(detected_times_1st_ > 10) board_2nd_switch_ = false;
			   
				cvFindCornerSubPix( gray_image, corners_1st, corner_count_1st, cvSize( 11, 11 ), 
											cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
				
											
				for( int i=0, j=0; j < board_n_1st; ++i, ++j ){
					 CV_MAT_ELEM( *image_points_1st,  float, i, 0 ) = corners_1st[j].x;
					 CV_MAT_ELEM( *image_points_1st,  float, i, 1 ) = corners_1st[j].y;
					 CV_MAT_ELEM( *object_points_1st, float, i, 0 ) = j/board_1st_.board_w * board_1st_.side_length;
					 CV_MAT_ELEM( *object_points_1st, float, i, 1 ) = j%board_1st_.board_w * board_1st_.side_length;
					 CV_MAT_ELEM( *object_points_1st, float, i, 2 ) = 0.0f;
				 }
				ROS_DEBUG("--- OK --- board1 -- detected");
				
				vision_cp::cb_pose board_pose;
				board_pose.label = 1;
				visionCP::calc_cb_pose(object_points_1st, image_points_1st, board_pose.pose);
				poses_batch.cb_poses.push_back(board_pose);
		 cvDrawChessboardCorners( color_image, board_sz_1st, corners_1st, corner_count_1st, found_1st );
			}
			else { ROS_DEBUG ("--- No --- board1 -- not detected");}
			
		  int found_2nd = 0;
		  if(board_2nd_switch_)
		  found_2nd = cvFindChessboardCorners( color_image, board_sz_2nd, corners_2nd,
														   &corner_count_2nd, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
										   
		  if(found_2nd!=0)
		  {
			   detected_times_2nd_ ++;
			   if(detected_times_2nd_ > 10) board_1st_switch_ = false;
			   
				cvFindCornerSubPix( gray_image, corners_2nd, corner_count_2nd, cvSize( 11, 11 ), 
											cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
				
											
				for( int i=0, j=0; j < board_n_2nd; ++i, ++j ){
					 CV_MAT_ELEM( *image_points_2nd,  float, i, 0 ) = corners_2nd[j].x;
					 CV_MAT_ELEM( *image_points_2nd,  float, i, 1 ) = corners_2nd[j].y;
					 CV_MAT_ELEM( *object_points_2nd, float, i, 0 ) = j/board_2nd_.board_w * board_2nd_.side_length;
					 CV_MAT_ELEM( *object_points_2nd, float, i, 1 ) = j%board_2nd_.board_w * board_2nd_.side_length;
					 CV_MAT_ELEM( *object_points_2nd, float, i, 2 ) = 0.0f;
				 }
				ROS_DEBUG("--- OK --- board2 -- detected");
				
				vision_cp::cb_pose board_pose;
				board_pose.label = 1;
				visionCP::calc_cb_pose(object_points_2nd, image_points_2nd, board_pose.pose);
				poses_batch.cb_poses.push_back(board_pose);
			cvDrawChessboardCorners( color_image, board_sz_2nd, corners_2nd, corner_count_2nd, found_2nd );
			}
			else { ROS_DEBUG ("--- No --- board2 -- not detected");}

	     cvShowImage( "Calibration", color_image );
		  
		  cvWaitKey(10);
       		  cvReleaseMat(&image_points_1st);
		  cvReleaseMat(&object_points_1st);
		  cvReleaseMat(&image_points_2nd);
		  cvReleaseMat(&object_points_2nd);
		  cvReleaseImage(&gray_image);

		  board_pub_.publish(poses_batch);
  }
  
  visionCP::~visionCP()
  {
    cvReleaseMat(&intrinsic_matrix_);
    cvReleaseMat(&distortion_coeffs_);
  }
  
  //"cb" is short for "chess board"
  void visionCP::calc_cb_pose(CvMat* obj_pts, CvMat* img_pts, geometry_msgs::Pose & board_pose )
  {
	  CvMat* trans_vec = cvCreateMat(3,1,CV_32FC1);
	  CvMat* rot_vec = cvCreateMat(3,1,CV_32FC1);
	  CvMat* rot_matrix = cvCreateMat(3,3,CV_32FC1);
	  cvFindExtrinsicCameraParams2(obj_pts, img_pts, intrinsic_matrix_, distortion_coeffs_, rot_vec, trans_vec);
	  cvRodrigues2(rot_vec, rot_matrix, NULL);
	  board_pose.position.x = CV_MAT_ELEM( *trans_vec, float, 0, 0 );
	  board_pose.position.y = CV_MAT_ELEM( *trans_vec, float, 1, 0 );
	  board_pose.position.z = CV_MAT_ELEM( *trans_vec, float, 2, 0 );

	  CvMat* z_unit_vec = cvCreateMat(3,1,CV_32FC1);
	  CvMat* z_new_vec  = cvCreateMat(3,1,CV_32FC1);
	  CV_MAT_ELEM( *z_unit_vec, float, 0, 0 ) = 0.0;
	  CV_MAT_ELEM( *z_unit_vec, float, 1, 0 ) = 0.0;
	  CV_MAT_ELEM( *z_unit_vec, float, 2, 0 ) = -1.0;
	  cvMatMul(rot_matrix, z_unit_vec, z_new_vec);
	  float z_x = CV_MAT_ELEM( *z_new_vec, float, 0, 0 );
	  float z_z = CV_MAT_ELEM( *z_new_vec, float, 2, 0 );
	  float board_yaw = -atan2f(z_z , z_x);
	  board_pose.orientation = createQuaternionMsgFromRollPitchYaw(0.0, 0.0, board_yaw);
	  ROS_DEBUG("board angle %3f", board_yaw);
	  cvReleaseMat(&trans_vec);
	  cvReleaseMat(&rot_vec);
	  cvReleaseMat(&rot_matrix);
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
