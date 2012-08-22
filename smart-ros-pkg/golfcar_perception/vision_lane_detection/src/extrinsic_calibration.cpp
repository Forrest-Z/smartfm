//20120822
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "lane_marker_common.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace golfcar_vision;

int n_boards = 0;
const int board_dt = 20;
int board_w;
int board_h;
    
int main(int argc, char** argv) 
{
	 board_w = 5; // Board width in squares
	 board_h = 8; // Board height 
	 n_boards = 1; // Number of boards
	 int board_n = board_w * board_h;
	 CvSize board_sz = cvSize( board_w, board_h );
	 
	 IplImage *image=0;
    CvMat* image_points			= cvCreateMat( n_boards*board_n, 2, CV_32FC1 );
	 CvMat* object_points		= cvCreateMat( n_boards*board_n, 3, CV_32FC1 );
	 CvMat* point_counts			= cvCreateMat( n_boards, 1, CV_32SC1 );
	 CvMat* intrinsic_matrix	= cvCreateMat( 3, 3, CV_32FC1 );
	 CvMat* distortion_coeffs	= cvCreateMat( 5, 1, CV_32FC1 );
	
    if(argc != 2){return -1;}
	
    if((image = cvLoadImage( argv[1], 1)) == 0){
    return -1;}

	 CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
	 int corner_count;
	 int successes = 0;
	 
	 IplImage *gray_image = cvCreateImage( cvGetSize( image ), 8, 1 );

	 // Find chessboard corners:
	 int found = cvFindChessboardCorners( image, board_sz, corners,
														&corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

	 // Get subpixel accuracy on those corners
	 cvCvtColor( image, gray_image, CV_BGR2GRAY );
	 cvFindCornerSubPix( gray_image, corners, corner_count, cvSize( 11, 11 ), 
							   cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

	 // Draw it
	 cvDrawChessboardCorners( image, board_sz, corners, corner_count, found );
	 cvShowImage( "Calibration", image );

	 float scale_ratio = 1.0;
	 
	 // If we got a good board, add it to our data
	 if( corner_count == board_n ){
		 for( int i=0, j=0; j < board_n; ++i, ++j ){
		 	 CV_MAT_ELEM( *image_points,  float, i, 0 ) = corners[j].x;
			 CV_MAT_ELEM( *image_points,  float, i, 1 ) = corners[j].y;
			 CV_MAT_ELEM( *object_points, float, i, 0 ) = j/board_w * scale_ratio;
			 CV_MAT_ELEM( *object_points, float, i, 1 ) = j%board_w * scale_ratio;
			 CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
		 }
		CV_MAT_ELEM( *point_counts, int, successes, 0 ) = board_n;
		successes++;
		printf("---OK---corners accepted");
	}
	else { printf ("image not OK, please take another picture"); return 0;}
	
	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 462.55911;
	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 1 ) = 0.00;
	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 2 ) = 326.21463;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 0 ) = 0.0;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 472.34580;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 2 ) = 188.72264;
	CV_MAT_ELEM( *intrinsic_matrix, float, 2, 0 ) = 0.0;
	CV_MAT_ELEM( *intrinsic_matrix, float, 2, 1 ) = 0.0;
	CV_MAT_ELEM( *intrinsic_matrix, float, 2, 2 ) = 1.0;
	
	CV_MAT_ELEM( *distortion_coeffs, float, 0, 0 ) = -0.00791;
	CV_MAT_ELEM( *distortion_coeffs, float, 1, 0 ) = -0.05311;		
	CV_MAT_ELEM( *distortion_coeffs, float, 2, 0 ) = 0.00462;
	CV_MAT_ELEM( *distortion_coeffs, float, 3, 0 ) = 0.00101; 
	CV_MAT_ELEM( *distortion_coeffs, float, 4, 0 ) = 0.0;
	
	cvSave( "~/Intrinsics.xml", intrinsic_matrix );
	cvSave( "~/Distortion.xml", distortion_coeffs );
	
	CvMat* trans_vec = cvCreateMat(3,1,CV_32FC1);
	CvMat* rot_vec = cvCreateMat(3,1,CV_32FC1);
	CvMat* rot_matrix = cvCreateMat(3,3,CV_32FC1);
	
	cvFindExtrinsicCameraParams2(object_points, image_points, intrinsic_matrix, distortion_coeffs, rot_vec, trans_vec);
	cvRodrigues2(rot_vec, rot_matrix, NULL);
	float x_trans, y_trans, z_trans;
	x_trans = CV_MAT_ELEM( *distortion_coeffs, float, 0, 0 );
	y_trans = CV_MAT_ELEM( *distortion_coeffs, float, 1, 0 );
	z_trans = CV_MAT_ELEM( *distortion_coeffs, float, 2, 0 );
	printf("---------x_trans, y_trans, z_trans (%5f, %5f, %5f)----------\n", x_trans, y_trans, z_trans);

	float M11_rot = CV_MAT_ELEM( *rot_matrix, float, 0, 0 );
	float M21_rot = CV_MAT_ELEM( *rot_matrix, float, 1, 0 );
	float M31_rot = CV_MAT_ELEM( *rot_matrix, float, 2, 0 );
	float M32_rot = CV_MAT_ELEM( *rot_matrix, float, 2, 1 );
	float M33_rot = CV_MAT_ELEM( *rot_matrix, float, 2, 2 );
	
	double yaw, pitch, roll;
	yaw	  = atan2f(M21_rot, M11_rot);
	pitch = atan2f(- M31_rot, sqrtf(M32_rot*M32_rot+M33_rot*M33_rot));
	roll  = atan2f(M32_rot, M33_rot);
	printf("---------yaw, pitch, roll (%5f, %5f, %5f)----------\n", yaw, pitch, roll);
	
	CvMat* z_unit_vec = cvCreateMat(3,1,CV_32FC1);
	CvMat* z_new_vec  = cvCreateMat(3,1,CV_32FC1);
	
	CV_MAT_ELEM( *z_unit_vec, float, 0, 0 ) = 0.0;
	CV_MAT_ELEM( *z_unit_vec, float, 1, 0 ) = 0.0;
	CV_MAT_ELEM( *z_unit_vec, float, 2, 0 ) = 1.0;
	
	cvMatMul(rot_matrix, z_unit_vec, z_new_vec);
	
	float z_x = CV_MAT_ELEM( *z_new_vec, float, 0, 0 );
	float z_y = CV_MAT_ELEM( *z_new_vec, float, 1, 0 );
	float z_z = CV_MAT_ELEM( *z_new_vec, float, 2, 0 );
	printf("---------z_x, z_y, z_z (%5f, %5f, %5f)----------\n", z_x, z_y, z_z);
	
	assert(z_z<=1.0 && z_z>=-1.0);
	float z_angle = acosf(z_z);
	if(z_angle<0) z_angle = z_angle + M_PI;
	z_angle = z_angle/M_PI*180.0;
	
	printf("z angle from the camera z axis: %5f", z_angle);		
   return 0;
}

