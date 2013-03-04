/*
 * to provide basic msgs/functions that lane marker detection needs.
 */

#ifndef LANER_MARKER_COMMON_H
#define LANER_MARKER_COMMON_H

#include <cmath>
#include <algorithm>
#include <iostream>

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "sensor_msgs/PointCloud.h"
#include "vision_lane_detection/markers_info.h"
#include "vision_lane_detection/lanes_info.h"


//--------------------------------------------Parameters for "ipm"---------------------------------------------

//This number can be calculated dynamically from tf between "camera" and "base_link" in future version.
#define DIS_CAM_BASE_X  1.795

//Find four points on the ground, and their projection point should be inside the raw camera image;
//The difference in X-coordinates will decide GND_HEIGHT, 
//which further decide the pixel/meter ratio in the new ipm image, image->height/GND_HEIGHT;

//According to both calculation and tests, lowest part of the image is about 3 meters away from the camera;
//To calculate its coordinate in "base_link", X Coordinates plus "DIS_CAM_BASE_X";

//these parameters should change according to pitch angle of the camera;
//may make it self-adjusted later, according to tf, camera verticle fov, etc; at least adjustable from launch files.
//parameter for "filter_demo2.bag"

#define RECT_P0_X 3.0
#define RECT_P0_Y 2.0
#define RECT_P1_X 3.0
#define RECT_P1_Y -2.0
#define RECT_P2_X 15.0
#define RECT_P2_Y -10.0
#define RECT_P3_X 15.0
#define RECT_P3_Y 10.0
//This value is actually calculated from RECT_P2_X-RECT_P0_X;
#define GND_HEIGHT 12.0       

//-----------------------------------------Parameters for "image_proc"----------------------------------------
#define BINARY_THRESH           190
#define BLOCK_SIZE              25
//#define BLOCK_SIZE              65
#define OFFSET                  -10

//These parameters are all rough; even there true metrics may vary; 
//the drawings are not exactly the same in fact;
#define LETTER_HEIGHT                   2.5
#define ONE_HEAD_ARROW_WIDTH            0.44
#define ONE_HEAD_ARROW_HEIGHT           5.5
#define SPLITING_HEAD_ARROW_WIDTH       1.33
#define SPLITING_HEAD_ARROW_HEIGHT      5.5
#define FORK_ARROW_WIDTH                0.7
#define FORK_ARROW_HEIGHT               5.5
#define ZIGZAG_LINE_HEIGHT              4.17
#define CONTOUR_PERIMETER_THRESH        4.0
#define LONG_SIDE_THRESH                2.0
#define SHORT_SIDE_THRESH                0.3
#define BOUNDARY_MARGIN                 3

#define LANES_CLUSTER_ANGLE_THRESH M_PI*10.0/180.0

using namespace std;

namespace golfcar_vision{

void line_calculate(CvBox2D box, double long_side_parameter[3], double short_side_parameter[3]);
int is_equal( const void* _a, const void* _b, void* userdata );

    CvPoint2D32f centroid_centering_coordinate(CvPoint original_point, CvPoint2D32f centroid)
    {
        CvPoint2D32f new_point;
        new_point.x = -((float)original_point.y - centroid.y);
        new_point.y = -((float)original_point.x - centroid.x);
        return new_point;
    }

    unsigned int find_longest_distance(std::vector<float> &distance_vector)
    {
        float longest_distance = 0.0;
        unsigned int longest_serial = 0;
        for(unsigned int i=0; i<distance_vector.size(); i++)
        {
            if(distance_vector[i]>longest_distance)
            {
                longest_distance = distance_vector[i];
                longest_serial = i;
            }
        }
        return longest_serial;
    }

    void calc_cvBoxPoints( CvBox2D box, CvPoint2D32f pt[4] )
    {
      double angle = - box.angle*M_PI/180.0;
      float a = (float)cos(angle)*0.5f;
      float b = (float)sin(angle)*0.5f;
 
      pt[0].x = box.center.x - a*box.size.width - b*box.size.height;
      pt[0].y = box.center.y + b*box.size.width - a*box.size.height;
      pt[1].x = box.center.x + a*box.size.width - b*box.size.height;
      pt[1].y = box.center.y - b*box.size.width - a*box.size.height;
      pt[2].x = 2*box.center.x - pt[0].x;
      pt[2].y = 2*box.center.y - pt[0].y;
      pt[3].x = 2*box.center.x - pt[1].x;
      pt[3].y = 2*box.center.y - pt[1].y;
    } 

    void DrawBox(CvBox2D box, IplImage* img, CvScalar ext_color)
    {
          CvPoint2D32f point[4];
          int i;
          for ( i=0; i<4; i++)
          {
              point[i].x = 0;
              point[i].y = 0;
          }
          
         calc_cvBoxPoints(box, point); 
         CvPoint pt[4];
         
         for ( i=0; i<4; i++)
         {
             pt[i].x = (int)point[i].x;
             pt[i].y = (int)point[i].y;
         }
         cvLine( img, pt[0], pt[1], ext_color, 2, 8, 0 );
         cvLine( img, pt[1], pt[2], ext_color, 2, 8, 0 );
         cvLine( img, pt[2], pt[3], ext_color, 2, 8, 0 );
         cvLine( img, pt[3], pt[0], ext_color, 2, 8, 0 );
     }

	//http://alienryderflex.com/polygon/
    template <class T>
    bool pointInPolygon(T p, std::vector<T> poly)
	{
		int polySides = poly.size();
		int      i, j=polySides-1 ;
		bool  oddNodes = false      ;

		for (i=0; i<polySides; i++) {
			if (((poly[i].y< p.y && poly[j].y>=p.y)
			      ||   (poly[j].y< p.y && poly[i].y>=p.y))
					&&  (poly[i].x<=p.x || poly[j].x<=p.x)) {
				          if(poly[i].x+(p.y-poly[i].y)/(poly[j].y-poly[i].y)*(poly[j].x-poly[i].x)<p.x)
							{oddNodes=!oddNodes;} 			}
			j=i; }

		return oddNodes;
	}

    void line_calculate(CvBox2D box, double long_side_parameter[3], double short_side_parameter[3])
    {
    	CvPoint2D32f point_tmp[4];
    	calc_cvBoxPoints(box, point_tmp);
    	double first_line_para[3], second_line_para[3];
    	first_line_para[0] = point_tmp[0].y - point_tmp[1].y;
    	first_line_para[1] = point_tmp[1].x - point_tmp[0].x;
    	first_line_para[2] = point_tmp[0].x * point_tmp[1].y -  point_tmp[1].x * point_tmp[0].y;

    	second_line_para[0] = point_tmp[2].y - point_tmp[1].y;
    	second_line_para[1] = point_tmp[1].x - point_tmp[2].x;
    	second_line_para[2] = point_tmp[2].x * point_tmp[1].y -  point_tmp[1].x * point_tmp[2].y;

    	if(box.size.width>box.size.height)
    	{
    		long_side_parameter[0] = first_line_para[0];
    		long_side_parameter[1] = first_line_para[1];
    		long_side_parameter[2] = first_line_para[2];
    		short_side_parameter[0] = second_line_para[0];
    		short_side_parameter[1] = second_line_para[1];
    		short_side_parameter[2] = second_line_para[2];
    	}
    	else
    	{
    		long_side_parameter[0] = second_line_para[0];
    		long_side_parameter[1] = second_line_para[1];
    		long_side_parameter[2] = second_line_para[2];
    		short_side_parameter[0] = first_line_para[0];
    		short_side_parameter[1] = first_line_para[1];
    		short_side_parameter[2] = first_line_para[2];
    	}
    }

    //use OpenCV cvSeqPartition();
    //http://opencv.willowgarage.com/documentation/clustering_and_search_in_multi-dimensional_spaces.html
    int is_equal( const void* _a, const void* _b, void* userdata )
    {
    	size_t a = *(const size_t*)_a;
    	size_t b = *(const size_t*)_b;

    	CvSeq *contour = (CvSeq*) userdata;
    	printf("a, b %d,\t%d\t", a, b);

    	CvSeq *contour_a = 0;
    	CvSeq *contour_b = 0;

    	size_t i=0;
    	for (; contour != 0; contour = contour->h_next)
    	{
    		if(a==i) contour_a = contour;
    		if(b==i)contour_b = contour;
    		if(contour_a!=0 && contour_b!=0) break;
    		i++;
    	}

    	CvBox2D cvBox_a, cvBox_b;
    	CvMemStorage *mem_box_a = cvCreateMemStorage(0);
    	CvMemStorage *mem_box_b = cvCreateMemStorage(0);

    	cvBox_a = cvMinAreaRect2(contour_a, mem_box_a);
    	cvBox_b = cvMinAreaRect2(contour_b, mem_box_b);
    	printf("cvBox_a center (%3f, %3f), width heigh: (%3f, %3f)\n", cvBox_a.center.x,  cvBox_a.center.y, cvBox_a.size.width, cvBox_a.size.height);
    	printf("cvBox_b center (%3f, %3f), width heigh: (%3f, %3f)\n", cvBox_b.center.x,  cvBox_b.center.y, cvBox_b.size.width, cvBox_b.size.height);

    	CvPoint2D32f pointA[4], pointB[4];
    	calc_cvBoxPoints(cvBox_a, pointA);
    	calc_cvBoxPoints(cvBox_b, pointB);
    	double longsideA[3], longsideB[3];
    	double shortsideA[3], shortsideB[3];
    	line_calculate(cvBox_a, longsideA, shortsideA);
    	line_calculate(cvBox_b, longsideB, shortsideB);

    	printf("lineA long: (%3f,  %3f,  %3f)\t", longsideA[0], longsideA[1], longsideA[2]);
    	printf("lineA short: (%3f,  %3f,  %3f)\n", shortsideA[0], shortsideA[1], shortsideA[2]);
    	printf("lineB long: (%3f,  %3f,  %3f)\t", longsideB[0], longsideB[1], longsideB[2]);
    	printf("lineB short: (%3f,  %3f,  %3f)\n", shortsideB[0], shortsideB[1], shortsideB[2]);

    	double longside_angleA = atan2(longsideA[0], -longsideA[1]);
    	if(longside_angleA<0) longside_angleA = longside_angleA + M_PI;
    	double longside_angleB = atan2(longsideB[0], -longsideB[1]);
    	if(longside_angleB<0) longside_angleB = longside_angleB + M_PI;
    	double delt_angle = fabs(longside_angleA-longside_angleB);
    	if(delt_angle>M_PI_2) delt_angle = M_PI - delt_angle;
    	bool angle_criterion = delt_angle < 5.0*M_PI/180.0;

    	printf("angle %3f \t", delt_angle);

    	//distance of pointB[1] (second box) to the first short line;
    	double distance = fabs(shortsideA[0]*pointB[1].x+shortsideA[1]*pointB[1].y+shortsideA[2])/sqrt(shortsideA[0]*shortsideA[0]+shortsideA[1]*shortsideA[1]);

    	bool distance_criterion = distance < 2*30;

    	printf("distance %3f \n", distance);

    	 cvReleaseMemStorage(&mem_box_a);
    	 cvReleaseMemStorage(&mem_box_b);

    	return (angle_criterion && distance_criterion);
    }

	void  Img_preproc(IplImage *src, IplImage *binary_image)
	{
        IplImage *It = 0, *Iat = 0;
        It = cvCreateImage(cvSize(src->width,src->height),IPL_DEPTH_8U, 1);
        Iat = cvCreateImage(cvSize(src->width,src->height),IPL_DEPTH_8U, 1);
		cvThreshold(src,It,BINARY_THRESH,255,CV_THRESH_BINARY);
		cvAdaptiveThreshold(src, Iat, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, BLOCK_SIZE, OFFSET);
		cvAnd(It, Iat, binary_image);
		cvShowImage("It", It);
		cvShowImage("Iat", Iat);
		cvWaitKey(1);
		cvReleaseImage(&It);
		cvReleaseImage(&Iat);
	}

	void  Img_preproc_local(IplImage *src, IplImage *binary_image)
	{
		cvAdaptiveThreshold(src, binary_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, BLOCK_SIZE, OFFSET);
		cvShowImage("Iat_local", binary_image);
		cvWaitKey(1);
	}
};

#endif
