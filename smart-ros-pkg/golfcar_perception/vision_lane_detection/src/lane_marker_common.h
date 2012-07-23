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
#include "vision_lane_detection/marker_info.h"
#include "vision_lane_detection/markers_info.h"

//--------------------------------------------Parameters for "ipm"---------------------------------------------

//This number can be calculated dynamicallyfrom tf between "camera" and "base_link" in future version.
#define DIS_CAM_BASE_X  1.795

//Find four points on the ground, and their projection point should be inside the raw camera image;
//The difference in X-coordinates will decide GND_HEIGHT, 
//which further decide the pixel/meter ratio in the new ipm image, image->height/GND_HEIGHT;

//According to both calculation and tests, lowest part of the image is about 3 meters away from the camera;
//To calculate its coordinate in "base_link", X Coordinates plus "DIS_CAM_BASE_X";
#define RECT_P0_X 3.0
#define RECT_P0_Y 1.5
#define RECT_P1_X 3.0
#define RECT_P1_Y -1.5
#define RECT_P2_X 10.0
#define RECT_P2_Y -5.0
#define RECT_P3_X 10.0
#define RECT_P3_Y 5.0
//This value is actually calculated from 10.0-3.0;
#define GND_HEIGHT 7.0       

//-----------------------------------------Parameters for "image_proc"----------------------------------------
#define BINARY_THRESH           190
#define BLOCK_SIZE              41
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
#define CONTOUR_PERIMETER_THRESH        7.0
#define LONG_SIDE_THRESH                2.0
#define BOUNDARY_MARGIN                 2

using namespace std;

namespace golfcar_vision{

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
};

#endif
