/*
 * author: Baoxing
 * date:   2013/04/24
 */

#ifndef LOCAL_TRACK_SHOW_H
#define LOCAL_TRACK_SHOW_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include "../data_type/datatype_semantic.h"

using namespace std;
using namespace ros;

namespace golfcar_semantics{

    class local_track_show {
        public:
    	local_track_show(CvSize img_size, double show_scale);
        ~local_track_show();

        //extracted ped_semantic information;
        void show_update(double x, double y, CvPoint & prev_point, CvScalar bgr_color);

        private:
        CvSize image_size_;
        double show_scale_;
		IplImage *local_trajectory_img_;
		CvPoint lidar_center_;

		void draw_backGND();
		void drawArrow(IplImage* img, CvPoint pStart, CvPoint pEnd, int len, int alpha,  CvScalar color, int thickness=1, int lineType=8);
		void traj_pixel(double x, double y, CvPoint &img_pt);
   };
};

#endif
