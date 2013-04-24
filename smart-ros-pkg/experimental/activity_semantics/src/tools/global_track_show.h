/*
 * author: Baoxing
 * date:   2013/04/24
 */

#ifndef GLOBAL_TRACK_SHOW_H
#define GLOBAL_TRACK_SHOW_H

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

    class global_track_show
    {

        public:
    	global_track_show( const char * image_path, double show_scale);
        ~global_track_show();

        //extracted ped_semantic information;
        void show_update(double x, double y, CvScalar bgr_color);

        private:
        double show_scale_;
		IplImage *global_trajectory_img_;
   };
};

#endif
