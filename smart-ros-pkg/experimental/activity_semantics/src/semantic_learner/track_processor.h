/*
 * track_processor
 * author: Baoxing
 * date:   2013/04/25
 * description: track_processor, to perform classification and other processing for pedestrian tracks;
 *				this class will be used as an inserted class in "ped_semantics";
 */

#ifndef GOLFCAR_SEMANTICS_TRACK_PROCESSOR_H
#define GOLFCAR_SEMANTICS_TRACK_PROCESSOR_H

#include <ros/ros.h>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/contrib/contrib.hpp>
#include <cv_bridge/CvBridge.h>
#include <fmutil/fm_math.h>

#include "../data_type/activity_map.h"


using namespace std;
using namespace ros;
using namespace cv;

namespace golfcar_semantics
{

    class track_processor {

        public:
    	track_processor();
        ~track_processor();

    	void GridMap_init();
    	void learn_activity_map();
    	void view_activity_map();
        activity_map *AM_;

        private:
    	void accumulate_grid_activity(activity_grid &grid ,track_common track, size_t element_serial);
        void learn_moving_direction();
        void show_moving_direction();

    	char* image_path_;
    	double map_scale_;
   };
};

#endif
