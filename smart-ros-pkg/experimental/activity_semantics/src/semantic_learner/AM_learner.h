/*
 * AM_learner
 * author: Baoxing
 * date:   2013/04/25
 * description: activity-map learner, to learn semantics using map-oriented methods;
 *				this class will be used as an inserted class in "ped_semantics";
 */

#ifndef GOLFCAR_SEMANTICS_AM_LEARNER_H
#define GOLFCAR_SEMANTICS_AM_LEARNER_H

#include <ros/ros.h>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include "../data_type/activity_map.h"

using namespace std;
using namespace ros;
using namespace cv;

namespace golfcar_semantics
{

    class AM_learner {

        public:
    	AM_learner(char* image_path, double map_scale, pd_track_container* pd_container);
        ~AM_learner();

        void learn_activity_map();
        void learn_moving_direction();

        void view_activity_map();
        void show_moving_direction();

        activity_map *AM_;

        private:
    	void GridMap_init();
    	char* image_path_;
    	double map_scale_;
   };
};

#endif
