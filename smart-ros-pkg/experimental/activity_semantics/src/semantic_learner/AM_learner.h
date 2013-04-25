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
        void view_activity_map();

        activity_map *AM_;

        private:
    	void GridMap_init(char* image_path, double map_scale);
    	double map_resolution_;
    	Mat road_map_;
   };
};

#endif
