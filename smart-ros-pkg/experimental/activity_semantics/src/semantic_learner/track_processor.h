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
    	track_processor(pd_track_container* pd_container, size_t track_size_thresh_, double track_time_thresh_, double track_length_thresh_);
        ~track_processor();
        pd_track_container *track_container_;
        void ped_track_classification();

		size_t track_size_thresh_;
		double track_time_thresh_, track_length_thresh_;
   };
};

#endif
