#ifndef PED_CROSSING_DETECTION_H
#define PED_CROSSING_DETECTION_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <image_geometry/pinhole_camera_model.h>
#include <vector>
#include "lane_marker_common.h"
#include "continuous_lane.h"
#include <stdlib.h>

#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#define LANES_CLUSTER_ANGLE_THRESH M_PI*10.0/180.0 

namespace golfcar_vision{

    class ped_cross_detect {
        public:
        ped_cross_detect(double scale);
        ~ped_cross_detect();        

        void Detect_PdCrossing (IplImage* src);
        CvSeq* Filter_candidates (CvContourScanner &scanner);
        continuous_lane* lane_extractor_;
        double scale_;
        std::vector<unsigned int> cluster_lines (vision_lane_detection::conti_lanes &lines);
    };
};

#endif
