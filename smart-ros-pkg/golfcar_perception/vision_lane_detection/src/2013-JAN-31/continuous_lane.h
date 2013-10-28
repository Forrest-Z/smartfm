#ifndef LANE_MARKER_CONTINUOUS_LANE_H
#define LANE_MARKER_CONTINUOUS_LANE_H

#include "ransac/ransac_parabola.h"
#include "ransac/ransac_2dLines.h"
#include "lane_marker_common.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

namespace golfcar_vision{
	
    class continuous_lane 
    {
	     public:
        continuous_lane();
        ~continuous_lane(){};        
		  void ransac_lane(CvSeq *contours, IplImage *contour_img, CvScalar ext_color, vision_lane_detection::conti_lanes & lanes_inImg);
    };
};

#endif
