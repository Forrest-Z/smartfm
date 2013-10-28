#ifndef LANE_MARKER_RANSAC_LANE_H
#define LANE_MARKER_RANSAC_LANE_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "lane_marker_common.h"
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>

namespace golfcar_vision{
	
    class ransac_lane
    {
	     public:
		ransac_lane();
		~ransac_lane(){};

		void multiple_lanes(CvSeq *contours, double scale, IplImage *thining_img, IplImage *contour_img, int contour_serial, vision_lane_detection::lanes_info & lanes_inImg);
		void MorphologicalThinning(CvMat *pSrc, CvMat *pDst);

	     private:
		void ThinSubiteration1(CvMat *pSrc, CvMat *pDst);
		void ThinSubiteration2(CvMat *pSrc, CvMat *pDst);
    };
};

#endif
