#ifndef MODT_SHAPE_ANALYZER_H
#define MODT_SHAPE_ANALYZER_H

#
#include <vector>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <sensor_msgs/PointCloud.h>
#include "ransac_Lshape.hpp"

namespace mrpt{
	
    class shape_analyzer
    {
	     public:
		shape_analyzer();
		~shape_analyzer(){};
		void RANSAC_shape(sensor_msgs::PointCloud &input_cloud);
    };
};

#endif
