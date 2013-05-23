/*
 * Author: 	Baoxing
 * Date:	2013/May/23
 *
 * this is a class which incorporates another two classes:
 * (1) topo_extractor: to perform image thinning, extract road skeleton, and build its topology representation;
 * (2) topo_semantics: to perform semantic analysis based on road topology representation;
 *
 * inside this class, we provide several purposes;
 * (1) image binarization; binary image is needed by "topo_extractor";
 * (2) find covering area of each edge and node clusters;
 * (3) interface with further processing;
 */


#ifndef GOLFCAR_SEMANTICS_ROAD_SEMANTICS_H
#define GOLFCAR_SEMANTICS_ROAD_SEMANTICS_H

#include <iostream>
#include <float.h>
#include <math.h>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "topo_extractor/datatypes.hh"
#include "topo_extractor/utils.hh"
#include "topo_extractor/TopoExtractor.h"
#include "topo_semantic/TopoSemantic.h"

using namespace std;

namespace golfcar_semantics{

    class road_semantics {

    public:
    	road_semantics(string parameter_file);
    	~road_semantics();

        void image_processing();
        void network_semantics();
        void topo_cover_area();

        topo_extractor *topology_extractor_;
        topo_semantic  *semantic_extractor_;

        //task-specific map for further usage;

    };
};

#endif
