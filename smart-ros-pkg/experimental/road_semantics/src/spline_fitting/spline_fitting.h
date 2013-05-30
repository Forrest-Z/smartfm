/*
 * Author: 	Baoxing
 * Date:	2013/May/29
 *
 * Description: to model road segmnet as cubic spline; this class serves as a handy API;
 */

#ifndef GOLFCAR_SEMANTICS_SPLINE_FITTING_H
#define GOLFCAR_SEMANTICS_SPLINE_FITTING_H

#include <iostream>
#include <float.h>
#include <math.h>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_statistics.h>

using namespace std;
using namespace cv;

namespace golfcar_semantics{

    class spline_fitting {

    public:
    	spline_fitting();
    	~spline_fitting();

    	void network_semantics();

    private:
    	void parameter_init();
    };

};

#endif
