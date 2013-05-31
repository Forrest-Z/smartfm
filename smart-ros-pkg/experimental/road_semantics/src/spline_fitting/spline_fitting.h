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
    	spline_fitting(vector<CvPoint> raw_input, double map_ratio, double control_length);
    	~spline_fitting();

    	void calc_road_length();
    	void cubicSpline_fitting();

    	bool alloc_flag_;

    	vector<CvPoint> raw_points_;
    	vector<CvPoint> output_points_;

    	double map_ratio_;
    	double control_length_;
    	double road_length_;

    	//GSL formatted data;
    	size_t data_point_Num_, breaks_Num_, coeffs_Num_;
    	gsl_bspline_workspace *bwx_, *bwy_;
    	gsl_multifit_linear_workspace *mwx_, *mwy_;

    	//pay attention:
    	//in our case, we should fit 2 separate splines for "x_" and "y_" values;
    	//road edge length "s_" serves as the horizontal axis;
    	gsl_vector *s_, *x_, *y_, *w_;
    	gsl_vector *Bx_, *By_;	//basis function values evaluated at si, i.e. B(s_i); actually Bx_, By_ should be the same, but for convenience we keep two independent copies;
    	gsl_vector *cx_, *cy_;
    	gsl_matrix *Xx_, *Xy_, *covx_, *covy_;

    	double chisqx_, Rsqx_, dofx_, tssx_;
    	double chisqy_, Rsqy_, dofy_, tssy_;
    };

};

#endif
