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
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <cv_bridge/CvBridge.h>
#include <fmutil/fm_math.h>
#include "../data_type/activity_map.h"
#include <assert.h>

#include "road_semantics.h"


using namespace std;
using namespace ros;
using namespace cv;

namespace golfcar_semantics
{
    class AM_learner {

        public:
    	AM_learner(const char* image_path, double map_scale, pd_track_container* pd_container, road_semantics* road_semantics_analyzer);
        ~AM_learner();
    	void GridMap_init();
    	void learn_activity_map();
    	void view_activity_map();
    	void record_map_into_file();
        activity_map *AM_;

        private:
    	void accumulate_grid_activity(activity_grid &grid ,track_common track, size_t element_serial);
        void learn_moving_direction();

        void map_incorporating_sources();
        void moving_direction_GP();
        void obstacle_dist();
        void boundary_direction();

        void skel_direction();
        void find_nearest_points(double gridx, double gridy, vector<CvPoint> &deputy_points, int & nearestPt_ID, double & nearestDist);
        void direction_probe(int x_grid, int y_grid);

        void semantic_learning();
        void pedestrian_path();
        void ped_sidewalk();
        void ped_crossing();
        void ped_sourcesink();
        void ped_SSEM();
        void self_EM( vector<Vec2f>& samples_vector, int & cluster_N, double & loglikelihood_output);
        double prob_2DGaussian(Mat sample, Mat Mean, Mat CovMat);
        void draw_covMatrix_eclipse( Mat img, Mat CurrCovMat, Mat Means, int i);


        void show_moving_direction();

    	const char* image_path_;
    	double map_scale_;

    	string gp_file_;
		Rect gp_ROI_;
		double GPmean_min_,GPmean_max_,GPvar_min_,GPvar_max_;
		double GPmean_min2_,GPmean_max2_,GPvar_min2_,GPvar_max2_;
		road_semantics* road_semantics_analyzer_;
		vector <Point2f> SourceSinks_;

		Mat road_image_;

		//for visualization purpose of moving speed and direction;
		Rect visual_ROI_;
		void visualize_arrow_ROI();
		void drawArrow(Mat img, Point arrow_center, int trunk_lenth, double arrow_angle, int side_length, double alpha,  Scalar color, int thickness);
		void plotVar(Mat img, Point arrow_center, int side_length, float var_value);
    };
};

#endif
