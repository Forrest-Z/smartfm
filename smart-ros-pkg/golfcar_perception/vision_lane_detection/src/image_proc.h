#ifndef LANE_MARKER_IMAGE_PROC_H
#define LANE_MARKER_IMAGE_PROC_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <image_geometry/pinhole_camera_model.h>
#include <vector>
#include "lane_marker_common.h"
#include <stdlib.h>
#include "libsvm/svm.h"

//disable continuous lane temporarily, since golfcart 10.04 doesn't have required mrpt-library;
//#include "ransac_parabola.h"

#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

namespace golfcar_vision{

    class image_proc {
        public:
        image_proc(string svm_model_path, string svm_scale_path);
        ~image_proc();        

        void Extract_Markers (IplImage* src, float scale, vision_lane_detection::markers_info &markers_para, 
			      int &frame_serial, CvPoint2D32f* dst_pointer,
			      CvMat* projection_matrix, vision_lane_detection::markers_info &markers_para_2nd);
		
        private:
        bool init_flag_; 
        float scale_;
        CvPoint corners_[4];
        //left line1 and right line2;
        //y=Ax+C
        double para_A1_;
        double para_C1_;
        double para_A2_;
        double para_C2_;
        
        //flag decides whether to extract training images or not;
        bool extract_image_;
        
        CvSeq* Filter_candidates (CvContourScanner &scanner);
        bool CheckPointInside(CvPoint pt_para);
        bool CheckPointOffSideBounds(CvPoint pt_para);

        struct svm_model *svm_model_;
        int classify_contour(double weight_input, double perimeter_input, CvHuMoments &HM_input, 
				CvBox2D &Box_input, int polyNum_input);
        void pose_contour(CvSeq *contour, CvMoments &cvm, vision_lane_detection::marker_info &marker_para);
        void cvt_pose_baselink(vision_lane_detection::marker_info &marker_para);
	//void continuous_lane(CvSeq *contours, IplImage *contour_img, CvScalar ext_color);
		
	void restore_scalefile(string filename,double* &feature_min, double* &feature_max, int &feature_index);
	double output(int index, double value);
	char* readline(FILE *input);
	double* feature_max_;
	double* feature_min_;
	int feature_index_;
	double lower_, upper_;

	CvMat* intrinsic_A_, *distortion_coeffs_;
 	void pose_contour_Extrinsic(CvSeq *contour_raw, int contour_class, CvMat* projection_matrix,
						 vision_lane_detection::marker_info &marker_para);
	void corresPoints_extrinCalib(int contour_class, CvMat* srcMarkerPts, tf::Pose & markerPose_base);
	CvMat* M1_gndPts_, *M2_gndPts_, *M3_gndPts_;
    };
};

#endif
