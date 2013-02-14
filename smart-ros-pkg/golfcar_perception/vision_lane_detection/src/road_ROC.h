#ifndef GOLFCAR_VISION_ROAD_ROC_H
#define GOLFCAR_VISION_ROAD_ROC_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <stdlib.h>
#include <float.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <geometry_msgs/PolygonStamped.h>
#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <fmutil/fm_math.h>
#include "lane_marker_common.h"
#include "svm_classifier.h"
#include "ransac_lane.h"

#include "ocr_client.h"
#include "word_identifier.h"

using namespace std;
using namespace ros;
using namespace tf;

#define LANES_CLUSTER_ANGLE_THRESH M_PI*10.0/180.0

namespace golfcar_vision{

	void line_calculate(CvBox2D box, double long_side_parameter[3], double short_side_parameter[3]);
	int is_equal( const void* _a, const void* _b, void* userdata );

	void line_calculate(CvBox2D box, double long_side_parameter[3], double short_side_parameter[3])
	{
		CvPoint2D32f point_tmp[4];
		calc_cvBoxPoints(box, point_tmp);
		double first_line_para[3], second_line_para[3];
		first_line_para[0] = point_tmp[0].y - point_tmp[1].y;
		first_line_para[1] = point_tmp[1].x - point_tmp[0].x;
		first_line_para[2] = point_tmp[0].x * point_tmp[1].y -  point_tmp[1].x * point_tmp[0].y;

		second_line_para[0] = point_tmp[2].y - point_tmp[1].y;
		second_line_para[1] = point_tmp[1].x - point_tmp[2].x;
		second_line_para[2] = point_tmp[2].x * point_tmp[1].y -  point_tmp[1].x * point_tmp[2].y;

		if(box.size.width>box.size.height)
		{
			long_side_parameter[0] = first_line_para[0];
			long_side_parameter[1] = first_line_para[1];
			long_side_parameter[2] = first_line_para[2];
			short_side_parameter[0] = second_line_para[0];
			short_side_parameter[1] = second_line_para[1];
			short_side_parameter[2] = second_line_para[2];
		}
		else
		{
			long_side_parameter[0] = second_line_para[0];
			long_side_parameter[1] = second_line_para[1];
			long_side_parameter[2] = second_line_para[2];
			short_side_parameter[0] = first_line_para[0];
			short_side_parameter[1] = first_line_para[1];
			short_side_parameter[2] = first_line_para[2];
		}
	}

	//use OpenCV cvSeqPartition();
	//http://opencv.willowgarage.com/documentation/clustering_and_search_in_multi-dimensional_spaces.html
	int is_equal( const void* _a, const void* _b, void* userdata )
	{
		size_t a = *(const size_t*)_a;
		size_t b = *(const size_t*)_b;

		CvSeq *contour = (CvSeq*) userdata;
		//printf("a, b %d,\t%d\t", a, b);

		CvSeq *contour_a = 0;
		CvSeq *contour_b = 0;

		size_t i=0;
		for (; contour != 0; contour = contour->h_next)
		{
			if(a==i) contour_a = contour;
			if(b==i)contour_b = contour;
			if(contour_a!=0 && contour_b!=0) break;
			i++;
		}

		CvBox2D cvBox_a, cvBox_b;
		CvMemStorage *mem_box_a = cvCreateMemStorage(0);
		CvMemStorage *mem_box_b = cvCreateMemStorage(0);

		cvBox_a = cvMinAreaRect2(contour_a, mem_box_a);
		cvBox_b = cvMinAreaRect2(contour_b, mem_box_b);
		//printf("cvBox_a center (%3f, %3f), width heigh: (%3f, %3f)\n", cvBox_a.center.x,  cvBox_a.center.y, cvBox_a.size.width, cvBox_a.size.height);
		//printf("cvBox_b center (%3f, %3f), width heigh: (%3f, %3f)\n", cvBox_b.center.x,  cvBox_b.center.y, cvBox_b.size.width, cvBox_b.size.height);

		CvPoint2D32f pointA[4], pointB[4];
		calc_cvBoxPoints(cvBox_a, pointA);
		calc_cvBoxPoints(cvBox_b, pointB);
		double longsideA[3], longsideB[3];
		double shortsideA[3], shortsideB[3];
		line_calculate(cvBox_a, longsideA, shortsideA);
		line_calculate(cvBox_b, longsideB, shortsideB);

		//printf("lineA long: (%3f,  %3f,  %3f)\t", longsideA[0], longsideA[1], longsideA[2]);
		//printf("lineA short: (%3f,  %3f,  %3f)\n", shortsideA[0], shortsideA[1], shortsideA[2]);
		//printf("lineB long: (%3f,  %3f,  %3f)\t", longsideB[0], longsideB[1], longsideB[2]);
		//printf("lineB short: (%3f,  %3f,  %3f)\n", shortsideB[0], shortsideB[1], shortsideB[2]);

		double longside_angleA = atan2(longsideA[0], -longsideA[1]);
		if(longside_angleA<0) longside_angleA = longside_angleA + M_PI;
		double longside_angleB = atan2(longsideB[0], -longsideB[1]);
		if(longside_angleB<0) longside_angleB = longside_angleB + M_PI;
		double delt_angle = fabs(longside_angleA-longside_angleB);
		if(delt_angle>M_PI_2) delt_angle = M_PI - delt_angle;
		bool angle_criterion = delt_angle < 5.0*M_PI/180.0;

		//printf("angle %3f \t", delt_angle);

		//distance of pointB[1] (second box) to the first short line;
		double distance = fabs(shortsideA[0]*pointB[1].x+shortsideA[1]*pointB[1].y+shortsideA[2])/sqrt(shortsideA[0]*shortsideA[0]+shortsideA[1]*shortsideA[1]);

		bool distance_criterion = distance < 2*30;

		//printf("distance %3f \n", distance);

		 cvReleaseMemStorage(&mem_box_a);
		 cvReleaseMemStorage(&mem_box_b);

		return (angle_criterion && distance_criterion);
	}



    class road_roc {
        public:
    	road_roc();
        ~road_roc();

        private:
        ros::NodeHandle nh_, private_nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        sensor_msgs::CvBridge bridge_;
        tf::TransformListener tf_;

        ros::Subscriber	polygon_sub_;
        bool polygon_init_;

        double center_x_, center_y_;
        bool fixedTf_inited_;

        int center_pix_x_, center_pix_y_;
        bool ipm_para_init_;

        double scale_;
        //polygon of interest in ipm image for processing;
        std::vector<CvPoint> ipm_polygon_;

        //flag decides whether to extract training images or not;
        bool extract_training_image_;
        
        golfcar_ml::svm_classifier *road_roc_classifier_;
        OcrClientNode lane_ocr_;
        word_identifier word_detector_;

        CvSeq* filter_contours (CvContourScanner &scanner);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        void polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& polygon_in);
        void extract_training_image(IplImage* binary_img);
        int  classify_contour(double weight_input, double perimeter_input, CvHuMoments &HM_input, CvBox2D &Box_input, int polyNum_input);

        long int frame_serial_;

        void IpmImage_to_pcl(std::vector <CvPoint2D32f> & pts_image, sensor_msgs::PointCloud &pts_3d);
        std::vector<size_t> cluster_contours (CvSeq* contour, std::vector <size_t> lane_serials);

        void MorphologicalThinning(CvMat *pSrc, CvMat *pDst);
        void ThinSubiteration1(CvMat *pSrc, CvMat *pDst);
        void ThinSubiteration2(CvMat *pSrc, CvMat *pDst);
    };
};

#endif
