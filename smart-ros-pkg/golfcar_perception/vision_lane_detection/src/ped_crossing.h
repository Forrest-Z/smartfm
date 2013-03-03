#ifndef GOLFCAR_VISION_PED_CROSSING_H
#define GOLFCAR_VISION_PED_CROSSING_H

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

using namespace std;
using namespace ros;
using namespace tf;

namespace golfcar_vision{
    class ped_crossing {
        public:
    	ped_crossing();
        ~ped_crossing();

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
        
        golfcar_ml::svm_classifier *ped_crossing_classifier_;
        ransac_lane *lane_extractor_;

        CvSeq* filter_contours (CvContourScanner &scanner);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        void polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& polygon_in);
        void extract_training_image(IplImage* binary_img);
        int  classify_contour(double weight_input, double perimeter_input, CvHuMoments &HM_input, CvBox2D &Box_input, int polyNum_input);


        long int frame_serial_;

        ros::Publisher markers_pub_;
        ros::Publisher markers_ptcloud_pub_;

        void IpmImage_to_pcl(std::vector <CvPoint2D32f> & pts_image, sensor_msgs::PointCloud &pts_3d);

        //int is_equal( const void* _a, const void* _b, void* userdata );
        std::vector<size_t> cluster_contours (CvSeq* contour, std::vector <size_t> lane_serials);
        //void line_calculate(CvBox2D box, double long_side_parameter[3], double short_side_parameter[3]);
    };
};

#endif
