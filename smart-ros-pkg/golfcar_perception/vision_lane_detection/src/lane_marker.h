#ifndef GOLFCAR_VISION_LANE_MARKER_H
#define GOLFCAR_VISION_LANE_MARKER_H

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

using namespace std;
using namespace ros;
using namespace tf;

namespace golfcar_vision{

    class lane_marker {
        public:
    	lane_marker();
        ~lane_marker();

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
        bool extract_training_image_, store_parameter_;
        string image_folder_path_;

        golfcar_ml::svm_classifier *marker_classifier_;

        CvSeq* filter_contours (CvContourScanner &scanner);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        int classify_contour(double weight_input, double perimeter_input, CvHuMoments &HM_input, CvBox2D &Box_input, int polyNum_input);
        void pose_contour(CvSeq *contour, CvMoments &cvm, vision_lane_detection::marker_info &marker_para);
        void cvt_pose_baselink(vision_lane_detection::marker_info &marker_para);
        void extract_training_image(IplImage* binary_img);
        void polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& polygon_in);
        long int frame_serial_;

        ros::Publisher markers_pub_;
        ros::Publisher markers_ptcloud_pub_;

        void IpmImage_to_pcl(std::vector <CvPoint2D32f> & pts_image, sensor_msgs::PointCloud &pts_3d);

        //2013-March
        bool mask_init_;
        IplImage* image_mask_;

    };
};

#endif
