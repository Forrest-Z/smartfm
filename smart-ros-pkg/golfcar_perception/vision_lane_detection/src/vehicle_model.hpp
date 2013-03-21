#ifndef VEHICLE_MODEL_HPP
#define VEHICLE_MODEL_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace ros;
using namespace tf;

namespace golfcar_vision{

    class vehicle_model {
        public:
    	vehicle_model(double length, double width, double height):length_(length),width_(width), height_(height)
    	{
    		skeleton3D_.resize(8, pcl::PointXYZ(0.0, 0.0, 0.0));
    		skeletonIMG_.resize(8, cvPoint(0, 0));

    		skeleton3D_[0].x=0.0; 	skeleton3D_[0].y=width/2.0; 	skeleton3D_[0].z=0.0;
    		skeleton3D_[1].x=0.0; 	skeleton3D_[1].y=-width/2.0; 	skeleton3D_[1].z=0.0;
    		skeleton3D_[2].x=0.0; 	skeleton3D_[2].y=-width/2.0; 	skeleton3D_[2].z=height;
    		skeleton3D_[3].x=0.0; 	skeleton3D_[3].y=width/2.0; 	skeleton3D_[3].z=height;
    		skeleton3D_[4].x=length; 	skeleton3D_[4].y=width/2.0; 	skeleton3D_[4].z=0.0;
    		skeleton3D_[5].x=length; 	skeleton3D_[5].y=-width/2.0; 	skeleton3D_[5].z=0.0;
    		skeleton3D_[6].x=length; 	skeleton3D_[6].y=-width/2.0; 	skeleton3D_[6].z=height;
    		skeleton3D_[7].x=length; 	skeleton3D_[7].y=width/2.0; 	skeleton3D_[7].z=height;
    	};
        ~vehicle_model(){};

        void DrawSkel(IplImage *image_draw, CvScalar color, int thickness)
        {
        	cvLine(image_draw, skeletonIMG_[0],skeletonIMG_[1], color, thickness);
        	cvLine(image_draw, skeletonIMG_[1],skeletonIMG_[2], color, thickness);
        	cvLine(image_draw, skeletonIMG_[2],skeletonIMG_[3], color, thickness);
        	cvLine(image_draw, skeletonIMG_[3],skeletonIMG_[0], color, thickness);

        	cvLine(image_draw, skeletonIMG_[4],skeletonIMG_[5], color, 1);
        	cvLine(image_draw, skeletonIMG_[5],skeletonIMG_[6], color, 1);
        	cvLine(image_draw, skeletonIMG_[6],skeletonIMG_[7], color, 1);
        	cvLine(image_draw, skeletonIMG_[7],skeletonIMG_[4], color, 1);

        	cvLine(image_draw, skeletonIMG_[0],skeletonIMG_[4], color, 1);
        	cvLine(image_draw, skeletonIMG_[1],skeletonIMG_[5], color, 1);
        	cvLine(image_draw, skeletonIMG_[2],skeletonIMG_[6], color, 1);
        	cvLine(image_draw, skeletonIMG_[3],skeletonIMG_[7], color, 1);
        };
        vector<pcl::PointXYZ> skeleton3D_;
        vector<CvPoint> skeletonIMG_;
        double length_, width_, height_;
   };
};

#endif

