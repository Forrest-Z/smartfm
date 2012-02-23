#include "visualizer.h"
#include "cv_helper.h"
#include <iomanip>
#include <iostream>

using namespace cv;
using namespace std;

HOGVisualizer::HOGVisualizer(ros::NodeHandle &n) : n_(n), it_(n_)
{
    ros::NodeHandle nh("~");
    nh.param("ROI_text", ROI_text, true);
    nh.param("verified_text", verified_text, false);
    nh.param("vision_rect", vision_rect, false);
    nh.param("ROI_rect", ROI_rect, false);
    nh.param("verified_rect", verified_rect, true);
    /// get image from the USB cam
    image_sub_.subscribe(it_,"/usb_cam/image_raw",20);

    /// start processign only after the first image is present
    started = false;

    /// from the laser ( blue rect )

    people_detect_sub_.subscribe(n, "pedestrian_detect", 20);

    /// from vision verified (green rect)
    people_roi_sub_.subscribe(n, "veri_pd_vision", 20);

    /// Final confirmation
    people_verified_sub_ .subscribe(n, "veri_show_batch", 20);

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, pedestrian_vision_batch, pedestrian_vision_batch, pedestrian_vision_batch> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), image_sub_, people_detect_sub_, people_roi_sub_, people_verified_sub_);
    sync.registerCallback(boost::bind(&HOGVisualizer::peopleCallback,this, _1, _2, _3, _4));

    cvNamedWindow("HOG Visualizer");

    ros::spin();
}

HOGVisualizer::~HOGVisualizer(){}

void HOGVisualizer::peopleCallback(const sensor_msgs::ImageConstPtr image, const pedestrian_vision_batchConstPtr laser_detect, const pedestrian_vision_batchConstPtr vision_roi, const pedestrian_vision_batchConstPtr verified)
{
    Mat img;
    Cv_helper::sensormsgsToCv(image, img);

    /// roi_rects_ : laser based ( blue )
    /// detect_rects_ : vision based detection ( green )
    /// verified_rects_ : final confirmation  (red )


    /// visualizer : Add detection id text

    for(unsigned int i=0;i<laser_detect->pd_vector.size();i++)
    {
        if(vision_rect) rectangle(img, Point(laser_detect->pd_vector[i].cvRect_x1, laser_detect->pd_vector[i].cvRect_y1) , Point(laser_detect->pd_vector[i].cvRect_x2, laser_detect->pd_vector[i].cvRect_y2), Scalar(0,255,0), 1);
    }


    //need to refill the cvRect points;


    for(unsigned int i=0;i<verified->pd_vector.size();i++)
    {
        Size roi_size; Point roi_point;
        sensing_on_road::pedestrian_vision temp_rect = verified->pd_vector[i];
        Cv_helper::fillRoiRectangle(Size(640, 360), &roi_size, &roi_point, temp_rect );
        temp_rect.cvRect_x1=roi_point.x;temp_rect.cvRect_y1=roi_point.y;
        temp_rect.cvRect_x2=roi_point.x+roi_size.width;temp_rect.cvRect_y2=roi_point.y+roi_size.height;
        if(verified_rect) rectangle(img,Point(temp_rect.cvRect_x1, temp_rect.cvRect_y1) , Point(temp_rect.cvRect_x2, temp_rect.cvRect_y2), Scalar(0,0,255), 2);
        if(verified_text) drawIDandConfidence(img, temp_rect);
    }

    for(unsigned int i=0;i<vision_roi->pd_vector.size();i++)
    {
        Point UL = Point(vision_roi->pd_vector[i].cvRect_x1, vision_roi->pd_vector[i].cvRect_y1);
        Point BR = Point(vision_roi->pd_vector[i].cvRect_x2, vision_roi->pd_vector[i].cvRect_y2);
        sensing_on_road::pedestrian_vision temp_rect = vision_roi->pd_vector[i];
        if(ROI_rect) rectangle(img,UL, BR, Scalar(255,0,0), 1);
        if(ROI_text) drawIDandConfidence(img, temp_rect);
    }
    started = true;
    imshow("HOG Visualizer", img);
    cvWaitKey(3);
}

void HOGVisualizer::drawIDandConfidence(Mat& img, sensing_on_road::pedestrian_vision& pv)
{
    std::stringstream ss,ss2;
    ss<<pv.object_label;
    Point BR = Point(pv.cvRect_x2, pv.cvRect_y2);
    Point BL = Point(pv.cvRect_x1, pv.cvRect_y2);
    putText(img, ss.str(), BL+Point(2,-2), FONT_HERSHEY_PLAIN, 0.8, cvScalar(0,0,255), 1, 8);
    ss2<<setprecision(2)<<fixed<<pv.confidence*100.0;
    putText(img, ss2.str(), BR+Point(-45,-2), FONT_HERSHEY_PLAIN, 0.8, cvScalar(0,0,255), 1, 8);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "HOGVisualizerVis");
    ros::NodeHandle n;
    HOGVisualizer ic(n);
    ros::spin();
    return 0;
}
