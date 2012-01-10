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
    image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &HOGVisualizer::imageCallback, this);
    /// start processign only after the first image is present
    started = false;

    /// from the laser ( blue rect )
    people_detect_sub_ = n.subscribe("pedestrian_detect", 1, &HOGVisualizer::peopleDetectCallback, this);

    /// from vision verified (green rect)
    people_roi_sub_ = n.subscribe("veri_pd_vision", 1, &HOGVisualizer::peopleRoiCallback, this);

    /// Final confirmation
    people_verified_sub_ = n.subscribe("veri_show_batch", 1, &HOGVisualizer::peopleVerifiedCallback, this);

    cvNamedWindow("Image window");
}

HOGVisualizer::~HOGVisualizer(){}
void HOGVisualizer::peopleVerifiedCallback(sensing_on_road::pedestrian_vision_batch pr)
{/// final confirmation

    /// No filtering done here
    if(started)
    {
        verified_rects_.pd_vector.clear();
        verified_rects_=pr;
    }
}
void HOGVisualizer::peopleRoiCallback(sensing_on_road::pedestrian_vision_batch pr)
{/// confirmed by HOG
    if(started)
    {
        roi_rects_.pd_vector.clear();
        //for(int i=0;i<roi_rects_.pd_vector.size();i++)
        //roi_rects_.pd_vector.push_back(pr);		
        roi_rects_=pr;
    }
}
void HOGVisualizer::peopleDetectCallback(sensing_on_road::pedestrian_vision_batch pr)
{
    if(started)
    {
        detect_rects_.pd_vector.clear();
        detect_rects_=pr;
    }
}	

void HOGVisualizer::imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{		
    cv_bridge::CvImagePtr cv_image;
    try
    {
        cv_image = cv_bridge::toCvCopy(msg_ptr, "bgra8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    Mat img(cv_image->image);



    /// roi_rects_ : laser based ( blue )
    /// detect_rects_ : vision based detection ( green )
    /// verified_rects_ : final confirmation  (red )


    /// visualizer : Add detection id text

    for(unsigned int i=0;i<detect_rects_.pd_vector.size();i++)
    {
        if(vision_rect) rectangle(img, Point(detect_rects_.pd_vector[i].cvRect_x1, detect_rects_.pd_vector[i].cvRect_y1) , Point(detect_rects_.pd_vector[i].cvRect_x2, detect_rects_.pd_vector[i].cvRect_y2), Scalar(0,255,0), 1);
    }


    //need to refill the cvRect points;


    for(unsigned int i=0;i<verified_rects_.pd_vector.size();i++)
    {
        Size roi_size; Point roi_point;
        Cv_helper cv_help;
        cv_help.fillRoiRectangle(Size(640, 360), &roi_size, &roi_point, verified_rects_.pd_vector[i]);
        sensing_on_road::pedestrian_vision temp_rect = verified_rects_.pd_vector[i];
        temp_rect.cvRect_x1=roi_point.x;temp_rect.cvRect_y1=roi_point.y;
        temp_rect.cvRect_x2=roi_point.x+roi_size.width;temp_rect.cvRect_y2=roi_point.y+roi_size.height;
        if(verified_rect) rectangle(img,Point(temp_rect.cvRect_x1, temp_rect.cvRect_y1) , Point(temp_rect.cvRect_x2, temp_rect.cvRect_y2), Scalar(0,0,255), 2);
        if(verified_text) drawIDandConfidence(img, temp_rect);
    }

    for(unsigned int i=0;i<roi_rects_.pd_vector.size();i++)
    {
      Point UL = Point(roi_rects_.pd_vector[i].cvRect_x1, roi_rects_.pd_vector[i].cvRect_y1);
      Point BR = Point(roi_rects_.pd_vector[i].cvRect_x2, roi_rects_.pd_vector[i].cvRect_y2);
      if(ROI_rect) rectangle(img,UL, BR, Scalar(255,0,0), 1);
      if(ROI_text) drawIDandConfidence(img, roi_rects_.pd_vector[i]);
    }
    started = true;
    imshow("Image window", img);
    cvWaitKey(3);

}	

void HOGVisualizer::drawIDandConfidence(Mat& img, sensing_on_road::pedestrian_vision& pv)
{
    std::stringstream ss,ss2;
    ss<<pv.object_label;
    Point BR = Point(pv.cvRect_x2, pv.cvRect_y2);
    Point BL = Point(pv.cvRect_x1, pv.cvRect_y2);
    putText(img, ss.str(), BL+Point(2,-2), FONT_HERSHEY_PLAIN, 0.8, cvScalar(0,255,255), 1, 8);
    ss2<<setprecision(2)<<fixed<<pv.confidence*100.0;
    putText(img, ss2.str(), BR+Point(-45,-2), FONT_HERSHEY_PLAIN, 0.8, cvScalar(0,255,255), 1, 8);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "HOGVisualizerVis");
    ros::NodeHandle n;
    HOGVisualizer ic(n);
    ros::spin();
    return 0;
}
