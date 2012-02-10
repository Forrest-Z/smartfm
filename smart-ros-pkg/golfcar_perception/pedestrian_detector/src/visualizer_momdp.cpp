#include <iomanip>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensing_on_road/pedestrian_vision_batch.h>
#include <sensing_on_road/pedestrian_vision.h>

#include <ped_momdp_sarsop/peds_believes.h>

#include "cv_helper.h"


struct pedBelife_vis
{
    double left_side, right_side;
    int decision;
    int id;
};

class VisualizeMomdp
{
public:
    VisualizeMomdp(ros::NodeHandle &n);

private:
    ros::NodeHandle n_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber ped_bel_sub_, people_roi_sub_;

    bool started, ROI_text, verified_text, vision_rect, verified_rect, ROI_rect;
    vector<pedBelife_vis> ped_bel;

    ros::Time vision_roi_time_;
    sensing_on_road::pedestrian_vision_batch vision_roi_;

    void imageCallback(const sensor_msgs::ImageConstPtr image);
    void roiCallback(const sensing_on_road::pedestrian_vision_batchConstPtr vision_roi);
    void drawIDandConfidence(cv::Mat& img, sensing_on_road::pedestrian_vision& pv);
    void pedBeliefCallback(ped_momdp_sarsop::peds_believes ped_bel);
};

VisualizeMomdp::VisualizeMomdp(ros::NodeHandle &n) : n_(n), it_(n_)
{
    ros::NodeHandle nh("~");
    nh.param("ROI_text", ROI_text, true);
    nh.param("ROI_rect", ROI_rect, false);

    // get image from the USB cam
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, boost::bind(&VisualizeMomdp::imageCallback, this, _1));

    // start processign only after the first image is present
    started = false;

    // from laser (green rect)
    people_roi_sub_ = n.subscribe("veri_pd_vision", 1, &VisualizeMomdp::roiCallback, this);

    ped_bel_sub_ = n.subscribe("peds_believes", 1, &VisualizeMomdp::pedBeliefCallback, this);
}

void VisualizeMomdp::roiCallback(const sensing_on_road::pedestrian_vision_batchConstPtr vision_roi)
{
    vision_roi_ = *vision_roi;
    vision_roi_time_ = vision_roi->header.stamp;
}

void VisualizeMomdp::imageCallback(const sensor_msgs::ImageConstPtr image)
{
    ROS_INFO("imageCallback");
    Mat img;
    Cv_helper::sensormsgsToCv(image, img);

    if( image->header.stamp - vision_roi_time_ < ros::Duration(0.5) ) {
        for(unsigned int i=0; i<vision_roi_.pd_vector.size(); i++)
        {
            Point UL = Point(vision_roi_.pd_vector[i].cvRect_x1, vision_roi_.pd_vector[i].cvRect_y1);
            Point BR = Point(vision_roi_.pd_vector[i].cvRect_x2, vision_roi_.pd_vector[i].cvRect_y2);
            sensing_on_road::pedestrian_vision temp_rect = vision_roi_.pd_vector[i];
            if(ROI_rect) rectangle(img,UL, BR, Scalar(255,0,0), 1);
            if(ROI_text) drawIDandConfidence(img, temp_rect);

        }
    }

    started = true;
    imshow("MOMDP Visualizer", img);
    cvWaitKey(3);
}

void VisualizeMomdp::pedBeliefCallback(ped_momdp_sarsop::peds_believes ped_bel_cb)
{
    ped_bel.resize(ped_bel_cb.believes.size());
    for(size_t i=0; i<ped_bel_cb.believes.size(); i++)
    {
        double left =  ped_bel_cb.believes[i].belief_value[0] + ped_bel_cb.believes[i].belief_value[3];
        double right =  ped_bel_cb.believes[i].belief_value[1] + ped_bel_cb.believes[i].belief_value[2];
        int decision;

        if(ped_bel_cb.believes[i].action==1)
            decision=1;
        else if(ped_bel_cb.believes[i].action==2)
            decision=-1;
        else if(ped_bel_cb.believes[i].action==0)
        {
            if(ped_bel_cb.robotv>0.1)
                decision=1;
            else if( ped_bel_cb.robotv < 0.1)
                decision=-1;
        }

        ped_bel[i].decision = decision;
        ped_bel[i].left_side = left;
        ped_bel[i].right_side = right;
        ped_bel[i].id = ped_bel_cb.believes[i].ped_id;
    }

}

void VisualizeMomdp::drawIDandConfidence(Mat& img, sensing_on_road::pedestrian_vision& pv)
{
    std::stringstream ss,ss2;
    float offset = 10.0;
    ss<<pv.object_label;
    /// first row
    Point UL = Point(pv.cvRect_x1, pv.cvRect_y1); /// top left
    Point UR = Point(pv.cvRect_x2, pv.cvRect_y1); /// top left
    Point UR_UP = Point(pv.cvRect_x2, pv.cvRect_y1-offset); /// top left
    Point UC = Point((pv.cvRect_x2+pv.cvRect_x1)/2, pv.cvRect_y1); /// top mid
    Point UP_C_OFF = Point((pv.cvRect_x1+pv.cvRect_x2)/2, pv.cvRect_y1+offset); /// top thickness
    Point UL_OFF = Point(pv.cvRect_x1, pv.cvRect_y1+offset); /// topleft thickness
    //Point UR_OFF = Point(pv.cvRect_x2, pv.cvRect_y1+offset); /// topright thickness

    /// second row
    Point UR_2OFF = Point(pv.cvRect_x2, pv.cvRect_y1+(2*offset)); /// 2nd row mid thickness
    Point UR_OFF = Point(pv.cvRect_x2, pv.cvRect_y1+offset);
    Point BR = Point(pv.cvRect_x2, pv.cvRect_y2); /// bot left
    Point BL = Point(pv.cvRect_x1, pv.cvRect_y2); /// bot right

    for(size_t i=0; i<ped_bel.size(); i++)
    {

        if(ped_bel[i].id == pv.object_label)
        {
            std::cout <<ped_bel[i].id <<" " <<ped_bel[i].left_side <<" " <<ped_bel[i].right_side <<std::endl;

            //std::cout << " x " << pv.cvRect_x1 << " y " << pv.cvRect_y1;
            //std::cout << " x " << pv.cvRect_x2 << " y " << pv.cvRect_y2 << std::endl;;

            /// Gradient Coding
            //int left_gradient = 255-(255.0*ped_bel[i].left_side);
            //int right_gradient = 255-(255.0*ped_bel[i].right_side);

            //rectangle(img,UL, UP_C_OFF, Scalar(255,left_gradient,left_gradient), CV_FILLED);
            //rectangle(img,UL, UP_C_OFF, Scalar(255,left_gradient,left_gradient), CV_FILLED);
            //rectangle(img,UC, UR_OFF, Scalar(255,right_gradient,right_gradient), CV_FILLED);

            /// size encoding
            //rectangle(img,UL, UP_C_OFF, Scalar(255,255,255), CV_FILLED);
            /// fill with white
            //rectangle(img,UL, UR_OFF, Scalar(255,255,255), CV_FILLED);
            /// left bar
            double length = 40;//(pv.cvRect_x2 - pv.cvRect_x1);
            //Point leftP = Point(pv.cvRect_x1, -ped_bel[i].left_side*length+ pv.cvRect_y1);
            Point leftP = Point(pv.cvRect_x1, ped_bel[i].left_side*length+ pv.cvRect_y1);
            rectangle(img,leftP, UC, Scalar(255,0,0), CV_FILLED);
            //Point leftP2 = Point(pv.cvRect_x2, -30+ pv.cvRect_y1);
            //rectangle(img,leftP2, UC, Scalar(255,255,255), CV_FILLED);


            /// right bar
            Point rightP = Point(pv.cvRect_x2,  ped_bel[i].right_side*length + pv.cvRect_y1);
            rectangle(img, rightP, UC, Scalar(255,0,0), CV_FILLED);





            /// lower panel
            if(ped_bel[i].decision==-1)
                rectangle(img,UL, UR_UP, Scalar(0,0,255), CV_FILLED); /// red
                else
                    rectangle(img,UL, UR_UP, Scalar(0,255,0), CV_FILLED); /// green



        }
    }

    putText(img, ss.str(), BL+Point(2,-2), FONT_HERSHEY_PLAIN, 0.8, cvScalar(0,255,255), 1, 8);
    //ss2<<setprecision(2)<<fixed<<pv.confidence*100.0;
    //putText(img, ss2.str(), BR+Point(-45,-2), FONT_HERSHEY_PLAIN, 0.8, cvScalar(0,255,255), 1, 8);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "MOMDP_Visualizer");
    ros::NodeHandle n;
    VisualizeMomdp * ic = new VisualizeMomdp(n);
    ros::spin();
    delete ic;
    return 0;
}
