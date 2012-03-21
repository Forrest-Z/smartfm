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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace message_filters;
using namespace sensing_on_road;
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
    image_transport::SubscriberFilter image_sub_;
    ros::Subscriber ped_bel_sub_;
    message_filters::Subscriber<sensing_on_road::pedestrian_vision_batch> people_roi_sub_;
    bool started, ROI_text, verified_text, vision_rect, verified_rect, ROI_rect, publish_verified;
    vector<pedBelife_vis> ped_bel;
    double threshold_;
    void syncCallback(const sensor_msgs::ImageConstPtr image, const sensing_on_road::pedestrian_vision_batchConstPtr vision_roi);
    void drawIDandConfidence(cv::Mat& img, sensing_on_road::pedestrian_vision& pv);
    void pedBeliefCallback(ped_momdp_sarsop::peds_believes ped_bel);
    void colorHist(cv::Mat src);
};

VisualizeMomdp::VisualizeMomdp(ros::NodeHandle &n) : n_(n), it_(n_)
{
    ros::NodeHandle nh("~");
    nh.param("ROI_text", ROI_text, true);
    nh.param("ROI_rect", ROI_rect, false);
    //nh.param("Publish_verified", publish_verified, true);
    nh.param("threshold", threshold_, 0.0);
    // get image from the USB cam
    image_sub_.subscribe(it_, "/usb_cam/image_raw", 20);

    // start processign only after the first image is present
    started = false;

    // from laser (green rect)
    people_roi_sub_.subscribe(n, "veri_pd_vision", 20);

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, pedestrian_vision_batch> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), image_sub_, people_roi_sub_);
    sync.registerCallback(boost::bind(&VisualizeMomdp::syncCallback,this, _1, _2));

    ped_bel_sub_ = n.subscribe("peds_believes", 1, &VisualizeMomdp::pedBeliefCallback, this);

    ros::spin();
}

void VisualizeMomdp::syncCallback(const sensor_msgs::ImageConstPtr image, const sensing_on_road::pedestrian_vision_batchConstPtr vision_roi)
{
    ROS_INFO("imageCallback with roi rect %d", ROI_rect);
    Mat img;
    Cv_helper::sensormsgsToCv(image, img);

    /*SURF surf;
    Mat mask;
    Mat bw_img;
    vector<KeyPoint>  keypoints;
    cv::cvtColor(img, bw_img, CV_BGRA2GRAY);
    assert(bw_img.type()==CV_8UC1);
    surf(bw_img,mask,keypoints);
    drawKeypoints(img, keypoints, img, Scalar(0,255,0));*/

    /// roi_rects_ : laser based ( blue )
    /// detect_rects_ : vision based detection ( green )
    /// verified_rects_ : final confirmation  (red )
    for(unsigned int i=0; i<vision_roi->pd_vector.size(); i++)
    {
        Point UL = Point(vision_roi->pd_vector[i].cvRect_x1, vision_roi->pd_vector[i].cvRect_y1);
        Point BR = Point(vision_roi->pd_vector[i].cvRect_x2, vision_roi->pd_vector[i].cvRect_y2);
        sensing_on_road::pedestrian_vision temp_rect = vision_roi->pd_vector[i];
        //if(ROI_rect)
        {
            ROS_INFO("Confidence: %lf", vision_roi->pd_vector[i].confidence);
            if(threshold_ > 0)
            {
                if(vision_roi->pd_vector[i].confidence*100 > threshold_)
                    rectangle(img,UL, BR, Scalar(255,0,0), 1);
            }
            else rectangle(img,UL, BR, Scalar(255,0,0), 1);

        }
        if(ROI_text)
        {
            if(threshold_ > 0)
            {
                if(vision_roi->pd_vector[i].confidence*100 > threshold_)
                    drawIDandConfidence(img, temp_rect);
            }
            else drawIDandConfidence(img, temp_rect);
        }
    }


    started = true;
    imshow(ros::this_node::getName().c_str(), img);
    cvWaitKey(3);
}

void VisualizeMomdp::pedBeliefCallback(ped_momdp_sarsop::peds_believes ped_bel_cb)
{
    ped_bel.resize(ped_bel_cb.believes.size());
    for(size_t i=0; i<ped_bel_cb.believes.size(); i++)
    {
        double left =  ped_bel_cb.believes[i].belief_value[0] + ped_bel_cb.believes[i].belief_value[3];
        double right =  ped_bel_cb.believes[i].belief_value[1] + ped_bel_cb.believes[i].belief_value[2];
        int decision=-2;

        if(ped_bel_cb.believes[i].action==1)
            decision=1;
        else if(ped_bel_cb.believes[i].action==2)
            decision=-1;
        else if(ped_bel_cb.believes[i].action==0)
        {
            //if(ped_bel_cb.robotv>0.1)
            //decision=1;
            //else if( ped_bel_cb.robotv < 0.1)
            //decision=-1;

            decision =0;
        }
        else
        {
            std::cout << "Strange action " << ped_bel_cb.believes[i].action << std::endl;
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

    /// Set visualizer geometry
    double D_panel_width = 35;//pv.cvRect_x2 - pv.cvRect_x1;
    double D_panel_height = 10;//pv.cvRect_y1 - pv.cvRect_y2;
    ///flipped because of image coord

    /// decision panel
    Point D_TopLeft = Point(pv.cvRect_x1, pv.cvRect_y1);
    Point D_BotRight =
            Point(pv.cvRect_x1+D_panel_width,pv.cvRect_y1+D_panel_height);

    //if(ped_bel[i].decision==-1)
    //rectangle(img,UL, UR_UP, Scalar(0,0,255), CV_FILLED); /// red
    //else


    /// belief panel
    double gap=5;
    double Bbox_width = 35;//pv.cvRect_x2 - pv.cvRect_x1;
    double Bbox_height = 30;

    Point B_TopLeft = Point(pv.cvRect_x1, pv.cvRect_y1+D_panel_height + gap);
    Point B_TopRight = Point(pv.cvRect_x1+Bbox_width,
                             pv.cvRect_y1+D_panel_height + gap);

    Point B_BotLeft = Point(pv.cvRect_x1, pv.cvRect_y1+D_panel_height
                            +Bbox_height + gap);
    Point B_BotRight = Point(pv.cvRect_x1+Bbox_width,
                             pv.cvRect_y1+D_panel_height+Bbox_height + gap);

    //rectangle(img,B_BotRight, B_TopLeft, Scalar(0,0,0),1);

    /// belief bar
    double bar_width=10;
    double bar_buff=5;
    double min_height=5;


    ///// first row
    ////Point UL =
    //Point UR = Point(pv.cvRect_x2, pv.cvRect_y1); /// top left
    //Point UR_UP = Point(pv.cvRect_x2, pv.cvRect_y1-offset); /// top left
    //Point UC = Point((pv.cvRect_x2+pv.cvRect_x1)/2, pv.cvRect_y1); /// top mid
    //Point UP_C_OFF = Point((pv.cvRect_x1+pv.cvRect_x2)/2,    pv.cvRect_y1+offset); /// top thickness
    //Point UL_OFF = Point(pv.cvRect_x1, pv.cvRect_y1+offset);
    ///topleft thickness
    ////Point UR_OFF = Point(pv.cvRect_x2, pv.cvRect_y1+offset);
    ///topright thickness

    ///// second row
    //Point UR_2OFF = Point(pv.cvRect_x2, pv.cvRect_y1+(2*offset));
    /// 2nd row mid thickness
    //Point UR_OFF = Point(pv.cvRect_x2, pv.cvRect_y1+offset);
    //Point BR = Point(pv.cvRect_x2, pv.cvRect_y2); /// bot left
    //Point BL = Point(pv.cvRect_x1, pv.cvRect_y2); /// bot right

    ////Point LeftMid = Point((pv.cvRect_x1+UC.x)/2, pv.cvRect_y1);
    ////Point RightMid = Point((pv.cvRect_x2+UC.x)/2, pv.cvRect_y1);
    //Point LeftMid = Point((pv.cvRect_x1+10, pv.cvRect_y2);
    //Point RightMid = Point((pv.cvRect_x2-10, pv.cvRect_y2);

    for(size_t i=0; i<ped_bel.size(); i++)
    {

        if(ped_bel[i].id == pv.object_label)
        {
            // if((ped_bel[i].id==61) || (ped_bel[i].id==62)) /// ISER 2ped  bag file fixed pedestrians
            //   return;

            std::cout<<ped_bel[i].id<<" "<<ped_bel[i].left_side<<" "<<ped_bel[i].right_side << " decision " << ped_bel[i].decision
                    <<std::endl;

            if (ped_bel[i].decision==1)
                rectangle(img,D_TopLeft, D_BotRight, Scalar(0,255,0), CV_FILLED); /// green
            else
                rectangle(img,D_TopLeft, D_BotRight, Scalar(0,0,255), CV_FILLED);
            /// red and blue are flipped WTF ????


            //cout << " x " << pv.cvRect_x1 << " y " << pv.cvRect_y1;
            //cout << " x " << pv.cvRect_x2 << " y " << pv.cvRect_y2 << endl;;

            /// Gradient Coding

            //int left_gradient = 255-(255.0*ped_bel[i].left_side);
            //int right_gradient = 255-(255.0*ped_bel[i].right_side);

            //rectangle(img,UL, UP_C_OFF,    Scalar(255,left_gradient,left_gradient), CV_FILLED);
            //rectangle(img,UL, UP_C_OFF,    Scalar(255,left_gradient,left_gradient), CV_FILLED);
            //rectangle(img,UC, UR_OFF,    Scalar(255,right_gradient,right_gradient), CV_FILLED);

            /// size encoding
            //rectangle(img,UL, UP_C_OFF, Scalar(255,255,255), CV_FILLED);
            ///// fill with white
            //rectangle(img,UL, UR_OFF, Scalar(255,255,255), CV_FILLED);

            /// left bar
            //double length = 40;//(pv.cvRect_x2 - pv.cvRect_x1);
            ////Point leftP = Point(pv.cvRect_x1,    -ped_bel[i].left_side*length+ pv.cvRect_y1);
            //Point leftP = Point(pv.cvRect_x1,    ped_bel[i].left_side*length+ pv.cvRect_y2);
            //rectangle(img,leftP, LeftMid, Scalar(255,0,0), CV_FILLED);
            ////Point leftP2 = Point(pv.cvRect_x2, -30+ pv.cvRect_y1);
            ////rectangle(img,leftP2, UC, Scalar(255,255,255), CV_FILLED);


            ///// right bar
            //Point rightP = Point(pv.cvRect_x2,    ped_bel[i].right_side*length + pv.cvRect_y2);
            //rectangle(img, rightP, RightMid, Scalar(255,0,0), CV_FILLED);


            /// clean up background
            rectangle(img,B_TopLeft, B_BotRight, Scalar(255,255,255,150),CV_FILLED);

            /// left bar
            double lvalue = ped_bel[i].left_side*Bbox_height;
            if(lvalue < min_height)
                lvalue = min_height;
            Point lbar_TopRight = Point(B_TopLeft.x+bar_width+bar_buff,
                                        B_BotLeft.y- lvalue);
            /// fill
            rectangle(img, Point(B_BotLeft.x+bar_buff, B_BotLeft.y),
                      lbar_TopRight, Scalar(255,0,0,0.7),CV_FILLED);
            /// outline
            rectangle(img,Point(B_BotLeft.x+bar_buff, B_BotLeft.y),
                      lbar_TopRight, Scalar(0,0,0),1);


            /// right bar
            double rvalue = ped_bel[i].right_side*Bbox_height;
            if(rvalue < min_height)
                rvalue = min_height;
            Point rbar_TopLeft = Point(B_TopRight.x-bar_width-bar_buff,
                                       B_BotRight.y- rvalue);
            /// fill
            rectangle(img,Point(B_BotRight.x-bar_buff, B_BotRight.y),
                      rbar_TopLeft, Scalar(255,0,0,0.7),CV_FILLED);
            /// outline
            rectangle(img,Point(B_BotRight.x-bar_buff, B_BotRight.y),
                      rbar_TopLeft, Scalar(0,0,0),1);

            //rectangle(img,UL, UR_UP, Scalar(0,255,255), CV_FILLED);
            ///yellow ( cruise )

        }
    }

    Point UL = Point(pv.cvRect_x1, pv.cvRect_y1);
    Point BR = Point(pv.cvRect_x2, pv.cvRect_y2);
    Point BL = Point(pv.cvRect_x1, pv.cvRect_y2); /// bot left
    putText(img, ss.str(), BL+Point(2,-2), FONT_HERSHEY_PLAIN,
            0.8,cvScalar(0,255,255), 1, 8);
    cout<<ss.str();


    colorHist(Mat(img, Rect(UL,BR)));
    //ss2<<setprecision(2)<<fixed<<pv.confidence*100.0;
    //putText(img, ss2.str(), BR+Point(-45,-2), FONT_HERSHEY_PLAIN,0.8,    cvScalar(0,255,255), 1, 8);
}

void VisualizeMomdp::colorHist(cv::Mat src)
{
    Mat hsv;
    cvtColor(src, hsv, CV_BGR2HSV);

    // let's quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 10, sbins = 1;
    int histSize[] = {hbins, sbins};
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };
    MatND hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};

    calcHist( &hsv, 1, channels, Mat(), // do not use mask
              hist, 2, histSize, ranges,
              true, // the histogram is uniform
              false );
    double maxVal=0;
    minMaxLoc(hist, 0, &maxVal, 0, 0);

    int scale = 10;
    Mat histImg = Mat::zeros(sbins*scale, hbins*10, CV_8UC3);

    for( int h = 0; h < hbins; h++ )
        for( int s = 0; s < sbins; s++ )
        {
            float binVal = hist.at<float>(h, s);
            int intensity = cvRound(binVal*255/maxVal);
            cout << intensity << ' ';
            /*cvRectangle( histImg, Point(h*scale, s*scale),
                         Point( (h+1)*scale - 1, (s+1)*scale - 1),
                         Scalar::all(intensity),
                         CV_FILLED );*/
        }

    cout<<endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MOMDP_Visualizer");
    ros::NodeHandle n;
    VisualizeMomdp * ic = new VisualizeMomdp(n);
    return 0;
}
