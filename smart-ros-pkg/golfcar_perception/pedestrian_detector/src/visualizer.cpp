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
#include "GraphUtils.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace message_filters;
using namespace sensing_on_road;
struct pedBelife_vis
{
    //double left_side, right_side;
    vector<double> goals;
    int decision;
    int id;
};

struct pedBelief_chart
{
    vector<float> BL;
    vector<float> BR;
    vector<float> TL;
    vector<float> TR;
};

struct speed_chart
{
    vector<float> cmd;
    vector<float> feedback;
};
class Visualizer
{
public:
    Visualizer(ros::NodeHandle &n);

private:
    ros::NodeHandle n_;

    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter image_sub_;
    ros::Subscriber ped_bel_sub_, cmd_vel_sub_;
    ros::Publisher cmd_vel_stamped_pub_;
    message_filters::Subscriber<sensing_on_road::pedestrian_vision_batch> people_roi_sub_;
    message_filters::Subscriber<geometry_msgs::TwistStamped> speed_cmd_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> speed_fb_sub_;
    bool started, ROI_text, verified_text, vision_rect, verified_rect, ROI_rect, publish_verified;
    vector<pedBelife_vis> ped_bel;
    //pedBelief_chart chart;
    vector< vector<float> > belief_chart;
    speed_chart sp_chart;
    int belief_chart_size, speedcmd_chart_size;
    double threshold_;
    void syncCallback(const sensor_msgs::ImageConstPtr image, const sensing_on_road::pedestrian_vision_batchConstPtr vision_roi);
    void speedCallback(const geometry_msgs::TwistStampedConstPtr cmd, const nav_msgs::OdometryConstPtr feedback);
    void drawIDandConfidence(cv::Mat& img, sensing_on_road::pedestrian_vision& pv);
    void pedBeliefCallback(ped_momdp_sarsop::peds_believes ped_bel);
    void cmdVellCallback(geometry_msgs::Twist cmdvel);
    void colorHist(cv::Mat src);
    void drawBeliefChart();
    void resetBeliefChart();
    void resetSpeedChart();
    void updateBelief();
    void extrapolateBelief();
    vector<Scalar> colors;

    ros::Time beliefLastUpdate;
    int trackingID;
    bool endBeliefChart;
};

enum color_t
{GREEN, PURPLE, BLUE, RED};

Visualizer::Visualizer(ros::NodeHandle &n) : n_(n), it_(n_)
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

    speed_cmd_sub_.subscribe(n, "cmd_vel_stamped", 20);
    speed_fb_sub_.subscribe(n, "encoder_odom", 20);
    cmd_vel_sub_ = n.subscribe("cmd_vel", 10, &Visualizer::cmdVellCallback, this);
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, pedestrian_vision_batch> imageSyncPolicy;
    Synchronizer<imageSyncPolicy> imageSync(imageSyncPolicy(20), image_sub_, people_roi_sub_);
    imageSync.registerCallback(boost::bind(&Visualizer::syncCallback,this, _1, _2));


    typedef sync_policies::ApproximateTime<geometry_msgs::TwistStamped, nav_msgs::Odometry> speedSyncPolicy;
    Synchronizer<speedSyncPolicy> speedSync(speedSyncPolicy(20), speed_cmd_sub_, speed_fb_sub_);
    speedSync.registerCallback(boost::bind(&Visualizer::speedCallback,this, _1, _2));
    cmd_vel_stamped_pub_ = n.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 10);

    ped_bel_sub_ = n.subscribe("peds_believes", 1, &Visualizer::pedBeliefCallback, this);
    belief_chart_size = 400;
    speedcmd_chart_size = 400;

    colors.push_back(Scalar(0, 255, 0));//0, 255)); //green
    colors.push_back(Scalar(255, 0, 246));//246, 0)); //purple
    colors.push_back(Scalar(30, 0, 255));//255, 0)); //blue
    colors.push_back(Scalar(255, 0, 0)); //red


    beliefLastUpdate = ros::Time::now();
    resetBeliefChart();
    resetSpeedChart();
    ros::spin();
}

void Visualizer::resetBeliefChart()
{
    cout<<"Reseting belief chart"<<endl;
    endBeliefChart = false;

    ped_bel.clear();
    int chart_size = 4;
    belief_chart.clear();
    belief_chart.resize(chart_size);

    for(size_t i=0; i<chart_size; i++)
    {
        belief_chart[i].resize(belief_chart_size);
        cout<< "Size of belief_chart: "<<belief_chart[i].size()<<endl;
    }

    trackingID = -1;
}

void Visualizer::resetSpeedChart()
{
    sp_chart.cmd.clear();
    sp_chart.feedback.clear();

    sp_chart.cmd.resize(speedcmd_chart_size);
    sp_chart.feedback.resize(speedcmd_chart_size);
}

void Visualizer::cmdVellCallback(geometry_msgs::Twist cmdvel)
{
    geometry_msgs::TwistStamped cmdvelts;
    cmdvelts.header.stamp = ros::Time::now();
    cmdvelts.header.frame_id = "base_link";
    cmdvelts.twist = cmdvel;
    cmd_vel_stamped_pub_.publish(cmdvelts);
}

void Visualizer::speedCallback(const geometry_msgs::TwistStampedConstPtr cmd, const nav_msgs::OdometryConstPtr feedback)
{
    cout <<"Speed sync!"<<endl;
    sp_chart.cmd.erase(sp_chart.cmd.begin());
    sp_chart.feedback.erase(sp_chart.feedback.begin());

    sp_chart.cmd.insert(sp_chart.cmd.end(), cmd->twist.linear.x);
    sp_chart.feedback.insert(sp_chart.feedback.end(), feedback->twist.twist.linear.x);

    ros::NodeHandle nh("~");
    string speedchart_bg;
    nh.param("speedchart_bg", speedchart_bg, string(""));
    IplImage *img = cvLoadImage(speedchart_bg.c_str());
    setCustomGraphColor(colors[RED]); //red
    drawFloatGraph(&sp_chart.cmd[0], sp_chart.cmd.size(), img, 0, 2.0, img->width, img->height, "Speed command");
    setCustomGraphColor(colors[GREEN]); //green
    //drawFloatGraph(&sp_chart.feedback[0], sp_chart.feedback.size(), img, 0, 2.5, img->width, img->height);
    IplImage *img2 = cvCreateImage(cvSize(img->width,img->height), 8, 3);
    cvResize(img, img2);
    cvNamedWindow("Speed Command", CV_WINDOW_AUTOSIZE);
    cvShowImage("Speed Command", img2);
    if((char)cvWaitKey(3) == 's');
    cvReleaseImage(&img);
    cvReleaseImage(&img2);

    extrapolateBelief();
    drawBeliefChart();
}

void Visualizer::syncCallback(const sensor_msgs::ImageConstPtr image, const sensing_on_road::pedestrian_vision_batchConstPtr vision_roi)
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
    Mat img2;
    resize(img, img2, Size(), 1.0, 1.0);
    imshow(ros::this_node::getName().c_str(), img2);
    //if(resetSpeedChart();
    //drawBeliefChart();
    char keypressed = cvWaitKey(3);
    switch (keypressed)
    {
        case 's':
            resetSpeedChart();
            break;
        case 'r':
            resetBeliefChart();
            break;
        case 'p':
            endBeliefChart = true;
            break;
    }

}

//void Visualizer::drawSpeedChart()
//{

//}

void Visualizer::drawBeliefChart()
{
    ros::NodeHandle nh("~");
    string beliefchart_bg;
    nh.param("beliefchart_bg", beliefchart_bg, string(""));
    IplImage *img = cvLoadImage(beliefchart_bg.c_str());
    for(size_t i=0; i<4; i++)
    {
        setCustomGraphColor(colors[i]); //purple
        drawFloatGraph(&belief_chart[i][0], belief_chart[i].size(), img, 0, 1, img->width, img->height, "Pedestrian Belief");
    }

    //draw legend
    Mat beliefImg(img);
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,0.8,1.0, 0,1,CV_AA);

    Point linePosOffset(-5, -5);
    Point lineLength (-10, 0);
    Point TR_pt(img->width-60, 15);
    Point textOffset(-80, 0);
    const string labeltxt[] = {"D", "C", "B", "A"};
    for(size_t i=0; i<4; i++, TR_pt += textOffset)
    {
        stringstream ss;

        ss<<setprecision(2)<<fixed<<labeltxt[i]<<": "<<belief_chart[i][belief_chart[i].size()-1];
        putText(beliefImg, ss.str(), TR_pt, FONT_HERSHEY_PLAIN, 0.8,cvScalar(0,0,0), 1, 8);
        Point linePos = TR_pt + linePosOffset;
        Scalar RGB = colors[3-i];
        Scalar BRG = Scalar(RGB[2], RGB[1], RGB[0]);
        line(beliefImg, linePos, linePos + lineLength, BRG, 2);
    }

    //putText(beliefImg, "G3", TR_pt-textOffset, FONT_HERSHEY_PLAIN, 0.8,cvScalar(0,0,0), 1, 8);
    //cvPutText(img, "G4", TR_pt, &font, CV_RGB(0,0,0));
    //cvLine(img, TR_pt+legendPosOffset, TR_pt+legendPosOffset+legendLength, CV_RGB(colors[0][0], colors[0][1], colors[0][2]), 1, CV_AA);
    //TR_pt = cvPoint(toprightx-=50, toprighty);
    /*cvPutText(img, "G3", TR_pt, &font, CV_RGB(0,0,0));
	cvPutText(img, "G2", cvPoint(toprightx-=50, toprighty), &font, CV_RGB(0,0,0));
	cvPutText(img, "G1", cvPoint(toprightx-=50, toprighty), &font, CV_RGB(0,0,0));

	cvNamedWindow("chart", CV_WINDOW_AUTOSIZE);*/
    imshow("chart", beliefImg);
    cvWaitKey(3);

    cvReleaseImage(&img);
}

void Visualizer::extrapolateBelief()
{
    //this function will add the latest belief into the vector to keep in sync with speed command chart
    cout<< "Duration since last belief update "<<(ros::Time::now()-beliefLastUpdate)<<endl;

    belief_chart[0].erase(belief_chart[0].begin());
    belief_chart[1].erase(belief_chart[1].begin());
    belief_chart[2].erase(belief_chart[2].begin());
    belief_chart[3].erase(belief_chart[3].begin());

    if(ped_bel.size()==0)
    {
        belief_chart[0].insert(belief_chart[0].end(), 0);
        belief_chart[1].insert(belief_chart[1].end(), 0);
        belief_chart[2].insert(belief_chart[2].end(), 0);
        belief_chart[3].insert(belief_chart[3].end(), 0);
    }
    else
    {
        if(ped_bel[0].id == trackingID && !endBeliefChart)
        {
            belief_chart[0].insert(belief_chart[0].end(), ped_bel[0].goals[0]);
            belief_chart[1].insert(belief_chart[1].end(), ped_bel[0].goals[1]);
            belief_chart[2].insert(belief_chart[2].end(), ped_bel[0].goals[2]);
            belief_chart[3].insert(belief_chart[3].end(), ped_bel[0].goals[3]);
        }
        else
        {
            belief_chart[0].insert(belief_chart[0].end(), 0);
            belief_chart[1].insert(belief_chart[1].end(), 0);
            belief_chart[2].insert(belief_chart[2].end(), 0);
            belief_chart[3].insert(belief_chart[3].end(), 0);
        }
    }

}

void Visualizer::pedBeliefCallback(ped_momdp_sarsop::peds_believes ped_bel_cb)
{

    if(trackingID == -1) trackingID = ped_bel_cb.believes[0].ped_id;
    ped_bel.resize(ped_bel_cb.believes.size());
    for(size_t i=0; i<ped_bel_cb.believes.size(); i++)
    {
        if(ped_bel_cb.believes[i].ped_id == trackingID) beliefLastUpdate = ros::Time::now();
        //double left =  ped_bel_cb.believes[i].belief_value[0] + ped_bel_cb.believes[i].belief_value[3];
        //double right =  ped_bel_cb.believes[i].belief_value[1] + ped_bel_cb.believes[i].belief_value[2];
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
        ped_bel[i].goals.clear();

        //move the last goals value to as the first one: workaround specific for crossing case
        //for(size_t j=0; j< ped_bel_cb.believes[i].belief_value.size(); j++)
        //           ped_bel[i].goals.push_back(ped_bel_cb.believes[i].belief_value[j]);
        ped_bel[i].goals.push_back(max(ped_bel_cb.believes[i].belief_value[3], ped_bel_cb.believes[i].belief_value[0]));
        //ped_bel[i].goals.push_back();
        ped_bel[i].goals.push_back(max(ped_bel_cb.believes[i].belief_value[1], ped_bel_cb.believes[i].belief_value[2]));
        //ped_bel[i].goals.push_back();

        ped_bel[i].decision = decision;
        //ped_bel[i].goals = left;
        //ped_bel[i].right_side = right;
        ped_bel[i].id = ped_bel_cb.believes[i].ped_id;
    }

}

void Visualizer::drawIDandConfidence(Mat& img, sensing_on_road::pedestrian_vision& pv)
{
    std::stringstream ss,ss2;
    float offset = 10.0;
    ss<<pv.object_label;




    //if(ped_bel[i].decision==-1)
    //rectangle(img,UL, UR_UP, Scalar(0,0,255), CV_FILLED); /// red
    //else

    /// belief bar
    double bar_width=10;
    double bar_buff=5;
    double min_height=5;

    /// belief panel
    double gap=5;



    for(size_t i=0; i<ped_bel.size(); i++)
    {
        /// Set visualizer geometry

        double Bbox_width =  gap + (bar_width + gap)*ped_bel[0].goals.size();
        double Bbox_height = 30;

        double D_panel_width = Bbox_width;//pv.cvRect_x2 - pv.cvRect_x1;
        double D_panel_height = 10;//pv.cvRect_y1 - pv.cvRect_y2;
        ///flipped because of image coord

        /// decision panel
        Point D_TopLeft = Point(pv.cvRect_x1, pv.cvRect_y1);
        Point D_BotRight = Point(pv.cvRect_x1+D_panel_width,pv.cvRect_y1+D_panel_height);

        /// remove the annoying belief bar that still appearing even when the pedestrian rect is disappeared
        if(pv.cvRect_x1 == 0 && pv.cvRect_y1 ==0) continue;

        Point B_TopLeft = Point(pv.cvRect_x1, pv.cvRect_y1+D_panel_height + gap);
        Point B_TopRight = Point(pv.cvRect_x1+Bbox_width,
                                 pv.cvRect_y1+D_panel_height + gap);

        Point B_BotLeft = Point(pv.cvRect_x1, pv.cvRect_y1+D_panel_height
                                +Bbox_height + gap);
        Point B_BotRight = Point(pv.cvRect_x1+Bbox_width,
                                 pv.cvRect_y1+D_panel_height+Bbox_height + gap);


        if(ped_bel[i].id == pv.object_label)
        {
            // if((ped_bel[i].id==61) || (ped_bel[i].id==62)) /// ISER 2ped  bag file fixed pedestrians
            //   return;

            //std::cout<<ped_bel[i].id<<" "<<ped_bel[i].left_side<<" "<<ped_bel[i].right_side << " decision " << ped_bel[i].decision
            //      <<std::endl;

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

            for(size_t j = 0; j<ped_bel[i].goals.size(); j++)
            {
                assert(ped_bel[i].goals.size()==2);
                /// left bar
                double lvalue = ped_bel[i].goals[j]*Bbox_height;
                cout<<"lvalue "<<j<<" ="<<lvalue<<' ';
                if(lvalue < min_height)
                    lvalue = min_height;
                Point bar_TopRight = Point(B_TopLeft.x+(bar_width+bar_buff)*(j+1),
                                           B_BotLeft.y- lvalue);
                /// fill
                Scalar RGB = colors[BLUE];
                Scalar BRG = Scalar(RGB[2], RGB[1], RGB[0]);
                rectangle(img, Point(bar_buff + B_BotLeft.x + (bar_width+bar_buff)*j, B_BotLeft.y),
                          bar_TopRight, BRG,CV_FILLED);
                /// outline
                rectangle(img, Point(bar_buff + B_BotLeft.x + (bar_width+bar_buff)*j, B_BotLeft.y),
                          bar_TopRight, Scalar(0, 0, 0),1);
            }
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

void Visualizer::colorHist(cv::Mat src)
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
            //cout << intensity << ' ';
            /*cvRectangle( histImg, Point(h*scale, s*scale),
                         Point( (h+1)*scale - 1, (s+1)*scale - 1),
                         Scalar::all(intensity),
                         CV_FILLED );*/
        }

    //cout<<endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MOMDP_Visualizer");
    ros::NodeHandle n;
    Visualizer * ic = new Visualizer(n);
    return 0;
}
