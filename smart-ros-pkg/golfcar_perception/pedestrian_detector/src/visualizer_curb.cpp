#include <iomanip>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
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

class VisualizeCurb
{
public:
    VisualizeCurb();

private:
    ros::NodeHandle n_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    ros::Publisher pc_pub_;
    sensor_msgs::PointCloud curb_points;
    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    void curbCallback(const sensor_msgs::PointCloudConstPtr pc);
    tf::TransformListener *listener_;

    tf::MessageFilter<sensor_msgs::PointCloud>* curb_pc_filter_;
    message_filters::Subscriber<sensor_msgs::PointCloud> pc_sub_;
};

VisualizeCurb::VisualizeCurb() : it_(n_)
{
    ros::NodeHandle nh("~");

    // get image from the USB cam
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &VisualizeCurb::imageCallback, this);

    // laser's curb points filtered by available transform
    pc_sub_.subscribe(n_, "hybrid_pt", 10);
    curb_pc_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(pc_sub_, *listener_, "map", 10);
    curb_pc_filter_ -> registerCallback(boost::bind(&VisualizeCurb::curbCallback, this, _1));

    pc_pub_ = n_.advertise<sensor_msgs::PointCloud>("curb_points", 1);

    ros::spin();
}

void VisualizeCurb::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    Mat img;
    Cv_helper::sensormsgsToCv(image, img);
    imshow("curb_img", img);
    waitKey(2);
}

void VisualizeCurb::curbCallback(const sensor_msgs::PointCloudConstPtr pc)
{
    sensor_msgs::PointCloud pcin, pcout;
    pcin = *pc;
    try{
        listener_->transformPointCloud("map", pcin, pcout);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform point: %s", ex.what());
        return;
    }
    curb_points.header = pcout.header;
    if(curb_points.points.size()==100) curb_points.points.erase(curb_points.points.begin());
    if(pcout.points.size()>0) curb_points.points.push_back(pcout.points[0]);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "Curb_Visualizer");
    VisualizeCurb *ic = new VisualizeCurb::VisualizeCurb();
}
