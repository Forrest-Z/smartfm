#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/highgui/highgui.hpp>

#include <vision_opticalflow/Feature.h> //for message type Feature.msg

#include <vision_opticalflow/FeatureExtractor.h>

class FeatureExtractorNode
{
public:
    FeatureExtractorNode();
    
private:
    FeatureExtractor feature_;

    ros::NodeHandle nhp_, nh_;
    image_transport::ImageTransport it_sub_;
    image_transport::SubscriberFilter image_sub_, bgnd_sub_;
    ros::Timer timer_;

    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image
    > MySyncPolicy;
    
    message_filters::Synchronizer<MySyncPolicy> synchronizer;

    ros::Publisher feature_pub_;

    int skip_frames_, skip_frames_count_;
    double period_, last_time_;

    boost::mutex mutex_; // protects background_

    void imageCallback(const sensor_msgs::Image::ConstPtr & image,
    const sensor_msgs::Image::ConstPtr & background);
    void timerCallback(const ros::TimerEvent &);
};

FeatureExtractorNode::FeatureExtractorNode()
: nhp_("~"),
  it_sub_(nh_),
  image_sub_(it_sub_, "camera", 20),
  bgnd_sub_(it_sub_, "background", 20),
  synchronizer(MySyncPolicy(20), image_sub_, bgnd_sub_),
  skip_frames_(0),
  skip_frames_count_(0),
  period_(0.0),
  last_time_(0.0)
{
    synchronizer.registerCallback( boost::bind(&FeatureExtractorNode::imageCallback, this, _1, _2) );
    
    feature_pub_  = nhp_.advertise<vision_opticalflow::Feature>("feature", 10);

    timer_ = nh_.createTimer(ros::Duration(1), boost::bind(&FeatureExtractorNode::timerCallback, this, _1) );
}

void FeatureExtractorNode::timerCallback(const ros::TimerEvent & dummy)
{
    //do nothing
}

void FeatureExtractorNode::imageCallback(const sensor_msgs::ImageConstPtr& image,
        const sensor_msgs::Image::ConstPtr & background)
{

    double t_now = image->header.stamp.toSec();
    if( last_time_==0.0 ||
            (++skip_frames_count_ > skip_frames_ && t_now>last_time_+period_) )
    {
        //ROS_INFO("processing");
        skip_frames_count_ = 0;
        last_time_ = t_now;
    }
    else
    {
        //ROS_INFO("skipping");
        return;
    }

    cv_bridge::CvImageConstPtr cvImgFrame = cv_bridge::toCvShare(image, "bgr8");
    cv_bridge::CvImageConstPtr cvImgBgnd = cv_bridge::toCvShare(background, "bgr8");

    vision_opticalflow::Feature feature_msg = feature_.extract(cvImgFrame->image, cvImgBgnd->image, t_now);
    feature_msg.header = image->header;

    feature_pub_.publish(feature_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_extractor");
    FeatureExtractorNode node;
    ros::spin();
    return 0;
}