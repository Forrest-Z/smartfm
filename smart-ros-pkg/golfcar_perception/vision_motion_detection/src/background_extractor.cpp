#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <std_msgs/Empty.h>
#include <boost/thread/mutex.hpp>

#include <dynamic_reconfigure/server.h>
#include <vision_motion_detection/BackgroundExtractorConfig.h>
using vision_motion_detection::BackgroundExtractorConfig;

#include <vision_motion_detection/BackgroundExtractor.h>


class BackgroundExtractorNode
{
private:
    ros::NodeHandle nhp_, nh_;
    image_transport::ImageTransport it_sub_, it_pub_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher bgnd_pub_;
    ros::Subscriber reset_sub_;

    BackgroundExtractor background_;
    boost::mutex mutex_; // protects background_

    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    void resetCallback(const std_msgs::Empty &);

    dynamic_reconfigure::Server<BackgroundExtractorConfig> server_;
    void configCallback(BackgroundExtractorConfig & config, uint32_t level);

public:
    BackgroundExtractorNode();
};

BackgroundExtractorNode::BackgroundExtractorNode() : nhp_("~"), it_pub_(nhp_), it_sub_(nh_)
{
    image_sub_ = it_sub_.subscribe("image", 10, &BackgroundExtractorNode::imageCallback, this);
    bgnd_pub_  = it_pub_.advertise("background", 10);

    double alpha;
    nhp_.param("alpha", alpha, 0.005);
    ROS_INFO("background alpha value: %f", alpha);
    background_.set_alpha(alpha);

    reset_sub_ = nhp_.subscribe("reset", 1, &BackgroundExtractorNode::resetCallback, this);

    server_.setCallback( boost::bind(&BackgroundExtractorNode::configCallback, this, _1, _2) );
}

void BackgroundExtractorNode::configCallback(BackgroundExtractorConfig & config, uint32_t level)
{
    background_.set_alpha(config.alpha);

    if( level==1 )
    {
        mutex_.lock();
        background_.reset();
        mutex_.unlock();
    }
}

void BackgroundExtractorNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    mutex_.lock();
    cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image, "bgr8");
    background_.add(cv_image->image);
    cv_bridge::CvImage backgroundImg;
    backgroundImg.header = cv_image->header;
    backgroundImg.encoding = "bgr8";
    backgroundImg.image = background_.getImg();
    mutex_.unlock();
    bgnd_pub_.publish(backgroundImg.toImageMsg());
}

void BackgroundExtractorNode::resetCallback(const std_msgs::Empty & dummy)
{
    mutex_.lock();
    background_.reset();
    mutex_.unlock();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "background_extractor");
    BackgroundExtractorNode node;
    ros::spin();
    return 0;
}
