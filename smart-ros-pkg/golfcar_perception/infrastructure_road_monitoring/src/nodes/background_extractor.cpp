#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <std_msgs/Empty.h>
#include <boost/thread/mutex.hpp>

#include <infrastructure_road_monitoring/BackgroundExtraction.h>

//TODO: add a reconfigure interface to change the background alpha value


class BackgroundExtractor
{
private:
    ros::NodeHandle nhp_, nh_;
    image_transport::ImageTransport it_sub_, it_pub_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher bgnd_pub_;
    ros::Subscriber reset_sub_;

    Background background_;
    boost::mutex mutex_; // protects background_

    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    void resetCallback(const std_msgs::Empty &);

public:
    BackgroundExtractor();
};

BackgroundExtractor::BackgroundExtractor() : nhp_("~"), it_pub_(nhp_), it_sub_(nh_)
{
    image_sub_ = it_sub_.subscribe("image", 10, &BackgroundExtractor::imageCallback, this);
    bgnd_pub_  = it_pub_.advertise("background", 10);

    double alpha;
    nhp_.param("alpha", alpha, 0.005);
    ROS_INFO("background alpha value: %f", alpha);
    background_.set_alpha(alpha);

    reset_sub_ = nhp_.subscribe("reset", 1, &BackgroundExtractor::resetCallback, this);
}

void BackgroundExtractor::imageCallback(const sensor_msgs::ImageConstPtr& image)
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

void BackgroundExtractor::resetCallback(const std_msgs::Empty & dummy)
{
    mutex_.lock();
    background_.reset();
    mutex_.unlock();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "background_extractor");
    BackgroundExtractor bgnd_ext;
    ros::spin();
    return 0;
}
