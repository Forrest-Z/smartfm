#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>

#include "BackgroundExtraction.h"


class BackgroundExtractor
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher bgnd_pub_;

    Background background;

    void imageCallback(const sensor_msgs::ImageConstPtr& image);

public:
    BackgroundExtractor();
};

BackgroundExtractor::BackgroundExtractor() : it_(nh_)
{
    image_sub_ = it_.subscribe("camera", 10, &BackgroundExtractor::imageCallback, this);
    bgnd_pub_  = it_.advertise("background", 10);

    ros::NodeHandle nh("~");
    nh.param("alpha", background.alpha, 0.005);
    ROS_INFO("background alpha value: %f", background.alpha);
}

void BackgroundExtractor::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image, "bgr8");
    background.add(cv_image->image);
    cv_bridge::CvImage backgroundImg;
    backgroundImg.header = cv_image->header;
    backgroundImg.encoding = "bgr8";
    backgroundImg.image = background.getImg();
    bgnd_pub_.publish(backgroundImg.toImageMsg());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "background_extractor");
    BackgroundExtractor bgnd_ext;
    ros::spin();
    return 0;
}
