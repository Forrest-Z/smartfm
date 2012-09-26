#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <network_delay_experiment/delay.h>

ros::Publisher *delay_pub;

void ImageCallBack( const sensor_msgs::ImageConstPtr& image_msg,
                    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    ros::WallTime walltime = ros::WallTime::now();
    ros::Time time_now = ros::Time(walltime.sec, walltime.nsec);
    network_delay_experiment::delay delay;
    delay.seq = info_msg->header.seq;
    delay.time_now = time_now;
    delay.scan_time = info_msg->header.stamp;
    delay.nsec_delay = (time_now - info_msg->header.stamp).toNSec();
    delay_pub->publish(delay);  
}

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "image_sub");
    ros::NodeHandle n;
    image_transport::CameraSubscriber cam_sub_;
    ros::Publisher pub;
    
    image_transport::ImageTransport it_(n);
    cam_sub_ = it_.subscribeCamera("/camera_front/image_raw", 1, &ImageCallBack);
    
    pub = n.advertise<network_delay_experiment::delay>("vision_delay", 10);
    delay_pub = &pub;
    ros::spin();
}
