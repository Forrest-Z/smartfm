#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <network_delay_experiment/delay.h>

ros::Publisher *delay_pub;
image_transport::Publisher *img_pub;

void ImageCallBack( const sensor_msgs::ImageConstPtr& image_msg)
{
    ros::WallTime walltime = ros::WallTime::now();
    ros::Time time_now = ros::Time(walltime.sec, walltime.nsec);
    network_delay_experiment::delay delay;
    delay.seq = image_msg->header.seq;
    delay.time_now = time_now;
    delay.scan_time = image_msg->header.stamp;
    delay.nsec_delay = (time_now - image_msg->header.stamp).toNSec();
    delay_pub->publish(delay);  
    sensor_msgs::Image image_in = * image_msg;
    image_in.header.stamp = ros::Time::now();
    //image_in.width = 6;
    //image_in.step = 3 * image_in.width;
    //image_in.data.resize(image_in.step * image_in.height);
  //  ROS_INFO("size is %d",image_in.data.size());
	 img_pub->publish(image_in);
	 
}

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "image_sub");
    ros::NodeHandle n;
    
    ros::Publisher pub;
    
    image_transport::ImageTransport it_(n);
    image_transport::Subscriber cam_sub_ = it_.subscribe("/golfcart/camera_front2/image_raw", 1, ImageCallBack);
    image_transport::Publisher image_pub;
    image_pub = it_.advertise("/golfcart/camera_front/image_raw", 1);
    
    
    pub = n.advertise<network_delay_experiment::delay>("vision_delay", 10);
    delay_pub = &pub;
    img_pub = &image_pub;
    ros::spin();
}
