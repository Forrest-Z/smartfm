#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <vector>
#include <tf/transform_listener.h>
#include <cmath>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include "sensing_on_road/pedestrian_laser.h"
#include "sensing_on_road/pedestrian_laser_batch.h"
#include "sensing_on_road/pedestrian_vision.h"
#include "sensing_on_road/pedestrian_vision_batch.h"


namespace camera_projector
{

class camera_calib
{
public:
    double fc[2];   //focal_length
    double cc[2];   //principal_point
    double alpha_c; //skew_coeff
    double kc[5];   //distortions

    float raw_image_width;
    float raw_image_height;
    float scaled_image_width;
    float scaled_image_height;

    camera_calib()
    {
        fc[0]=471.21783;
        fc[1]=476.11993;
        cc[0]=322.35359;
        cc[1]=183.44632;
        alpha_c=0;

        kc[0]=0.00083;
        kc[1]=-0.03521;
        kc[2]=0.00135;
        kc[3]=0.00048;
        kc[4]=0;

        raw_image_width=640;
        raw_image_height=360;
    }

    ~camera_calib() {}
};


class camera_project
{
public:
    camera_project();

private:
    std::string ldmrs_single_id_, camera_frame_id_;
    camera_calib webcam_;
    ros::Subscriber pd_laser_batch_sub_;
    void project_to_image(const sensing_on_road::pedestrian_laser_batch &pd_laser_para);
    void projection (const geometry_msgs::Point32 &temp3Dpara, sensing_on_road::pedestrian_vision &tempprpara);
    ros::Publisher pd_vision_pub_;
    sensing_on_road::pedestrian_vision_batch pd_vision_batch_;

    void pcl_callback(const sensor_msgs::PointCloud::ConstPtr& pcl_in);
    message_filters::Subscriber<sensor_msgs::PointCloud> pd_pcl_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<sensor_msgs::PointCloud> * tf_filter_;
};

} //namespace
