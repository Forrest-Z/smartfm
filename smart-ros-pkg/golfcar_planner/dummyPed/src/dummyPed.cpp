
#include "dummyPed.h"



dummy_ped::dummy_ped()
{
    ros::NodeHandle nh;
    psgPedpub_ = nh.advertise<sensing_on_road::pedestrian_laser_batch>("ped_map_laser_batch", 2);

    message_filters::Subscriber<nav_msgs::Odometry> ped1_sub(nh, "/robot_1/base_pose_ground_truth", 1);
    message_filters::Subscriber<nav_msgs::Odometry> ped2_sub(nh, "/robot_2/base_pose_ground_truth", 1);
    message_filters::Subscriber<nav_msgs::Odometry> ped3_sub(nh, "/robot_3/base_pose_ground_truth", 1);
    message_filters::TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry> sync(ped1_sub, ped2_sub, ped3_sub, 10);
    sync.registerCallback(boost::bind(&dummy_ped::psgPedPoseCallback,this, _1, _2, _3));

    ros::spin();
}


void dummy_ped::psgPedPoseCallback(const nav_msgs::OdometryConstPtr odo1, const nav_msgs::OdometryConstPtr odo2, const nav_msgs::OdometryConstPtr odo3)
{
    sensing_on_road::pedestrian_laser_batch psglaser_batch;
    sensing_on_road::pedestrian_laser psglaser;

    psglaser.pedestrian_laser.point.x = odo1->pose.pose.position.x;
    psglaser.pedestrian_laser.point.y = odo1->pose.pose.position.y;
    psglaser.object_label = 1;
    psglaser_batch.pedestrian_laser_features.push_back(psglaser);

    psglaser.pedestrian_laser.point.x = odo2->pose.pose.position.x;
    psglaser.pedestrian_laser.point.y = odo2->pose.pose.position.y;
    psglaser.object_label = 2;
    psglaser_batch.pedestrian_laser_features.push_back(psglaser);

    psglaser.pedestrian_laser.point.x = odo3->pose.pose.position.x;
    psglaser.pedestrian_laser.point.y = odo3->pose.pose.position.y;
    psglaser.object_label = 3;
    psglaser_batch.pedestrian_laser_features.push_back(psglaser);

    psgPedpub_.publish(psglaser_batch);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_ped");

    dummy_ped dummy_ped_node;


}

