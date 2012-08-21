#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <tf/tf.h>

using namespace std;


class SimpleGoal
{
    public:
        geometry_msgs::Pose car_position;
        ros::Publisher goal_pub;
        ros::Timer send_goal_timer;
        ros::Subscriber map_sub;

        void on_map(const nav_msgs::OccupancyGrid::ConstPtr og)
        {
            car_position = og->info.origin;
            cout<<car_position<<endl;
        }
        void on_send_goal_timer(const ros::TimerEvent &e)
        {
            double roll=0, pitch=0, yaw=0;
            tf::Quaternion q;
            tf::quaternionMsgToTF(car_position.orientation, q);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            geometry_msgs::PointStamped p;
            p.header.stamp = ros::Time::now();
            p.point.x = 50.0;
            p.point.y = 4.46;
            p.point.z = 0.34;

            ROS_INFO("sent goal: (%f,%f,%f)", p.point.x, p.point.y, p.point.z);

            goal_pub.publish(p);
        }
        SimpleGoal()
        {
            ros::NodeHandle n;
            goal_pub = n.advertise<geometry_msgs::PointStamped>("goal", 2);
            send_goal_timer = n.createTimer(ros::Duration(2.0), &SimpleGoal::on_send_goal_timer, this);
            map_sub = n.subscribe("local_map", 2, &SimpleGoal::on_map, this);
        }
};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_goal");

    SimpleGoal sg;

    ros::spin();

    return 0;
}
