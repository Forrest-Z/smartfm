#include <string>
#include <iostream>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

#include <nav_msgs/Odometry.h>
#include <lowlevel/SimpleOdo.h>

using namespace std;


class SimpleOdoNode
{
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;

public:
    SimpleOdoNode(const string & topic_name)
    {
        pub = n.advertise<lowlevel::SimpleOdo>(topic_name+"_simple", 0);
        sub = n.subscribe(topic_name, 1, &SimpleOdoNode::callback, this);
    }

    void callback(const nav_msgs::Odometry &odomsg)
    {
        lowlevel::SimpleOdo m;
        m.stamp = odomsg.header.stamp;
        m.x = odomsg.pose.pose.position.x;
        m.y = odomsg.pose.pose.position.y;

        // Convert quaternion to RPY.
        btQuaternion q;
        double roll, pitch, yaw;
        tf::quaternionMsgToTF(odomsg.pose.pose.orientation, q);
        btMatrix3x3(q).getRPY(roll, pitch, yaw);
        m.th = yaw;

        m.v = odomsg.twist.twist.linear.x;
        m.w = odomsg.twist.twist.angular.z;

        pub.publish(m);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_odo_node");

    string topic_name;
    if( argc==2 ) {
        topic_name = argv[1];
    }
    else {
        ROS_ERROR("You must provide the topic name");
        return 1;
    }

    SimpleOdoNode n(topic_name);
    ros::spin();

    return 0;
}
