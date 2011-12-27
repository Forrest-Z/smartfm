/** Combines odometry (distance travelled) and IMU (yaw rate) to get position
 *  estimate.
 *
 * Gets the input from odometry ('golfcar_sampler') and 'pose2D',
 * publishes the resulting pose estimate as a tf broadcast (odom / base_link)
 * and as a Odometry message on the 'odom' channel.
 */

#include <math.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>

#include <golfcar_halsampler/odo.h>




class GolfcarOdometry
{
    public:
        GolfcarOdometry(ros::NodeHandle nh_);

    private:
        void pose2DCallBack(geometry_msgs::Pose2D pose2d);
        void samplerCallBack(golfcar_halsampler::odo sampler);

        ros::NodeHandle n;
        ros::Subscriber odoSub, pose2dSub;
        ros::Publisher odomPub;
        tf::TransformBroadcaster broadcaster_b;

        nav_msgs::Odometry odom;

        bool pose_init;
        double pose_pre, yaw;
};



GolfcarOdometry::GolfcarOdometry(ros::NodeHandle nh_) : n(nh_)
{
    odoSub = n.subscribe("golfcar_sampler", 1000, &GolfcarOdometry::samplerCallBack, this);
    pose2dSub = n.subscribe("pose2D", 1000, &GolfcarOdometry::pose2DCallBack, this);
    odomPub = n.advertise<nav_msgs::Odometry>("odom", 100);

    odom.header.frame_id = "odom";

    pose_init = false;
    yaw = 0;
}


void GolfcarOdometry::pose2DCallBack(geometry_msgs::Pose2D pose2d)
{
    yaw = pose2d.theta;
}


void GolfcarOdometry::samplerCallBack(golfcar_halsampler::odo sampler)
{
    if( ! pose_init )
    {
        //initialize position as zero and take first reading of yaw_rate
        pose_pre = sampler.pose;
        pose_init = true;
        return;
    }

    //calculate absolute x and y from heading and wheel encoder pose
    double distance = sampler.pose - pose_pre;
    pose_pre = sampler.pose;

    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x += distance * cos(yaw);
    odom.pose.pose.position.y += distance * sin(yaw);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = sampler.vel;
    odomPub.publish(odom);

    broadcaster_b.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::createQuaternionFromYaw(yaw),
                tf::Vector3(
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    0.0
                )
            ), ros::Time::now(), "odom", "base_link"
        )
    );
}






int main(int argc, char **argv)
{
    ros::init(argc, argv, "golfcar_odom");
    ros::NodeHandle nh_;
    GolfcarOdometry odom(nh_);
    ros::spin();
    return 0;
}
