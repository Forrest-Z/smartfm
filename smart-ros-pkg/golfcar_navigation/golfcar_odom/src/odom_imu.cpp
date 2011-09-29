/** Combines odometry (distance travelled) and IMU (yaw rate) to get position
 *  estimate.
 *
 * Gets the input from odometry ('golfcar_sampler') and IMU ('imu/data'),
 * publishes the resulting pose estimate as a tf broadcast (odom / base_link)
 * and as a Odometry message on the 'odom' channel.
 */

#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <golfcar_halsampler/odo.h>


class GolfcarOdoIMU
{
    public:
        GolfcarOdoIMU(ros::NodeHandle nh_);

    private:
        void samplerCallBack(golfcar_halsampler::odo);
        void imuCallBack(sensor_msgs::Imu);

        ros::NodeHandle n;
        ros::Subscriber odoSub, imuSub;
        ros::Publisher odomPub;
        tf::TransformBroadcaster broadcaster_b;

        nav_msgs::Odometry odom;
        geometry_msgs::Quaternion imuQ;

        bool imu_started, pose_init;
        double pose_pre, yaw_pre, yaw_rate, yaw_;
};



GolfcarOdoIMU::GolfcarOdoIMU(ros::NodeHandle nh_) : n(nh_)
{
    odoSub = n.subscribe("golfcar_sampler", 1000, &GolfcarOdoIMU::samplerCallBack, this);
    imuSub = n.subscribe("imu/data", 1000, &GolfcarOdoIMU::imuCallBack, this);
    odomPub = n.advertise<nav_msgs::Odometry>("odom", 100);

    odom.header.frame_id = "odom";

    imu_started = false;
    pose_init = false;
    yaw_ = 0;
}


void GolfcarOdoIMU::imuCallBack(sensor_msgs::Imu imu)
{
    imu_started = true;
    imuQ = imu.orientation;
    yaw_rate = imu.angular_velocity.z;
}


void GolfcarOdoIMU::samplerCallBack(golfcar_halsampler::odo sampler)
{
    btScalar pitch, roll, yaw;
    tf::Quaternion qt;
    tf::quaternionMsgToTF(imuQ, qt);
    btMatrix3x3(qt).getRPY(yaw, pitch, roll);
    yaw = - yaw; //the quaternion given out by xsens is weird. It is rotated 90 degree along y-axis

    if( ! pose_init || ! imu_started)
    {
        //initialize position as zero and take first reading of yaw_rate
        pose_pre = sampler.pose;
        yaw_pre = yaw;
        pose_init = true;
        if( imu_started )
            ROS_INFO("Odom start!");
        return;
    }


    double del_yaw=0;
    //only do integration when the vehicle is moving
    if(sampler.vel>0.05)
        del_yaw = yaw - yaw_pre;

    //calculate absolute x and y from heading and wheel encoder pose
    double distance = sampler.pose - pose_pre;
    pose_pre = sampler.pose;
    btMatrix3x3 btm;
    btm.setRPY(roll, pitch, yaw_);
    tf::Quaternion qt_temp;
    geometry_msgs::Quaternion imu_temp;
    btm.getRotation(qt_temp);
    tf::quaternionTFToMsg(qt_temp, imu_temp);

    double r11 = cos(yaw_+del_yaw/2);
    double r21 = sin(yaw_+del_yaw/2);
    double r31 = sin(pitch);
    yaw_ += del_yaw;
    yaw_pre = yaw;

    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x += distance*r11;
    odom.pose.pose.position.y += distance*r21;
    odom.pose.pose.position.z += distance*r31;
    odom.pose.pose.orientation = imu_temp;
    odom.twist.twist.linear.x = sampler.vel;
    odom.twist.twist.angular.x = distance;
    odomPub.publish(odom);

    broadcaster_b.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                qt_temp,
                tf::Vector3(
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z
                )
            ),
            ros::Time::now(), "odom", "base_link"
        )
    );
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "golfcar_odom_imu");
    ros::NodeHandle nh_;
    GolfcarOdoIMU odomimu(nh_);
    ros::spin();
    return 0;
}
