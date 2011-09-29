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
        void samplerCallBack(golfcar_halsampler::odo sampler);
        void imuCallBack(sensor_msgs::Imu imu);

        ros::NodeHandle n;
        ros::Subscriber odoSub;
        ros::Subscriber imuSub;
        ros::Publisher odom_pub;
        ros::Publisher filter_pub;
        nav_msgs::Odometry odom;
        geometry_msgs::Quaternion imuQ;
        golfcar_halsampler::odo golfcarodo;
        tf::TransformBroadcaster broadcaster_b;

        int pose_init;
        double yaw_pre, yaw_drift, yaw_minus;
        bool initialized;
        double pose_pre;
};


GolfcarOdoIMU::GolfcarOdoIMU(ros::NodeHandle nh_):n(nh_)
{
    odoSub = n.subscribe("golfcar_sampler", 1000, &GolfcarOdoIMU::samplerCallBack, this);
    imuSub = n.subscribe("imu/data", 1000, &GolfcarOdoIMU::imuCallBack, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);

    initialized = false;
    yaw_drift = 0;
    yaw_minus = 0;
}

void GolfcarOdoIMU::imuCallBack(sensor_msgs::Imu imu)
{
    imuQ = imu.orientation;
}

void GolfcarOdoIMU::samplerCallBack(golfcar_halsampler::odo sampler)
{
    tf::Quaternion qt;
    tf::quaternionMsgToTF(imuQ, qt);
    btScalar pitch, roll, yaw;
    btMatrix3x3(qt).getRPY(roll, pitch, yaw);
    //std::cout<<"rpy: " << roll <<" " <<pitch <<" " <<yaw <<std::endl;

    //handle the nan issue. This occurs when the imu is restarted
    if( isnan(yaw) || isnan(pitch) || isnan(roll) )
        return;

    if( ! initialized )
    {
        yaw_pre = yaw;
        yaw_minus = yaw;
        pose_pre = sampler.pose;
        initialized = true;
    }
    //Ensure that the orientation always start from yaw=0. That's the assumption made for odometry calculation

    //Only integrate yaw when the car is moving
    if(sampler.vel < 0.01 && sampler.vel > -0.01)
    {
        yaw_drift += (yaw - yaw_minus);
    }
    yaw_minus = yaw;
    yaw -= yaw_pre + yaw_drift;
    std::cout <<yaw <<' ' <<yaw_drift <<std::endl;

    btMatrix3x3 btm;
    btm.setRPY(roll, pitch, yaw);
    tf::Quaternion qt_temp;
    btm.getRotation(qt_temp);
    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(qt_temp, qMsg);

    double r11 = cos(yaw)*cos(pitch);
    double r21 = sin(yaw)*cos(pitch);
    double r31 = sin(pitch);
    double distance = sampler.pose - pose_pre;
    pose_pre = sampler.pose;

    odom.header.frame_id = "odom";
    odom.header.stamp = ros::Time::now();

    odom.pose.pose.position.x += distance * r11;
    odom.pose.pose.position.y += distance * r21;
    odom.pose.pose.position.z -= distance * r31;
    odom.pose.pose.orientation = qMsg;

    tf::Vector3 pose3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    broadcaster_b.sendTransform(
        tf::StampedTransform(
            tf::Transform(qt_temp, pose3),
            ros::Time::now(), "odom", "base_link"
        )
    );

    odom_pub.publish(odom);
}






int main(int argc, char **argv)
{
    ros::init(argc, argv, "golfcar_odom_imu");
    ros::NodeHandle nh_;
    GolfcarOdoIMU odomimu(nh_);
    ros::spin();
    return 0;
}
