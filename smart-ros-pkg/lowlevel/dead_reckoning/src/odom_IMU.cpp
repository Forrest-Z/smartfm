/** Combines odometry (distance travelled) and IMU (yaw rate) to get position
 *  estimate.
 *
 * Gets the input from encoders and IMU ('imu/data', yaw rate only),
 * publishes the resulting pose estimate as a tf broadcast (odomImu / base_link)
 * and as a Odometry message on the 'odom_imu' channel.
 */

#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <fmutil/fm_math.h>
#include <phidget_encoders/Encoders.h>

class OdoIMU
{
    public:
        OdoIMU(ros::NodeHandle nh_);

    private:
        void encodersCallBack(phidget_encoders::Encoders);
        void imuCallBack(sensor_msgs::Imu);
        void publishOdo();

        ros::NodeHandle n;
        ros::Subscriber encSub;
        ros::Subscriber imuSub;
        ros::Publisher odoImuPub;
        tf::TransformBroadcaster tfBroadcaster;

        geometry_msgs::Point position;
        double linear_speed, angular_speed;
        bool initialized;
        double roll, pitch, yaw, yaw_now;
        double dist_pre;
        double yaw_pre, yaw_drift, yaw_minus;
        double accumulated_dist;
};



OdoIMU::OdoIMU(ros::NodeHandle nh_):n(nh_)
{
    ROS_INFO("OdoIMU initializing");
    encSub = n.subscribe("/encoders", 1000, &OdoIMU::encodersCallBack, this);
    imuSub = n.subscribe("/ms/imu/data", 1000, &OdoIMU::imuCallBack, this);
    odoImuPub = n.advertise<nav_msgs::Odometry>("odo_imu", 100);

    initialized = false;
    yaw_drift = 0;
    roll = pitch = yaw = NAN;
    ROS_INFO("OdoIMU initialized");
}

#define R2D(a) ( (int)(fmutil::r2d( fmutil::angModPI(a) )) )

void OdoIMU::imuCallBack(sensor_msgs::Imu imuMsg)
{
    // Get RPY from the IMU
    tf::Quaternion qt;
    tf::quaternionMsgToTF(imuMsg.orientation, qt);
    btMatrix3x3(qt).getRPY(roll, pitch, yaw);
    ROS_DEBUG("rpy: %d, %d, %d (degrees)", R2D(roll), R2D(pitch), R2D(yaw));
}


void OdoIMU::encodersCallBack(phidget_encoders::Encoders encMsg)
{
    //handle the nan issue. This occurs when the imu is restarted
    if( isnan(roll) || isnan(pitch) || isnan(yaw) )
        return;
	yaw_now = yaw;
    if( ! initialized )
    {
        yaw_pre = yaw_now;
        yaw_minus = yaw_now;
        initialized = true;
        return;
    }
    //Ensure that the orientation always start from yaw=0.
    //That's the assumption made for odometry calculation

    // Only integrate yaw when the car is moving
    if( fabs(encMsg.v) < 0.01 ) //the car is not moving
        yaw_drift += yaw_now - yaw_minus;
    yaw_minus = yaw_now;
    yaw_now -= yaw_pre + yaw_drift;
    //std::cout <<yaw <<' ' <<yaw_drift <<std::endl;

    double r11 = cos(yaw_now)*cos(pitch);
    double r21 = sin(yaw_now)*cos(pitch);
    double r31 = sin(pitch);
    accumulated_dist+=encMsg.d_dist;
    position.x += encMsg.d_dist * r11;
    position.y += encMsg.d_dist * r21;
    position.z -= encMsg.d_dist * r31;
    
    linear_speed = encMsg.v;
    angular_speed = encMsg.w;

    ROS_DEBUG("Pose: x=%.2f, y=%.2f, th=%ddeg", position.x, position.y, (int)(yaw_now*180/M_PI));
    //std::cout<<accumulated_dist<<std::endl;
    publishOdo();
}


void OdoIMU::publishOdo()
{
    // Create the Odometry msg
    nav_msgs::Odometry odoImuMsg;
    odoImuMsg.header.stamp = ros::Time::now();
    odoImuMsg.header.frame_id = "odom";
    odoImuMsg.child_frame_id = "base_link";
    odoImuMsg.pose.pose.position = position;
    odoImuMsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw_now);
    odoImuMsg.twist.twist.linear.x = linear_speed;
    odoImuMsg.twist.twist.angular.z = angular_speed;
    // Publish it
    odoImuPub.publish(odoImuMsg);


    // Broadcast the TF
    tf::StampedTransform trans(tf::Transform(), odoImuMsg.header.stamp, odoImuMsg.header.frame_id, odoImuMsg.child_frame_id);
    tf::poseMsgToTF(odoImuMsg.pose.pose, trans);
    tfBroadcaster.sendTransform(trans);
}






int main(int argc, char **argv)
{
    ros::init(argc, argv, "odomImu");
    ROS_INFO("hallooooo");
    ros::NodeHandle nh_;
    OdoIMU odomimu(nh_);
    ros::spin();
    return 0;
}