/** Combines odometry (distance travelled) and IMU (yaw rate) to get position
 *  estimate.
 *
 * Gets the input from odometry ('odoImuMsg') and IMU ('imu/data', yaw rate only),
 * publishes the resulting pose estimate as a tf broadcast (odomImu / base_link)
 * and as a Odometry message on the 'odom_imu' channel.
 */

#include <math.h>

#include <ros/ros.h>
//#include <ros/console.h>

#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <lowlevel/Odometry.h>


class OdoIMU
{
    public:
        OdoIMU(ros::NodeHandle);

    private:
        void odoCallBack(lowlevel::Odometry);
        void imuCallBack(sensor_msgs::Imu);
        void publishOdo();

        ros::NodeHandle n;
        ros::Subscriber odoSub;
        ros::Subscriber imuSub;
        ros::Publisher odoImuPub;

        geometry_msgs::Point position;

        bool initialized;
        double roll, pitch, yaw;
        double pose_pre;
        double yaw_pre, yaw_drift, yaw_minus;
};



OdoIMU::OdoIMU(ros::NodeHandle nh_) : n(nh_)
{
    odoSub = n.subscribe("odom_linear", 1000, &OdoIMU::odoCallBack, this);
    imuSub = n.subscribe("imu/data", 1000, &OdoIMU::imuCallBack, this);
    odoImuPub = n.advertise<nav_msgs::Odometry>("odo_imu", 100);

    initialized = false;
    yaw_drift = 0;
    roll = pitch = yaw = NAN;
}


void OdoIMU::imuCallBack(sensor_msgs::Imu imuMsg)
{
    // Get RPY from the IMU
    tf::Quaternion qt;
    tf::quaternionMsgToTF(imuMsg.orientation, qt);
    btMatrix3x3(qt).getRPY(roll, pitch, yaw);
    //std::cout<<"rpy: " << roll <<" " <<pitch <<" " <<yaw <<std::endl;
}


void OdoIMU::odoCallBack(lowlevel::Odometry odoMsg)
{
    //handle the nan issue. This occurs when the imu is restarted
    if( isnan(roll) || isnan(pitch) || isnan(yaw) )
        return;

    if( ! initialized )
    {
        yaw_pre = yaw;
        yaw_minus = yaw;
        pose_pre = odoMsg.pose;
        initialized = true;
        return;
    }
    //Ensure that the orientation always start from yaw=0.
    //That's the assumption made for odometry calculation

    // Only integrate yaw when the car is moving
    if( fabs(odoMsg.vel) < 0.01 ) //the car is not moving
        yaw_drift += yaw - yaw_minus;
    yaw_minus = yaw;
    yaw -= yaw_pre + yaw_drift;
    //std::cout <<yaw <<' ' <<yaw_drift <<std::endl;

    double distance = odoMsg.pose - pose_pre;
    pose_pre = odoMsg.pose;

    double r11 = cos(yaw)*cos(pitch);
    double r21 = sin(yaw)*cos(pitch);
    double r31 = sin(pitch);

    position.x += distance * r11;
    position.y += distance * r21;
    position.z -= distance * r31;

    ROS_INFO("Pose: x=%.2f, y=%.2f, th=%ddeg", position.x, position.y, (int)(yaw*180/M_PI));

    publishOdo();
}


void OdoIMU::publishOdo()
{
    // Create the Odometry msg
    nav_msgs::Odometry odoImuMsg;
    odoImuMsg.header.stamp = ros::Time::now();
    odoImuMsg.header.frame_id = "odom_imu";
    odoImuMsg.child_frame_id = "base_link";
    odoImuMsg.pose.pose.position = position;
    odoImuMsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    // Publish it
    odoImuPub.publish(odoImuMsg);


    // Broadcast the TF
    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform trans(tf::Transform(), odoImuMsg.header.stamp, odoImuMsg.header.frame_id, odoImuMsg.child_frame_id);
    tf::poseMsgToTF(odoImuMsg.pose.pose, trans);
    tfBroadcaster.sendTransform(trans);
}






int main(int argc, char **argv)
{
    ros::init(argc, argv, "odomImu");
    ros::NodeHandle nh_;
    OdoIMU odomimu(nh_);
    ros::spin();
    return 0;
}
