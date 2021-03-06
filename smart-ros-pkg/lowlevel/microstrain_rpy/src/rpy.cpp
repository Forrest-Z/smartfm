/// Republishes IMU messages as imu_rpy messages (roll, pitch and yaw in degrees).

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <microstrain_rpy/imu_rpy.h>

ros::Publisher *imu_pub_;

void imuCallback(const sensor_msgs::Imu::ConstPtr msg)
{
    //added to publish rpy and tf
    static tf::TransformBroadcaster broadcaster_b;
    
    btScalar pitch, roll, yaw;
    geometry_msgs::Quaternion orientation = msg->orientation;
    btQuaternion btq(orientation.x, orientation.y, orientation.z, orientation.w);
    btMatrix3x3(btq).getEulerYPR(yaw, pitch, roll);
    
    //correct the roll angle from +-180 to +-0
    //if(roll>=0) roll-=M_PI;
    //else roll+=M_PI;

    btMatrix3x3 btm;
    btm.setEulerYPR(yaw, pitch, roll);
    btQuaternion btqt_temp;
    btm.getRotation(btqt_temp);
    microstrain_rpy::imu_rpy rpy;

    rpy.roll = roll/M_PI*180;
    rpy.pitch = pitch/M_PI*180;
    rpy.yaw = yaw/M_PI*180;
    broadcaster_b.sendTransform(
        tf::StampedTransform(
            tf::Transform(btqt_temp, tf::Vector3(0,0,0)),
                             ros::Time::now(), "ms_imu_base", "ms_imu"));
    imu_pub_->publish(rpy);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "microstrain_rpy");
    ros::NodeHandle n;
    ros::Publisher imu_pub= n.advertise<microstrain_rpy::imu_rpy>("rpy",1);
    imu_pub_ = &imu_pub;
    ros::Subscriber sub = n.subscribe("data", 1, imuCallback);
    ros::spin();

    return 0;
}
