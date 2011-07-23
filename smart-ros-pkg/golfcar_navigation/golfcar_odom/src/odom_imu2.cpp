#include <odom_imu2.h>

using namespace std;
namespace golfcar_odometry_imu{
    //Constructor
    golfcar_odometry_imu::golfcar_odometry_imu(ros::NodeHandle nh_):n(nh_)
    {
        sub = n.subscribe("golfcar_sampler", 1000, &golfcar_odometry_imu::samplerCallBack, this);
        imu = n.subscribe("imu/data", 1000, &golfcar_odometry_imu::imuCallBack, this);

        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
        initialized = false;
        yaw_drift =0;
        yaw_minus =0;
    }

    void golfcar_odometry_imu::imuCallBack(sensor_msgs::Imu imu)
    {
        imuQ = imu.orientation;
    }

    void golfcar_odometry_imu::samplerCallBack(golfcar_odom::odo sampler)
    {

        static tf::TransformBroadcaster broadcaster_b;
        tf::Quaternion qt_temp;
        btScalar pitch, roll, yaw;
        tf::Quaternion qt;

        tf::quaternionMsgToTF(imuQ, qt);

        btMatrix3x3(qt).getRPY(roll, pitch, yaw);
        //cout<<"rpy: "<< roll<<" "<< pitch <<" " <<yaw << endl;
        //handle the nan issue. This occur when the imu is restarted 
        if(isnan(yaw)||isnan(pitch)||isnan(roll)) 
            return;


        btMatrix3x3 btm;

        if(!initialized) 
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
        yaw -= (yaw_pre+yaw_drift);
        cout<<yaw<<' '<<yaw_drift<<endl;
        btm.setRPY(roll, pitch, yaw);
        btm.getRotation(qt_temp);
        geometry_msgs::Quaternion qMsg;
        tf::quaternionTFToMsg(qt_temp,qMsg);

        double r11 = cos(yaw)*cos(pitch);
        double r21 = sin(yaw)*cos(pitch);
        double r31 = sin(pitch);
        double distance = sampler.pose - pose_pre;
        pose_pre = sampler.pose;

        odom.header.frame_id = "odom";
        odom.header.stamp = ros::Time::now();

        odom.pose.pose.position.x+=distance * r11;
        odom.pose.pose.position.y+=distance * r21;
        odom.pose.pose.position.z-=distance * r31;
        odom.pose.pose.orientation = qMsg;

        broadcaster_b.sendTransform(
                tf::StampedTransform(
                    tf::Transform(qt_temp,tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z)),
                    ros::Time::now(),"odom", "base_link"));

        odom_pub.publish(odom);
    }


};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "golfcar_odom_imu");
    ros::NodeHandle nh_;
    golfcar_odometry_imu::golfcar_odometry_imu *odom_ = new golfcar_odometry_imu::golfcar_odometry_imu(nh_);


    ros::spin();

    return 0;
}
