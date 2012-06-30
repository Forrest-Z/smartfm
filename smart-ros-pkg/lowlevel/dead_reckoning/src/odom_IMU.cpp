/** Combines odometry (distance travelled) and IMU (yaw rate) to get position
 *  estimate.
 *
 * Gets the input from encoders and IMU ('imu/data', yaw rate only),
 * publishes the resulting pose estimate as a tf broadcast (odomImu / base_link)
 * and as a Odometry message on the 'odom' topic.
 */

#include <cmath>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <diagnostic_updater/publisher.h>

#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <fmutil/fm_math.h>

#include <phidget_encoders/EncoderOdo.h>


class OdoIMU
{
    public:
        OdoIMU();

    private:
        void encodersCallBack(phidget_encoders::EncoderOdo);
        void imuCallBack(sensor_msgs::Imu);
        void publishOdo();

        ros::NodeHandle nh_;
        ros::Subscriber enc_sub_;
        ros::Subscriber imu_sub_;
        ros::Publisher odo_imu_pub_;
        tf::TransformBroadcaster tf_broadcaster_;

        std::string frame_id_;
        geometry_msgs::Point position_;
        double linear_speed_, angular_speed_;
        bool initialized_;
        double roll_, pitch_, yaw_, yaw_now_;
        double dist_pre_;
        double yaw_pre_, yaw_drift_, yaw_minus_;

        diagnostic_updater::Updater diag_updater_;
        double diag_min_freq_, diag_max_freq_;
        diagnostic_updater::FrequencyStatusParam diag_param_fs_;
        diagnostic_updater::FrequencyStatus diag_task_fs_;
};



OdoIMU::OdoIMU()
: frame_id_("odom"),
  diag_min_freq_(5), diag_max_freq_(50),
  diag_param_fs_(&diag_min_freq_, &diag_max_freq_),
  diag_task_fs_(diag_param_fs_)
{
    enc_sub_ = nh_.subscribe("/encoder_odo", 100, &OdoIMU::encodersCallBack, this);
    imu_sub_ = nh_.subscribe("/ms/imu/data", 100, &OdoIMU::imuCallBack, this);
    odo_imu_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);

    ros::NodeHandle n("~");
    n.getParam("frame_id", frame_id_);

    initialized_ = false;
    yaw_drift_ = 0;
    roll_ = pitch_ = yaw_ = NAN;

    diag_updater_.setHardwareID("none");
    diag_updater_.add(diag_task_fs_);
}

#define R2D(a) ( (int)(fmutil::r2d( fmutil::angModPI(a) )) )

void OdoIMU::imuCallBack(sensor_msgs::Imu imuMsg)
{
    // Get RPY from the IMU
    tf::Quaternion qt;
    tf::quaternionMsgToTF(imuMsg.orientation, qt);
    tf::Matrix3x3(qt).getRPY(roll_, pitch_, yaw_);
    ROS_DEBUG("rpy: %d, %d, %d (degrees)", R2D(roll_), R2D(pitch_), R2D(yaw_));
}


void OdoIMU::encodersCallBack(phidget_encoders::EncoderOdo encMsg)
{
    //handle the nan issue. This occurs when the imu is restarted
    if( isnan(roll_) || isnan(pitch_) || isnan(yaw_) )
        return;
	yaw_now_ = yaw_;
    if( ! initialized_ )
    {
        yaw_pre_ = yaw_now_;
        yaw_minus_ = yaw_now_;
        initialized_ = true;
        return;
    }
    //Ensure that the orientation always start from yaw=0.
    //That's the assumption made for odometry calculation

    // Only integrate yaw when the car is moving
    if( fabs(encMsg.v) < 0.01 ) //the car is not moving
        yaw_drift_ += yaw_now_ - yaw_minus_;
    yaw_minus_ = yaw_now_;
    yaw_now_ -= yaw_pre_ + yaw_drift_;
    //std::cout <<yaw_ <<' ' <<yaw_drift_ <<std::endl;

    double r11 = cos(yaw_now_)*cos(pitch_);
    double r21 = sin(yaw_now_)*cos(pitch_);
    double r31 = sin(pitch_);
    position_.x += encMsg.d_dist * r11;
    position_.y += encMsg.d_dist * r21;
    position_.z -= encMsg.d_dist * r31;

    linear_speed_ = encMsg.v;
    angular_speed_ = encMsg.w;

    ROS_DEBUG("Pose: x=%.2f, y=%.2f, th=%ddeg", position_.x, position_.y, R2D(yaw_now_));
    publishOdo();
    diag_task_fs_.tick();
    diag_updater_.update();
}


void OdoIMU::publishOdo()
{
    // Create the Odometry msg
    nav_msgs::Odometry odoImuMsg;
    odoImuMsg.header.stamp = ros::Time::now();
    odoImuMsg.header.frame_id = frame_id_;
    odoImuMsg.child_frame_id = "base_link";
    odoImuMsg.pose.pose.position = position_;
    odoImuMsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_now_);
    odoImuMsg.twist.twist.linear.x = linear_speed_;
    odoImuMsg.twist.twist.angular.z = angular_speed_;
    // Publish it
    odo_imu_pub_.publish(odoImuMsg);


    // Broadcast the TF
    tf::StampedTransform trans(tf::Transform(), odoImuMsg.header.stamp, odoImuMsg.header.frame_id, odoImuMsg.child_frame_id);
    tf::poseMsgToTF(odoImuMsg.pose.pose, trans);
    tf_broadcaster_.sendTransform(trans);
}






int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_Imu");
    OdoIMU odomimu;
    ros::spin();
    return 0;
}
