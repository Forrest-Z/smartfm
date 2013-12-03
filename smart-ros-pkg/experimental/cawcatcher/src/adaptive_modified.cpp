#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <math.h>
#include <cawcatcher/CalPotRead.h>
#include <cawcatcher/AngleComm.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>

ros::Publisher *angle_pub;
cawcatcher::AngleComm angle_msg;

class CawCatcher
{
    double width_;
    double height_;
    double dist_;
    double max_dist_;
    double lamda_left_;
    double lamda_right_;
    int range_;
    Eigen::Matrix3d L;
    Eigen::Matrix3d R;

public:
    CawCatcher(double width,double height, int range)
    {
        width_ = width;
        height_ = height;
        range_ = range;
        max_dist_ = sqrt(pow(range_, 2) - pow(width_, 2) - pow(height_, 2));
        dist_ = 0;
        double a = 0.7017;
        lamda_left_ = dist_/width_;
        lamda_right_ = dist_/width_;

        L <<
        a, a, 0,
        -a, a, 0,
        0, 0, 1;

        R <<
        a, -a, 0,
        a, a, 0,
        0, 0, 1;

    }

    double compute_lamda(double y_)
    {   
        return (dist_ * y_) / (height_ * (y_ + width_));
    }

    int update(double velocity)
    {
        dist_ = fmin(pow(velocity/10, 2), max_dist_);
        dist_ = fmax(dist_, 3.0);
        double delta = sqrt(pow(range_, 2) - pow(width_, 2));
        double lamdamax = dist_ * delta / (width_ * (delta + width_));
        lamda_left_ = fmin(compute_lamda(5), lamdamax);
        lamda_right_ = fmin(compute_lamda(20), lamdamax);

        std::cout << "vel (km/h): " << velocity << std::endl;
	std::cout << "dist: " << dist_ << std::endl;
        std::cout << "lamda_L: " << (lamda_left_) << std::endl;
	std::cout << "lamda_Max: " << (lamdamax) << std::endl;
        return 0;
    }

    Eigen::VectorXf getAngle()
    {
        Eigen::VectorXd n_left(3);
        Eigen::VectorXd n_right(3);
        Eigen::VectorXf angle(4);

        n_left << width_, dist_ - lamda_left_ * height_, lamda_left_ * width_;
        std::cout << "vector n: [" << n_left[0] << ", " << n_left[1] << ", " << n_left[2] << "]" << std::endl;
        std::cout << "norm n: " << n_left.norm() << std::endl;
        n_right << width_, -dist_ + lamda_right_ * height_, lamda_right_ * width_;
        n_left = n_left/n_left.norm();
        std::cout << "unit vector n: [" << n_left[0] << ", " << n_left[1] << ", " << n_left[2] << "]" << std::endl;
        n_right = n_right/n_right.norm();
        n_left = L * n_left;
        std::cout << "unit vector nL: [" << n_left[0] << ", " << n_left[1] << ", " << n_left[2] << "]" << std::endl;
        std::cout << "norm n: " << n_left.norm() << std::endl;
        n_right = R * n_right;
/*               
        angle(1) = atan2(n_left(0), n_left(2));
        angle(0) = atan2(-n_left(1) * sin(angle(1)), n_left(0));
        angle(3) = atan2(n_right(0), n_right(2));
        angle(2) = atan2(-n_right(1) * sin(angle(3)), n_right(0));
*/
        angle(0) = asin(-n_left(1));
        angle(1) = asin(n_left(0) / cos(angle(0)));
        angle(2) = asin(-n_right(1));
        angle(3) = asin(n_right(0) / cos(angle(2)));

        return angle;
    }
};

CawCatcher iMiev_(0.75, 1.69, 50);

//void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	
    iMiev_.update(3.6 * msg->twist.twist.linear.x);
    Eigen::VectorXf angle(4);
    angle = iMiev_.getAngle();

    angle_msg.header.seq = msg->header.seq;
    angle_msg.header.stamp = msg->header.stamp;
    angle_msg.LRoll_com = (float) angle(0) * 180 / M_PI;
    angle_msg.LPitch_com = (float) angle(1) * 180 / M_PI;
    angle_msg.RRoll_com = (float) angle(2) * 180 / M_PI;
    angle_msg.RPitch_com = (float) angle(3) * 180 / M_PI;

    angle_pub->publish(angle_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "adaptive");
    ros::NodeHandle n;
    ros::Rate  r(200);
    ros::Subscriber sub = n.subscribe("can_imu_odom", 1, chatterCallback);
    angle_pub = new ros::Publisher(n.advertise<cawcatcher::AngleComm>("cawcatcherIN_2", 1));

    ros::spin();

    return 0;
}

