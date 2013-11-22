#include <ros/ros.h>
#include <math.h>
// #include <sensor_msgs/Joy.h>
#include <cawcatcher/CalPotRead.h>
#include <cawcatcher/AngleComm.h>
#include <geometry_msgs/Twist.h>

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
    double L[3][3];
    double R[3][3];
    int range_;

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

        L =
        {
            {a, a, 0}
            {-a, a, 0}
            {0, 0, 1}
        };

        R =
        {
            {a, -a, 0}
            {a, a, 0}
            {0, 0, 1}
        };
    }

    int update(double velocity)
    {
        dist_ = fmin(velocity*velocity/10, max_dist);
        double delta = sqrt(pow(range_, 2) - pow(width_, 2));
        double lamdamax = dist_ * delta / (width_ * (delta + width_));
        lamda_left_ = lamdamax*0.9;
        lamda_right_ = lamdamax*0.96;
        return 0;
    }

    int * getAngle();

};

namespace
{

double * MatrixMultiplication(double matrix1, double matrix2)
{
    static double matrixC[3][1];
    for(int i=0; i<3; i++)
    {
        matrixC[i][1]=0;
        for(int k=0; k<3; k++)
        {
            matrixC[i][1]=matrixC[i][1]+(matrix1[i][k] * matrix2[k][1]);
        }

    }
    return matrixC;
}

double * normal(double l, double h, double d, double lamda)
{
    static double vec[3][1];
    double val = sqrt(pow(l, 2) + pow(d - lamda*h, 2) - pow(lamda*l, 2));
    vec[1][1] = l/val;
    vec[2][1] = (d - lamda*h)/val;
    vec[3][1] = (lamda*l)/val;

    return vec;
}

}

int * CawCatcher::getAngle()
{
    static int double angle[4];

    double n_left = normal(width_, height_, dist_, lamda_left_);
    double n_right = normal(width_, -height_, -dist_, lamda_right_);
    double n_transf_left = MatrixMultiplication(L, n_left);
    double n_transf_right = MatrixMultiplication(R, n_right_);

    double pitch_left = atan2(n_transf_left[1][1], n_transf_left[3][1]);
    double roll_left = atan2(-n_transf_left[2][1] * sin(pitch_left), n_transf_left[1][1]);
    double pitch_right = atan2(n_transf_right[1][1], n_transf_right[3][1]);
    double roll_right = atan2(-n_transf_right[2][1] * sin(pitch_left), n_transf_right[1][1]);

    angle[1] = (int) roll_left * 180 / PI;
    angle[2] = (int) pitch_left * 180 / PI;
    angle[3] = (int) roll_right * 180 / PI;
    angle[4] = (int) pitch_right * 180 / PI;

    return angle;

}

CawCatcher iMiev(.75, 1.69, 50);

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

    iMiev.update(msg->linear.x);
    angle = iMiev.getAngle();

    angle_msg.LRoll_com = angle[1];
    angle_msg.LPitch_com = angle[2];
    angle_msg.RRoll_com = - angle[3];
    angle_msg.RPitch_com = angle[4];

    angle_pub->publish(angle_msg);
}

int main(int argc, char **argv)
{

//    ros::init(argc, argv, "adaptive");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("odometry", 1000, chatterCallback);

    ros::spin();

    return 0;















//    int main(int argc, char **argv)
//    {
//        ros::init(argc, argv, "joyIN");
//
//        ros::NodeHandle n;
//        int roll_angle  = 0;
//        int pitch_angle = 0;
////  ros::Subscriber sub = n.subscribe("/joy", 1, joy_servo_cb);
//
//        angle_pub = new ros::Publisher(n.advertise<cawcatcher::AngleComm>("cawcatcherIN", 1));
//
//        ros::Rate  r(10);
//
//        while (ros::ok())
//        {
//            roll_cg.update();
//            pitch_cg.update();
//
//            angle_msg.LRoll_com = roll_cg.getAngle();
//            angle_msg.LPitch_com = pitch_cg.getAngle();
//            angle_msg.RRoll_com = - roll_cg.getAngle();
//            angle_msg.RPitch_com = pitch_cg.getAngle();
//            angle_pub->publish(angle_msg);
//
//            r.sleep();
//        }
//        angle_msg.LRoll_com = 0;
//        angle_msg.LPitch_com = 0;
//        angle_msg.RRoll_com = 0;
//        angle_msg.RPitch_com = 0;
//        angle_pub->publish(angle_msg);
//        ros::spin();
//
//        return 0;
}











