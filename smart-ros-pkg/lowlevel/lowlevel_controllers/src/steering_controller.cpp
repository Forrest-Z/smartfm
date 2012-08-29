/** Steering controller.
 *
 * Polynomial control law to control the steering wheel position, based on
 * requested angular velocity.
 *
 * @todo make the parameters more flexible, especially the max and min steering
 * angle.
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <fmutil/fm_math.h>


class SteeringController
{
public:
    SteeringController();

private:
    void emergencyBtnCB(std_msgs::Bool);
    void cmdSteerCallBack(geometry_msgs::Twist);

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Subscriber emergency_btn_sub;
    ros::Publisher steer_pub;

    double distance_threshold;
    double gc_x, gc_y, yaw_feedback;
    bool emergency;
    int point_counts;
};



SteeringController::SteeringController()
{
    sub = n.subscribe("cmd_steer", 1000, &SteeringController::cmdSteerCallBack, this);
    emergency_btn_sub = n.subscribe("button_state_emergency", 1000, &SteeringController::emergencyBtnCB, this);
    steer_pub = n.advertise<std_msgs::Float64>("steer_angle", 1);
    emergency = false;
}


void SteeringController::emergencyBtnCB(std_msgs::Bool msg)
{
    emergency = msg.data;
}


void SteeringController::cmdSteerCallBack(geometry_msgs::Twist cmd_steer)
{
    float st = 0;
    if( ! emergency )
    {
        double wheel_angle = fmutil::r2d(cmd_steer.angular.z);
        double steering_angle = - 0.0016 * pow(wheel_angle,3)
                                - 0.0032 * pow(wheel_angle,2)
                                + 16.648 * wheel_angle
                                - 1.3232;
        fmutil::bound<double>(-540, &steering_angle, 540);
        st = -steering_angle;
    }

    std_msgs::Float64 msg;
    msg.data = st;
    steer_pub.publish(msg);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "steering_controller");
    SteeringController steeringController;
    ros::spin();
    return 0;
}
