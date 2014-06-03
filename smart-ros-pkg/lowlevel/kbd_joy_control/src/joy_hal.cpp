/** Reads position of the joystick, translates it into velocity commands, and
 * sends it to the lowlevel controls of the golfcar: throttle, steer_angle,
 * brake_angle, etc.
 *
 * NOTE:
 * to run the joystick node:
 *  rosparam set joy_node/dev "/dev/input/js0"
 *  rosrun joy joy_node
 */

#include <stdio.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>


#define STEER_ANG_MAX 450
#define FULL_BRAKE 100
#define FULL_THROTTLE 1
ros::Publisher throttle_pub, steering_pub, brake_pub;//, enable_pub;


void joyCallBack(sensor_msgs::Joy joy_msg)
{
    // 3 axes joystick, each axis value ranges from -1.0 to 1.0
    // axes[0] is the lateral axis: positive to the left
    // axes[1] is the longitudinal axis: positive to the front
    // axes[2] is the wheel axis: positive upward.

    float lat = joy_msg.axes[3];
    float lon = joy_msg.axes[1];

    ROS_INFO("Joystick: axes[1] (lat)=%f, axis[3] (lon)=%f, axis[2] (wheel)=%f",
    joy_msg.axes[1], joy_msg.axes[3], joy_msg.axes[2]);

    std_msgs::Float64 V, B;
    V.data = lon>0 ? lon*FULL_THROTTLE : 0;
    B.data = lon>0 ? 0 : -lon*FULL_BRAKE;
    throttle_pub.publish(V);
    brake_pub.publish(B);

    std_msgs::Float64 W;
    W.data = lat * STEER_ANG_MAX;
    steering_pub.publish(W);

    ROS_INFO("Resulting commands: throttle=%lf, brake_angle=%lf, steer_angle=%lf",
    V.data, B.data, W.data);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_drive");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joy", 10, joyCallBack);

    throttle_pub = n.advertise<std_msgs::Float64>("throttle", 1);
    steering_pub = n.advertise<std_msgs::Float64>("steer_angle", 1);
    brake_pub = n.advertise<std_msgs::Float64>("brake_angle", 1);
    //enable_pub = n.advertise<std_msgs::Bool>("hal_streamer_bit_3", 10);

    puts("Reading from Joystick");
    puts("---------------------------");

    ros::spin();

    return 0;
}
