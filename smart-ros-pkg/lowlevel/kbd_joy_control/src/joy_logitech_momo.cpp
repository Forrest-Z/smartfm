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


#define STEER_ANG_MAX 445
#define FULL_BRAKE 100
#define FULL_THROTTLE 1000
ros::Publisher throttle_pub, steering_pub, brake_pub, lblinker_pub, rblinker_pub;
ros::Publisher direction_pub_;
ros::Time last_update;

using namespace std;
void joyCallBack(sensor_msgs::Joy joy_msg)
{
  last_update = ros::Time::now();
  std_msgs::Float64 steering_angle, brake_angle, throttle;
  steering_angle.data = -joy_msg.axes[0] * 540;
  if(joy_msg.axes[1]>0) {
    throttle.data = joy_msg.axes[1];
    brake_angle.data = 0;
  }
  else {
    brake_angle.data = -joy_msg.axes[1] * 120;
    throttle.data = 0;
  }
  std_msgs::Bool direction;
  if(joy_msg.buttons[8]) {
    direction.data = false;
    direction_pub_.publish(direction);
  }
  if(joy_msg.buttons[9]){
    direction.data = true;
    direction_pub_.publish(direction);
  }
  steering_pub.publish(steering_angle);
  brake_pub.publish(brake_angle);
  throttle_pub.publish(throttle);
}
void timerCallback(const ros::TimerEvent& e)
{
  double last_update_duration = (ros::Time::now() - last_update).toSec();
  if(last_update_duration > 0.5){
    cout<<"Joystick has no responds! Stopping the vehicle."<<endl;
    std_msgs::Float64 zero_throttle, full_brake;
    zero_throttle.data = 0;
    full_brake.data = 120;
    throttle_pub.publish(zero_throttle);
    brake_pub.publish(full_brake);
  }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_drive");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joy", 1000, joyCallBack);

    throttle_pub = n.advertise<std_msgs::Float64>("throttle", 1000);
    steering_pub = n.advertise<std_msgs::Float64>("steer_angle", 1000);
    brake_pub = n.advertise<std_msgs::Float64>("brake_angle", 1000);
    lblinker_pub = n.advertise<std_msgs::Bool>("left_blinker", 1000);
    rblinker_pub = n.advertise<std_msgs::Bool>("right_blinker", 1000);
    direction_pub_ = n.advertise<std_msgs::Bool>("direction_ctrl", 1);
    ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);
    puts("Reading from Joystick");
    puts("---------------------------");
    
    ros::spin();

    return 0;
}
