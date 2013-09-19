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
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>


#define STEER_ANG_MAX 540
#define FULL_BRAKE 100
#define FULL_THROTTLE 1
ros::Publisher throttle_pub, steering_pub, brake_pub;//, enable_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_simulator");
    ros::NodeHandle n;
    throttle_pub = n.advertise<std_msgs::Float32>("throttle", 10);
    steering_pub = n.advertise<std_msgs::Float32>("steering_angle", 10);
    brake_pub = n.advertise<std_msgs::Float32>("brake_angle", 10);
    
    ros::Rate r(100);
    ros::NodeHandle priv_nh("~");
    double steer_frequency, throttle_frequency;
    priv_nh.param("steer_frequency", steer_frequency, 10.0);
    priv_nh.param("throttle_frequency", throttle_frequency, 50.0);
    while(ros::ok()){
      double steer_signal = sin(2*M_PI*steer_frequency*ros::Time::now().toSec())*STEER_ANG_MAX;
      double throttle_signal = sin(2*M_PI*throttle_frequency*ros::Time::now().toSec());
      double final_throttle, brake;
      if(throttle_signal>0){
	final_throttle = throttle_signal;
	brake = 0;
      }
      else{
	final_throttle = 0;
	brake = -throttle_signal*FULL_BRAKE;
      }
      std_msgs::Float32 steer_msg, throttle_msg, brake_msg;
      steer_msg.data = steer_signal;
      throttle_msg.data = final_throttle;
      brake_msg.data = brake;
      throttle_pub.publish(throttle_msg);
      steering_pub.publish(steer_msg);
      brake_pub.publish(brake_msg);
      ros::spinOnce();
      r.sleep();
    }
	
    return 0;
}
