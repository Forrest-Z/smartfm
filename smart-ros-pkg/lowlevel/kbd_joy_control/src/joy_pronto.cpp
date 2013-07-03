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
ros::Publisher throttle_pub, steering_pub, brake_pub, motion_pub, enable_pub, start_pub, shift_pub;

//For enable button
float g_enable_state, g_enable_butt_prev;
//For start button
float g_start_state, g_start_butt_prev;
//For gear
float g_shift_state;

void joyCallBack(sensor_msgs::Joy joy_msg)
{
    // 3 axes joystick, each axis value ranges from -1.0 to 1.0
    // axes[0] is the lateral axis: positive to the left
    // axes[1] is the longitudinal axis: positive to the front
    // axes[2] is the wheel axis: positive upward.

    float lat = joy_msg.axes[0];
    float lon = joy_msg.axes[4];
    float motion = joy_msg.axes[2];
    float enable_butt = joy_msg.buttons[6];
    float start_butt = joy_msg.buttons[7];
    float shift_lat = joy_msg.axes[6];
    float shift_lon = joy_msg.axes[7];
    
    ROS_INFO("Joystick: axes[0] (lat)=%f, axis[1] (lon)=%f, axis[2] (wheel)=%f",
    joy_msg.axes[0], joy_msg.axes[1], joy_msg.axes[2]);

    std_msgs::Int32 V, B;
    V.data = lon>0 ? lon*FULL_THROTTLE : 0;
    B.data = lon>0 ? 0 : -lon*FULL_BRAKE;
    throttle_pub.publish(V);
    brake_pub.publish(B);

    std_msgs::Float64 W;
    W.data = lat * STEER_ANG_MAX;
    steering_pub.publish(W);

    //Motion
    std_msgs::Int32 M;
    if(motion < 0.5) {
      M.data = 1;
    }else{
      M.data = 0;
    }
    motion_pub.publish(M);
    
    //Enable button
    std_msgs::Int32 En;
    if((enable_butt == 1) && (g_enable_butt_prev == 0)){
      if(g_enable_state == 0){
	g_enable_state = 1;
      }else{
	g_enable_state = 0;
      }
    }
    En.data = g_enable_state;
    g_enable_butt_prev = enable_butt;
    enable_pub.publish(En);
    
    //Start button
    std_msgs::Int32 St;
    if((start_butt == 1) && (g_start_butt_prev == 0)){
      if(g_start_state == 0){
	g_start_state = 1;
      }else{
	g_start_state = 0;
      }
    }
    St.data = g_start_state;
    g_start_butt_prev = start_butt;
    start_pub.publish(St);
    
    //Shift up-down
    std_msgs::Int32 Sh;
    if((lon <= -0.8) && ((shift_lat != 0) || (shift_lon != 0))){
      if(shift_lat == 1.0) g_shift_state = 0;
      if(shift_lon == 1.0) g_shift_state = 1;
      if(shift_lat == -1.0) g_shift_state = 2;
      if(shift_lon == -1.0) g_shift_state = 3;
    }
    Sh.data = g_shift_state;
    shift_pub.publish(Sh);
    
    ROS_INFO("Resulting commands: throttle=%d, brake_angle=%d, steer_angle=%f",
    V.data, B.data, W.data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_drive");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("joy", 1000, joyCallBack);

    throttle_pub = n.advertise<std_msgs::Int32>("throttle", 1000);
    steering_pub = n.advertise<std_msgs::Float64>("steer_angle", 1000);
    brake_pub = n.advertise<std_msgs::Int32>("brake_angle", 1000);
    motion_pub = n.advertise<std_msgs::Int32>("motion_en", 1000);
    enable_pub = n.advertise<std_msgs::Int32>("enable_sig", 1000);
    start_pub = n.advertise<std_msgs::Int32>("start_sig", 1000);
    shift_pub = n.advertise<std_msgs::Int32>("shift", 1000);
    
    //Initial state
    g_enable_state = 0.0;
    g_start_state = 0.0;
    g_shift_state = 0.0;
    
    puts("Reading from Joystick");
    puts("---------------------------");

    ros::spin();

    return 0;
}
