/*
Reads the keyboard, translates it into velocity commands, and sends
it to the golfcar_vel channel.
*/

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyhook");
    ros::NodeHandle n;
    ros::Publisher throttle_pub, steering_pub, brake_pub, lblinker_pub, rblinker_pub;

    throttle_pub = n.advertise<std_msgs::Float64>("throttle", 1000);
    steering_pub = n.advertise<std_msgs::Float64>("steer_angle", 1000);
    brake_pub = n.advertise<std_msgs::Float64>("brake_angle", 1000);
    lblinker_pub = n.advertise<std_msgs::Bool>("left_blinker", 1000);
    rblinker_pub = n.advertise<std_msgs::Bool>("right_blinker", 1000);

    // get the console in raw mode
    signal(SIGINT,quit);
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move car.");


    char c;
    int turning=0, speed=0;

    for(;;)
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        switch(c)
        {
        case KEYCODE_L:
            //ROS_DEBUG("LEFT");
            turning--;
            break;
        case KEYCODE_R:
            //ROS_DEBUG("RIGHT");
            turning++;
            break;
        case KEYCODE_U:
            //ROS_DEBUG("UP");
            speed++;
            break;
        case KEYCODE_D:
            // ROS_DEBUG("DOWN");
            speed--;
            break;
        case KEYCODE_Q:
            speed=0;
            turning=0;
            break;
        }

        std_msgs::Float64 throtte_msg, brake_msg, steer_msg;
        std_msgs::Bool lblinker_msg, rblinker_msg;

        if( speed<0 ) {
            brake_msg.data = -70 + speed * 10;
            throtte_msg.data = 0;
        }
        else {
            brake_msg.data = 0;
            throtte_msg.data = speed * 0.1;
        }

        if( turning > 0 ) {
            rblinker_msg.data = true;
            lblinker_msg.data = false;
        } else if( turning < 0 ) {
            rblinker_msg.data = false;
            lblinker_msg.data = true;
        } else {
            rblinker_msg.data = false;
            lblinker_msg.data = false;
        }
        steer_msg.data = turning * 5;

        throttle_pub.publish(throtte_msg);
        brake_pub.publish(brake_msg);
        steering_pub.publish(steer_msg);
        rblinker_pub.publish(rblinker_msg);
        lblinker_pub.publish(lblinker_msg);
    }


    return 0;
}
