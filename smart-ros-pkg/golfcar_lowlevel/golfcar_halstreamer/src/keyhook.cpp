#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
#include <ros/ros.h>
#include "golfcar_halstreamer/vel.h"
#include <sstream>
#include "joy/Joy.h"

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
	
	ros::Publisher chatter_pub = n.advertise<golfcar_halstreamer::vel>("golfcar_vel", 1000);
	signal(SIGINT,quit);
  char c;
  
  const float s_factor=0.05, t_factor=5;
  int turning=0, speed=8;
  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move car.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    //linear_=angular_=0;
    //ROS_DEBUG("value: 0x%02X\n", c);
  
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
    //std::stringstream ss;
    //ss << "%lf %lf",speed*s_factor, turning*t_factor << count;
	//char ss[100];
	//sprintf(ss,"%lf %lf",speed*s_factor, turning*t_factor );
	//msg.data = ss;
	golfcar_halstreamer::vel velocity;
	velocity.speed=speed*s_factor;
	//velocity.angle=turning*t_factor;
	ROS_INFO("Speed=%lf, Angle=%d", velocity.speed, (int)velocity.angle);
	
	chatter_pub.publish(velocity);
  }


  return 0;
}
