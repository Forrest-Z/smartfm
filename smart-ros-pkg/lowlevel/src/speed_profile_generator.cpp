#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
/**
 * Simple trapezoidal speed profile generation
 */
int state = 0;
float rate = 0.01;
float acceleration=1;
float speed=0;
float max_speed = 2;
float constant_time = 3; 
float cool_period = 2;
int count=0;
ros::Publisher speed_pub;
bool use_sim_time_=false;
void callback(const ros::TimerEvent&)
{
  switch (state)
  {
	//cool
	case 0:
		speed = 0;
		count++;
		if(count>=cool_period/rate) {state++;count=0;}
		break;
	//ramp up
	case 1:
		speed+=acceleration*rate;
		if(speed>=max_speed) state++;
		break;
	//constant
	case 2:
		speed = max_speed;
		count++;
		if(count>=constant_time/rate) state++;
		break;
	//ramp down
	case 3:
		speed-=acceleration*rate;
		if(speed<0) exit(0);
		break;
	}
	geometry_msgs::Twist tw;
	tw.linear.x = speed;
	speed_pub.publish(tw);
		
  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_profile");
    ros::NodeHandle n;
    n.param("use_sim_time", use_sim_time_, false);
    ROS_INFO_STREAM("Simulated time is "<<use_sim_time_);
    /**
    * Timers allow you to get a callback at a specified rate.  Here we create
    * two timers at different rates as a demonstration.
    */
    speed_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Timer timer = n.createTimer(ros::Duration(rate), callback);

    ros::spin();

    return 0;
}
