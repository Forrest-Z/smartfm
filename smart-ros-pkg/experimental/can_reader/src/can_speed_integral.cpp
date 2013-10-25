/*
 * can_speed_integral.cpp
 *
 *  Created on: Oct 6, 2013
 *      Author: Shen Xiaotong
 */

#include <ros/ros.h>
#include <iostream>
#include <can_reader/CanMsg.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <sound_play/sound_play.h>
#include <unistd.h>

#define MOTOR_SPEED_ID 0x298
#define BACK_WHEEL_SPEED_ID 0x208
#define FRONT_WHEEL_SPEED_ID 0x200
#define GEAR_SHIFT_ID 0x418


#define RADIUS  (0.265 * (10.7/10.2))
#define GEAR_RATIO  6.066

using namespace std;

const double COEFF_MOTORSPEED_ODOM = ((1.0)/GEAR_RATIO * 3.1415926 / 30.0 * RADIUS );
const double COEFF_WHEELSPEED_ODOM = (COEFF_MOTORSPEED_ODOM * 3.0);
const double COEFF_BACKWHEEL_ODOM  = (1/43.5 * 3.1415926* 2.0 * RADIUS);

// when the motor_speed > 50rpm, can use the wheel speedometer
// namely when the speed of the car is over 0.23 m/s, can use the wheel speedometer (output non-zero)

// But in order to get a better resolution(because the data is integer), when motor_speed > 300rpm,
// namely, when the car is over 1.5m/s, start to use speedometer
// in this way, the error is resolution error is bounded by 1% (the output of speedometer is about 100)

// in low speed, only motorspeed is usable, so try to publish it at a higher frequency
// try to publish and update in about 100 hz

double speed = 0.0; //global variable, update by either backwheel_speed or motor_speed
const double speed_th = 100.0;
bool reverse_flag = false;

ros::Publisher * can_move_dist_pub_;
ros::Publisher * can_move_delta_dist_pub_;

#define TIMER_INTERVAL 0.01

class counter
{
	ros::Time last_update_time_;
	ros::Duration tol_duration_;
	int count_;
	bool initial_;
	string name_;
	bool valid_;
public:
	counter(double tolerance, string name)
	{
		count_ = 0;
		initial_ = false;
		tol_duration_.fromSec(tolerance);
		name_ = name;
		valid_ = true;
	}
	bool update(void)
	{
			bool ret = true;
			count_ ++;
			if(initial_)
			{
				if((ros::Time::now() - last_update_time_) > tol_duration_)
				{
					ret = false;
					ROS_ERROR("%s is not updated at expected frequence ...\n",name_.c_str());
				}
			}
			else
			{
				initial_ = true;
			}

			last_update_time_ = ros::Time::now();
			if(valid_)valid_ = ret;
			return ret;
	}

	bool check(void)
	{
		bool ret = true;
		if(initial_)
		{
			if((ros::Time::now() - last_update_time_) > tol_duration_)
			{
				ret = false;
				ROS_ERROR("%s is not updated at expected frequence ...\n",name_.c_str());
			}
		}
		if(valid_)valid_ = ret;
		return ret;
	}

	bool isValid(void)
	{
		return valid_;
	}
};


// Just update the speed and counter here
//counter motor_counter(0.001,"can_motor_msg");
counter motor_counter(0.5,"can_motor_msg");
counter backwheel_counter(0.15,"can_backwheel_speed_msg");

void canMsgCallback(can_reader::CanMsg::ConstPtr can_msg_ptr)
{
	double motor_speed = 0.0;
	double left_backwheel_speed = 0.0;
	double right_backwheel_speed = 0.0;
	switch(can_msg_ptr->id)
	{
	case 0x298:
		motor_speed = can_msg_ptr->data[6]*256 + can_msg_ptr->data[7] - 10000;
		if(fabs(motor_speed) < 4.0) motor_speed = 0.0;
		speed = COEFF_MOTORSPEED_ODOM * motor_speed;
		motor_counter.update();
		break;
	case 0x208:
		left_backwheel_speed = (can_msg_ptr->data[4] - 192)*256 + can_msg_ptr->data[5];
		right_backwheel_speed = (can_msg_ptr->data[6] - 192)*256 + can_msg_ptr->data[7];
		if(left_backwheel_speed > speed_th && right_backwheel_speed > speed_th)
		{
			double tmp_speed = (left_backwheel_speed + right_backwheel_speed)/2.0;
			if(reverse_flag)
			{
				speed = - COEFF_WHEELSPEED_ODOM * tmp_speed;
			}
			else
			{
				speed = COEFF_WHEELSPEED_ODOM * tmp_speed;
			}
		}
		backwheel_counter.update();
		break;
	case 0x418:
		reverse_flag = (can_msg_ptr->data[0]==82);
		break;
	default:
		break;
	}
}



class numeric_integral
{
	bool initial_;
	double delta_;
	double integral_;
	ros::Time last_time_;
	ros::Duration interval_;
public:
	numeric_integral(void)
	{
		initial_ = false;
		delta_ = 0.0;
		integral_ = 0.0;
	}
	void reset(void)
	{
		initial_ = false;
		delta_ = 0.0;
		integral_ = 0.0;
	}
	void update(double derivative)
	{
		if(initial_)
		{
			interval_ = ros::Time::now() - last_time_;
			delta_ = derivative * interval_.toSec();
			integral_ += delta_;
		}
		else
		{
			initial_ = true;
		}
		last_time_ = ros::Time::now();

		if(initial_)
		{
			if(interval_.toSec() > 5.0 * TIMER_INTERVAL)
			{
				ROS_WARN("can odom, numerical intergral timer is not updated as expected");
			}
		}
	}

	double getDelta(void)
	{
		//return delta_;
		return interval_.toSec();
	}
	double getIntegral(void)
	{
		return integral_;
	}

};

numeric_integral ni_odom;
sound_play::SoundClient * sc_;
void publishTimerCallback(const ros::TimerEvent & event)
{
	static int error_count = 0;
	std_msgs::Float32 float_msg;
	// maximum speed is 20.0
	if(fabs(speed) > 20.0)
	{
		ROS_ERROR("driving too fast with speed %f m/s, not reasonable ...\n",speed);
	}

	motor_counter.check();
	backwheel_counter.check();

	if(motor_counter.isValid() && backwheel_counter.isValid())
	{
		ni_odom.update(speed);
		// publish the data here ...
		float_msg.data = ni_odom.getIntegral();
		can_move_dist_pub_->publish(float_msg);
		float_msg.data = ni_odom.getDelta();
		can_move_delta_dist_pub_->publish(float_msg);
	}
	else
	{
		error_count++;
	}

	if(error_count > 100)
	{
		ROS_ERROR("CAN BUS MSG ERROR ......\n");
		sc_->say("CAN BUS MESSAGE ERROR \n");
		error_count = 0;
	}

}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "can_speed_integral");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	sc_ = & sc;
	ros::Subscriber can_sub = nh.subscribe("can_msg",100,&canMsgCallback);
	ros::Publisher can_move_dist_pub = nh.advertise<std_msgs::Float32>("can_move_dist",20);
	ros::Publisher can_move_delta_dist_pub = nh.advertise<std_msgs::Float32>("can_move_delta_dist",20);

	ros::Timer publish_timer = nh.createTimer(ros::Duration(TIMER_INTERVAL),publishTimerCallback);

	can_move_dist_pub_ = & can_move_dist_pub;
	can_move_delta_dist_pub_ = & can_move_delta_dist_pub;

	ros::spin();
	return 0;
}
