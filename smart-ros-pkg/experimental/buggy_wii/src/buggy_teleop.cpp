/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * Copyright (c) 2013, Karl Damkjær Hansen, Singapore MIT Alliance for Research and Technology (SMART).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class BuggyTeleop
{
public:
  BuggyTeleop();

private:
  void joy1Callback(const sensor_msgs::Joy::ConstPtr& joy);
  void joy2Callback(const sensor_msgs::Joy::ConstPtr& joy);
  void enableSignalCallback(const std_msgs::Bool::ConstPtr&);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_axis_, full_left_axis_, full_right_axis_;
  int f_right_, f_left_;
  int reverse_b_;
  double t_scale_, b_scale_, b_default_, a_scale_, f_steer_, sensitivity_, pub_period_;

  ros::Publisher throttle_pub_;
  ros::Publisher braking_pub_;
  ros::Publisher steering_pub_;
  ros::Publisher direction_pub_;
  ros::Subscriber joy1_sub_, joy2_sub_;
  ros::Subscriber enable_sub_;

  std_msgs::Float32 last_throttle_published_;
  std_msgs::Float32 last_brake_published_;
  std_msgs::Float32 last_steering_published_;
  std_msgs::Bool last_reverse_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool enabled_;
  ros::Timer timer_;
  double pos_joy_range_, neg_joy_range_;
};

BuggyTeleop::BuggyTeleop():
  ph_("~"),
  linear_(1),
  angular_(2),
  deadman_axis_(0),
  full_left_axis_(6),
  full_right_axis_(7),
  t_scale_(1),
  b_scale_(1),
  b_default_(-120.0),
  a_scale_(1),
  f_right_(0),
  f_left_(0),
  f_steer_(360),
  reverse_b_(0),
  sensitivity_(1.0),
  pub_period_(0.05),
  enabled_(false)
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("button_deadman", deadman_axis_, deadman_axis_);
  ph_.param("button_full_right", full_right_axis_, full_right_axis_);
  ph_.param("button_full_left", full_left_axis_, full_left_axis_);
  ph_.param("scale_steering", a_scale_, a_scale_);
  ph_.param("scale_throttle", t_scale_, t_scale_);
  ph_.param("scale_braking", b_scale_, b_scale_);
  ph_.param("default_braking", b_default_, b_default_);
  ph_.param("full_steer", f_steer_, f_steer_);
  ph_.param("button_reverse", reverse_b_, reverse_b_);
  ph_.param("steering_sensitivity", sensitivity_, 1.0);
  ph_.param("neg_joy_range", neg_joy_range_, -0.85);
  ph_.param("pos_joy_range", pos_joy_range_, 1.0); 
  ph_.param("publish_period", pub_period_, pub_period_);
  throttle_pub_ = ph_.advertise<std_msgs::Float64>("throttle", 1);
  braking_pub_ = ph_.advertise<std_msgs::Float64>("brake_angle", 1);
  steering_pub_ = ph_.advertise<std_msgs::Float64>("steer_angle", 1);
  direction_pub_ = ph_.advertise<std_msgs::Bool>("direction_ctrl", 1);
  joy1_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy1", 10, &BuggyTeleop::joy1Callback, this);
  joy2_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy2", 10, &BuggyTeleop::joy2Callback, this);
  enable_sub_ = nh_.subscribe<std_msgs::Bool>("remote_enable", 1, &BuggyTeleop::enableSignalCallback, this);

  timer_ = nh_.createTimer(ros::Duration(pub_period_), boost::bind(&BuggyTeleop::publish, this));
}

void BuggyTeleop::enableSignalCallback(const std_msgs::Bool::ConstPtr& en)
{ 
	if(enabled_ != en->data) {
      	enabled_ = en->data;
        if(enabled_) std::cout<<"Enabling Signal"<<std::endl;
    	else std::cout<<"Disabling Signal"<<std::endl;
    }
}

void BuggyTeleop::joy2Callback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  std_msgs::Float32 throttle;
  std_msgs::Float32 braking;
  std_msgs::Float32 steering;
  std_msgs::Bool reverse;

  if (joy->axes[linear_] > 0)
  {
    throttle.data = t_scale_ * joy->axes[linear_];
    braking.data = 0;
  }
  else
  {
    throttle.data = 0;
    braking.data = b_scale_ * joy->axes[linear_];
  }
  double joy_steer = joy->axes[angular_];
  (joy_steer > 0) ? joy_steer/=pos_joy_range_ : joy_steer/=neg_joy_range_;
  int joy_sign = (joy_steer < 0) ? -1 : 1;
  joy_steer = fabs(joy_steer);
  if(joy_steer > 1) joy_steer = 1.0;

  //sensitivity_ need a non-zero value
  if(sensitivity_ <= 0) sensitivity_ = 0.001;

  double steer_expo_cmd = (exp(sensitivity_*joy_steer)-1)/(exp(sensitivity_)-1);
//  std::cout<<"Sign: "<<joy_sign<<" joy_steer: "<<joy_steer<<" steer_expo_cmd: "<<steer_expo_cmd<<std::endl;
  steering.data = a_scale_ * joy_sign * steer_expo_cmd;
  
  reverse.data = joy->buttons[reverse_b_];

  last_throttle_published_ = throttle;
  last_brake_published_ = braking;
  last_steering_published_ = steering;
  last_reverse_published_ = reverse;
}

void BuggyTeleop::joy1Callback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  f_right_ = joy->buttons[full_right_axis_];
  f_left_ = joy->buttons[full_left_axis_];
  deadman_pressed_ = joy->buttons[deadman_axis_];
}

void BuggyTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  //std::cout<<enabled_<<" "<<deadman_pressed_<<" "<<last_throttle_published_.data<<" "<<
  //last_brake_published_.data<<" "<<last_steering_published_.data<<std::endl;
  if (enabled_)
  {
	  if (deadman_pressed_)
	  {
		throttle_pub_.publish(last_throttle_published_);
		braking_pub_.publish(last_brake_published_);
		direction_pub_.publish(last_reverse_published_);
		if (f_left_)
		{
		  last_steering_published_.data = -f_steer_;
		}
		else if (f_right_)
		{
		  last_steering_published_.data = f_steer_;
		}
		steering_pub_.publish(last_steering_published_);
	  }
	  else
	  {
		std_msgs::Float32 zero;
		zero.data = 0.0;
		std_msgs::Float32 full_brake;
		full_brake.data = b_default_;

		throttle_pub_.publish(zero);
		braking_pub_.publish(full_brake);
		steering_pub_.publish(zero);
		
		std_msgs::Bool forward;
		forward.data = false;
		direction_pub_.publish(forward);
	  }
  }
  else
  {
	std_msgs::Float32 zero;
	zero.data = 0.0;
	throttle_pub_.publish(zero);
	braking_pub_.publish(zero);
	steering_pub_.publish(zero);
	std_msgs::Bool forward;
	forward.data = false;
	direction_pub_.publish(forward);
  }
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "buggy_teleop");
  BuggyTeleop buggy_teleop;

  ros::spin();
}
