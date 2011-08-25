#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#define BOUND(x,m,M) ( (x)>(M) ? (M) : (x)<(m) ? (m) : (x) )

class SteeringController
{
  public:
    SteeringController(ros::NodeHandle nh_);

  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber sampler_sub;
    ros::Publisher golfcarsteer_pub;

    double distance_threshold;
    double gc_x, gc_y, yaw_feedback;
    bool emergency;
    int point_counts;

    void emergencyCallBack(std_msgs::Bool m);
    void cmdVelCallBack(geometry_msgs::Twist cmd_vel);
};


SteeringController::SteeringController(ros::NodeHandle nh_) : nh(nh_)
{
  sub = nh.subscribe("cmd_vel", 1000, &SteeringController::cmdVelCallBack, this);
  sampler_sub = nh.subscribe("emergency", 1000, &SteeringController::emergencyCallBack, this);
  golfcarsteer_pub = nh.advertise<std_msgs::Float64>("steer_angle", 1);
  emergency = false;
}

void SteeringController::emergencyCallBack(std_msgs::Bool m)
{
  emergency = m.data;
}

void SteeringController::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
{
  double steering_angle = 0;
  if( ! emergency ) {
    double wheel_angle_deg = cmd_vel.angular.z / M_PI * 180;
    steering_angle = -0.0016 * pow(wheel_angle_deg,3);
    steering_angle -= 0.0032 * pow(wheel_angle_deg,2);
    steering_angle += 16.648 * wheel_angle_deg;
    steering_angle -= 1.3232;
    steering_angle = - BOUND( steering_angle, -540, 540 );
  }
  std_msgs::Float64 m;
  m.data = steering_angle;
  golfcarsteer_pub.publish(m);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "golfcar_localSteeringControl");
  ros::NodeHandle nh_;
  SteeringController control(nh_);
  ros::spin();
  return 0;
}
