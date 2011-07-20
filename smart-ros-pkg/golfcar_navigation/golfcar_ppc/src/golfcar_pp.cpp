#include <golfcar_ppc/golfcar_pp.h>

namespace golfcar_purepursuit {

  PurePursuit::PurePursuit()
  {
    ros::NodeHandle n;
    traj_sub_ = n.subscribe("pnc_trajectory", 100, &PurePursuit::trajCallBack, this);
    cmd_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    timer_ = n.createTimer(ros::Duration(0.08), &PurePursuit::controlLoop, this);

    ros::NodeHandle private_nh("~");
    if(!private_nh.getParam("normal_speed",normal_speed_)) normal_speed_ = 1.5;
    if(!private_nh.getParam("slow_speed",slow_speed_)) slow_speed_ = 1.0;
    if(!private_nh.getParam("start_decreasing",start_decreasing_)) start_decreasing_ = 20;
    if(!private_nh.getParam("turning_radius",turning_radius_)) turning_radius_ = 4;
    if(!private_nh.getParam("look_ahead",look_ahead_)) look_ahead_ = 3;
    if(!private_nh.getParam("look_ahead_bad",look_ahead_bad_)) look_ahead_bad_ = 1;
    if(!private_nh.getParam("max_steering",max_steering_)) max_steering_ = 0.65;
    if(!private_nh.getParam("switch_distance",switch_distance_)) switch_distance_ = 0.01;
    if(!private_nh.getParam("car_length",car_length_)) car_length_ = 1.632;

    last_segment_ = 0;

    std::cout<<"normal_speed: "<<normal_speed_<<"\n";
    std::cout<<"slow_speed: "<<slow_speed_<<"\n";
    std::cout<<"start_decreasing: "<<start_decreasing_<<"\n";
    std::cout<<"turning_radius: "<<turning_radius_<<"\n";
    std::cout<<"look_ahead: "<<look_ahead_<<"\n";
    std::cout<<"look_ahead_bad: "<<look_ahead_bad_<<"\n";
    std::cout<<"max_steering: "<<max_steering_<<"\n";
    std::cout<<"switch_distance: "<<switch_distance_<<"\n";
    std::cout<<"car_length: "<<car_length_<<"\n";
  }

  PurePursuit::~PurePursuit()
  {
  }

  void PurePursuit::trajCallBack(const nav_msgs::Path::ConstPtr &traj)
  {
    last_segment_ = 0;

    trajectory_.header.frame_id = "/odom";
    trajectory_.header.stamp = ros::Time::now();
    trajectory_.poses.resize(traj->poses.size());
    for(unsigned int i=0; i<traj->poses.size(); i++)
    {
      try {
	tf_.transformPose("/odom", traj->poses[i], trajectory_.poses[i]);
      }
      catch(tf::LookupException& ex) {
	ROS_ERROR("No Transform available Error: %s\n", ex.what());
	return;
      }
      catch(tf::ConnectivityException& ex) {
	ROS_ERROR("Connectivity Error: %s\n", ex.what());
	return;
      }
      catch(tf::ExtrapolationException& ex) {
	ROS_ERROR("Extrapolation Error: %s\n", ex.what());
	return;
      }
    }
  }

  void PurePursuit::controlLoop(const ros::TimerEvent &e)
  {	
    geometry_msgs::Twist cmd_ctrl;

    double cmd_vel;
    if((int) trajectory_.poses.size() > start_decreasing_)
      cmd_vel = normal_speed_;
    else
      cmd_vel = normal_speed_ * start_decreasing_/(1+start_decreasing_);

    double cmd_steer = 0;
    tf::Stamped<tf::Pose> pose;
    if(trajectory_.poses.size() > 1 && getRobotPose(pose))
    {
      double cur_x = pose.getOrigin().x();
      double cur_y = pose.getOrigin().y();
      double cur_yaw = tf::getYaw(pose.getRotation());
      int segment = get_segment(cur_x, cur_y);
      cmd_steer = get_steering(segment, cur_x, cur_y, cur_yaw, cmd_vel);
    }
    else
    {
      cmd_vel = 0;
      cmd_steer = 0;
    }

    cmd_ctrl.linear.x = cmd_vel;
    cmd_ctrl.angular.z = cmd_steer;
    cmd_pub_.publish(cmd_ctrl);
  }

  bool PurePursuit::getRobotPose(tf::Stamped<tf::Pose>& odom_pose) const 
  {
    odom_pose.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "/base_link";
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    try {
      tf_.transformPose("/odom", robot_pose, odom_pose);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      return false;
    }
    // check odom_pose timeout
    if (current_time.toSec() - odom_pose.stamp_.toSec() > 0.1) {
      ROS_WARN("PurePursuit transform timeout. Current time: %.4f, odom_pose stamp: %.4f, tolerance: %.4f",
	       current_time.toSec(), odom_pose.stamp_.toSec(), 0.1);
      return false;
    }

    return true;
  }

  double PurePursuit::get_distance(double x1, double y1, double x2, double y2)
  {
    return (sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)));
  }

  bool PurePursuit::get_center(double tar_x, double tar_y,
			       double ori_x, double ori_y, double inv_R,
			       double center[2])
  {
    double a, b, x, y, dl, dx, dy, dist1, dist2;
    x = (tar_x + ori_x)/2;
    y = (tar_y + ori_y)/2;
    dist1 = get_distance(ori_x, ori_y, tar_x, tar_y);
    double sq = 1/inv_R/inv_R - dist1*dist1/4;
    if(sq < 0) {
      ROS_ERROR("R*R-dist*dist/4=%lf < 0\n", sq);
      return false;
    }

    if(inv_R > 0)
      dist2 = sqrt(sq);
    else
      dist2 = -sqrt(sq);
    a = tar_y - ori_y;
    b = ori_x - tar_x;

    dl = sqrt(a*a + b*b);
    dx = -a*dist2/dl;
    dy = -b*dist2/dl;

    center[0] = x + dx;
    center[1] = y + dy;

    return true;
  }

  double PurePursuit::get_inv_R(int segment)
  {
    double inv_R = 0;
    double ori_yaw = trajectory_.poses[segment].pose.position.z;
    double tar_yaw = trajectory_.poses[segment+1].pose.position.z;
    double diff_yaw_abs = abs(tar_yaw-ori_yaw);
    if(diff_yaw_abs > M_PI/2) // hack to consider wrapping and sign
    {
      if(tar_yaw < ori_yaw)
	inv_R = 1/turning_radius_;
      else if(tar_yaw > ori_yaw)
	inv_R = -1/turning_radius_;
      else
	inv_R = 0;
    }
    else
    {
      if(tar_yaw > ori_yaw)
	inv_R = 1/turning_radius_;
      else if(tar_yaw < ori_yaw)
	inv_R = -1/turning_radius_;
      else
	inv_R = 0;
    }

    return inv_R;
  }

  bool PurePursuit::btwn_points(double tar_x, double tar_y,
				double ori_x, double ori_y,
				double inv_R, double x, double y)
  {
    if(inv_R == 0)
    {
      double a, b, c;
      a = tar_x - ori_x;
      b = tar_y - ori_y;
      c = (ori_x - tar_x)*x + (ori_y - tar_y)*y;
      if((a*tar_x + b*tar_y + c)*(a*ori_x + b*ori_y + c) > 0)
	return false;
      else
	return true;
    }
    else
    {
      double center[2];
      if(!get_center(tar_x, tar_y, ori_x, ori_y, inv_R, center))
	return btwn_points(tar_x, tar_y, ori_x, ori_y, 0, x, y);

      double arg1, arg2, arg;
      arg1 = atan2(ori_y - center[1], ori_x - center[0]);
      arg2 = atan2(tar_y - center[1], tar_x - center[0]);
      arg = atan2(y - center[1], x - center[0]);
      if(inv_R > 0)
      {
	while(arg2 < arg1)
	  arg2 += 2*M_PI;
	while(arg < arg1)
	  arg += 2*M_PI;
	if ( arg > arg2 )
	  return false;
	else
	  return true;
      }
      else
      {
	while(arg2 > arg1)
	  arg2 -= 2*M_PI;
	while(arg > arg1)
	  arg -= 2*M_PI;
	if ( arg < arg2 )
	  return false;
	else
	  return true;
      }
    }
  }

  void PurePursuit::get_projection(double tar_x, double tar_y,
				   double ori_x, double ori_y,
				   double inv_R, double cur_x, double cur_y,
				   double proj[2])
  {
    if(inv_R == 0)
    {
      double a, b, c, x, dl, dx, dy;
      a = tar_y - ori_y;
      b = ori_x - tar_x;
      c = (tar_x - ori_x)*ori_y + ori_x*(ori_y - tar_y);
      x = (a*cur_x + b*cur_y + c)/sqrt(a*a + b*b);

      dl = sqrt(a*a + b*b);
      dx = -a*x/dl;
      dy = -b*x/dl;

      proj[0] = cur_x + dx;
      proj[1] = cur_y + dy;
    }
    else
    {
      double center[2];
      if(!get_center(tar_x, tar_y, ori_x, ori_y, inv_R, center))
	return get_projection(tar_x, tar_y, ori_x, ori_y, 0, cur_x, cur_y, proj);

      double dist = get_distance(center[0], center[1], cur_x, cur_y);

      proj[0] = center[0] + (cur_x - center[0])/dist/abs(inv_R);
      proj[1] = center[1] + (cur_y - center[1])/dist/abs(inv_R);
    }
  }

  int PurePursuit::get_segment(double cur_x, double cur_y)
  {
    double tar_x, tar_y, ori_x, ori_y, inv_R;
    double prj[2];

    if(trajectory_.poses.size() < 1)
      return -1;
    else if(trajectory_.poses.size() < 2)
      return 0;

    int segment = last_segment_;
    bool bContinue;
    do
    {
      bContinue = false;

      tar_x = trajectory_.poses[segment+1].pose.position.x;
      tar_y = trajectory_.poses[segment+1].pose.position.y;
      ori_x = trajectory_.poses[segment].pose.position.x;
      ori_y = trajectory_.poses[segment].pose.position.y;
      inv_R = get_inv_R(segment);

      get_projection(tar_x, tar_y, ori_x, ori_y, inv_R, cur_x, cur_y, prj);
      if(!btwn_points(tar_x, tar_y, ori_x, ori_y, inv_R, prj[0], prj[1]))
      {
	if(get_distance(tar_x, tar_y, prj[0], prj[1]) < get_distance(ori_x, ori_y, prj[0], prj[1]))
	{
	  segment++;
	  if(segment+1 < (int) trajectory_.poses.size())
	    bContinue = true;
	  else
	  {
	    segment = -1;
	    bContinue = false;
	  }
	}
      }
      else if(get_distance(tar_x, tar_y, prj[0], prj[1]) < switch_distance_)
      {
	segment++;
	if(segment+1 < (int) trajectory_.poses.size())
	  bContinue = true;
	else
	{
	  segment = -1;
	  bContinue = false;
	}
      }
    } while(bContinue);

    last_segment_ = segment;
    return segment;
  }

  double PurePursuit::get_steering(int segment, double cur_x, double cur_y,
				   double cur_yaw, double& cmd_vel)
  {
    if(segment < 0)
    {
      ROS_DEBUG("steering, segment=-1");
      cmd_vel = 0;
      return 0;
    }
    else if((int) trajectory_.poses.size() < segment+1)
    {
      ROS_WARN("Something seriously wrong!");
      cmd_vel = 0;
      return 0;
    }

    double tar_x = trajectory_.poses[segment+1].pose.position.x;
    double tar_y = trajectory_.poses[segment+1].pose.position.y;
    double ori_x = trajectory_.poses[segment].pose.position.x;
    double ori_y = trajectory_.poses[segment].pose.position.y;
    double inv_R = get_inv_R(segment);
	
    double a, b, c, x, r, theta, gamma;
    double L = look_ahead_;

    if(inv_R == 0)
    {
      a = tar_y - ori_y;
      b = ori_x - tar_x;
      c = (tar_x - ori_x)*ori_y + ori_x*(ori_y - tar_y);
      r = 0;
      x = (a*cur_x + b*cur_y + c)/sqrt(a*a + b*b);
      theta = cur_yaw - atan2(tar_y - ori_y, tar_x - ori_x);
    }
    else
    {
      double center[2];
      if(!get_center(tar_x, tar_y, ori_x, ori_y, inv_R, center))
      {
	a = tar_y - ori_y;
	b = ori_x - tar_x;
	c = (tar_x - ori_x)*ori_y + ori_x*(ori_y - tar_y);
	r = 0;
	x = (a*cur_x + b*cur_y + c)/sqrt(a*a + b*b);
	theta = cur_yaw - atan2(tar_y - ori_y, tar_x - ori_x);
      }
      else
      {
	if(inv_R > 0)
	{
	  r = get_distance(center[0], center[1], cur_x, cur_y) - abs(1/inv_R);
	  x = (inv_R*(r*r + L*L) + 2*r)/(2*(1 + inv_R*r));
	  theta = cur_yaw - atan2(cur_x - center[0], center[1] - cur_y);
	}
	else
	{
	  r = -get_distance(center[0], center[1], cur_x, cur_y) + abs(1/inv_R);
	  x = (inv_R*(r*r + L*L) + 2*r)/(2*(1 + inv_R*r));
	  theta = cur_yaw - atan2(-cur_x + center[0], cur_y - center[1]);
	}
      }
    }
    while(theta > M_PI)
      theta -= 2*M_PI;
    while(theta < -M_PI)
      theta += 2*M_PI;

    if(abs(x) > L) // too off from the path
    {
      if(cmd_vel > slow_speed_)
	cmd_vel = slow_speed_;
      L = abs(x) + look_ahead_bad_;
    }

    gamma = 2/(L*L)*(x*cos(theta) - sqrt(L*L - x*x)*sin(theta));
    double steering = atan(gamma * car_length_);
    ROS_DEBUG("steering, segment=%d x=%lf y=%lf yaw=%lf cmd_vel=%lf", segment, cur_x, cur_y, cur_yaw, cmd_vel);
    ROS_DEBUG("inv_R=%lf L=%lf r=%lf x=%lf theta=%lf gamma=%lf steering=%lf", inv_R, L, r, x, theta, gamma, steering);

    if(steering > max_steering_)
      steering = max_steering_;
    else if(steering < -max_steering_)
      steering = -max_steering_;

    return steering;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "golfcar_pp");
  golfcar_purepursuit::PurePursuit *pp = new golfcar_purepursuit::PurePursuit();
  if(!pp) {
    ROS_ERROR("failed to start the process\n");
    return 1;
  }
  
  ros::spin();
  
  return 0;
}	
