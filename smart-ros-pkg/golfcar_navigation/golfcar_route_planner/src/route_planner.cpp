#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
using namespace std;

namespace route_planner {

	class RoutePlanner
    {
    public:
		RoutePlanner();
		~RoutePlanner();

    private:
	ros::Publisher waypoint_pub_;
	ros::Publisher g_plan_pub_;
	ros::Publisher pointCloud_pub_;
        ros::Timer timer_;
	tf::TransformListener tf_;
	void waypoint_pub_loop(const ros::TimerEvent &e);
	bool getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const;
	
	std::vector<geometry_msgs::Point> targets_;
	int WaypointNo_;
	};
};

namespace route_planner {

RoutePlanner::RoutePlanner()
{
	WaypointNo_=0;
	ros::NodeHandle n;
	waypoint_pub_ = n.advertise<geometry_msgs::PointStamped>("pnc_waypoint", 1);
	g_plan_pub_ = n.advertise<nav_msgs::Path>("pnc_globalplan", 1);
	pointCloud_pub_ = n.advertise<sensor_msgs::PointCloud>("pnc_waypointVis",1);
	timer_ = n.createTimer(ros::Duration(0.2), &RoutePlanner::waypoint_pub_loop, this);

	//setup the points
	
	geometry_msgs::Point target;
	std::vector<geometry_msgs::Point> targets;
	double res = 0.1;
	target.x = 1077; target.y = 2158; targets.push_back(target);
	target.x = 1067; target.y = 2272; targets.push_back(target);
	
	target.x = 1443; target.y = 3056; targets.push_back(target);
	target.x = 1473; target.y = 3137; targets.push_back(target);
	target.x = 1432; target.y = 3061; targets.push_back(target);
	target.x = 1093; target.y = 2349; targets.push_back(target);
	target.x = 885; target.y = 2023; targets.push_back(target);
	target.x = 829; target.y = 2003; targets.push_back(target);
	target.x = 769; target.y = 1998; targets.push_back(target);
	target.x = 507; target.y = 2271; targets.push_back(target);
	target.x = 469; target.y = 2318; targets.push_back(target);
	target.x = 412; target.y = 2334; targets.push_back(target);
	target.x = 363; target.y = 2347; targets.push_back(target);
	
	target.x = 324; target.y = 2331; targets.push_back(target);
	target.x = 302; target.y = 2285; targets.push_back(target);
	target.x = 300; target.y = 2216; targets.push_back(target);
	target.x = 781; target.y = 245; targets.push_back(target);
	target.x = 837; target.y = 213; targets.push_back(target);
	target.x = 894; target.y = 217; targets.push_back(target);
	
	target.x = 1994; target.y = 612; targets.push_back(target);
	target.x = 2010; target.y = 638; targets.push_back(target);
	
	target.x = 1881; target.y = 667; targets.push_back(target);
	
	target.x = 1833; target.y = 650; targets.push_back(target);
	
	target.x = 1415; target.y = 434; targets.push_back(target);
	target.x = 888; target.y = 242; targets.push_back(target);
	target.x = 841; target.y = 238; targets.push_back(target);
	
	target.x = 497; target.y = 912; targets.push_back(target);
	target.x = 410; target.y = 1175; targets.push_back(target);
	target.x = 399; target.y = 1229; targets.push_back(target);
	
	target.x = 431; target.y = 2286; targets.push_back(target);
	target.x = 481; target.y = 2244; targets.push_back(target);
	
	target.x = 663; target.y = 2010; targets.push_back(target);
	target.x = 720; target.y = 1986; targets.push_back(target);
	
	target.x = 943; target.y = 2044; targets.push_back(target);
	target.x = 983; target.y = 2099; targets.push_back(target);
	
	target.x = 1077; target.y = 2159; targets.push_back(target);
	
	
	std::vector<geometry_msgs::PoseStamped> final_targets;
	final_targets.resize(targets.size());
	targets_.resize(targets.size());
	for(int i=0; i<targets.size(); i++)
	{
		final_targets[i].header.frame_id = "/map";
		final_targets[i].header.stamp = ros::Time::now();
		
		targets_[i].x = final_targets[i].pose.position.x = targets[i].x * res;
		targets_[i].y = final_targets[i].pose.position.y = (3536 - targets[i].y) * res;
	}
	nav_msgs::Path p;
	p.poses.resize(final_targets.size());
	for(int i=0; i<final_targets.size(); i++)
	{
		p.poses[i] = final_targets[i] ;
	}
	p.header.stamp = ros::Time();
	p.header.frame_id = "/map";
	
	//p.poses.push_back(start);
	//p.poses.push_back(goal);
	g_plan_pub_.publish(p);
}

RoutePlanner::~RoutePlanner()
{

}

void RoutePlanner::waypoint_pub_loop(const ros::TimerEvent &e)
{
	//get global pose
	tf::Stamped<tf::Pose> global_pose;
	RoutePlanner::getRobotGlobalPose(global_pose);

	geometry_msgs::PointStamped map_point;
	geometry_msgs::PointStamped odom_point;
	map_point.header.frame_id = "/map";
	map_point.header.stamp = ros::Time();
	map_point.point = targets_[WaypointNo_];

	try {
      tf_.transformPoint("/odom", map_point, odom_point);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    }
	//publish the first waypoint in odom frame then continue to send the points until the last one
	waypoint_pub_.publish(odom_point);
	sensor_msgs::PointCloud pc;
	pc.header.stamp = ros::Time::now();
	pc.header.frame_id = "/odom";
	pc.points.resize(1);
	pc.points[0].x = odom_point.point.x;
	pc.points[0].y = odom_point.point.y;
	pointCloud_pub_.publish(pc);
	//get how near it is to the goal point, if reaches the threshold, send the next point
	double mapx = map_point.point.x, mapy = map_point.point.y;
	double robotx = global_pose.getOrigin().x(), roboty = global_pose.getOrigin().y();
	double distance = sqrt((mapx-robotx)*(mapx-robotx)+(mapy-roboty)*(mapy-roboty));
	if(distance < 0.5) WaypointNo_++;
	if(WaypointNo_==targets_.size()) WaypointNo_ = targets_.size() -1;
}

bool RoutePlanner::getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const 
  {
    odom_pose.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "/base_link";
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    try {
      tf_.transformPose("/map", robot_pose, odom_pose);
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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "golfcar_route_planner");
  route_planner::RoutePlanner *rp = new route_planner::RoutePlanner();
  if(!rp) {
    ROS_ERROR("failed to start the process\n");
    return 1;
  }
  
  ros::spin();
  
  return 0;
}	
