#include <pluginlib/class_list_macros.h>
#include <math.h> 
#include <golfcar_gp/gplan_scheduler.h>
#include <geometry_msgs/Pose2D.h>

PLUGINLIB_DECLARE_CLASS(golfcar_gp, GlobalPlan, golfcar_gp::GlobalPlan, nav_core::BaseGlobalPlanner)
using namespace std;

namespace golfcar_gp{
	
	void GlobalPlan::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		ros::NodeHandle global_node;
		g_plan_pub_ = global_node.advertise<nav_msgs::Path>("global_plan", 1);
		direction_pub_ = global_node.advertise<geometry_msgs::Point32>("direction", 1);

		ROS_INFO("gplan initialized");
	}
	
	bool GlobalPlan::makePlan(const geometry_msgs::PoseStamped&  start,	const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) 
	{
		
		//all in according to the pixels from global map
		geometry_msgs::Pose2D target;
		vector<geometry_msgs::Point> station_path;
		sp_.getPath((int)goal.pose.position.x, (int)goal.pose.position.y, station_path);

		vector<geometry_msgs::PoseStamped> final_targets;
		geometry_msgs::PoseStamped ps;

		nav_msgs::Path p;
		p.poses.resize(station_path.size());
		plan.resize(station_path.size());
		for(unsigned int i=0; i<station_path.size(); i++)
		{
			p.poses[i].header.frame_id = "/map";
			p.poses[i].header.stamp = ros::Time::now();
			p.poses[i].pose.position.x = station_path[i].x;
			p.poses[i].pose.position.y = station_path[i].y;
			p.poses[i].pose.orientation.w = 1.0;

			plan[i].header.frame_id = "/map";
			plan[i].header.stamp = ros::Time::now();
			plan[i].pose.position.x = station_path[i].x;
			plan[i].pose.position.y = station_path[i].y;
			plan[i].pose.orientation.w = 1.0;
		}
		p.header.stamp = ros::Time();
		p.header.frame_id = "/map";
		

		//p.poses.push_back(start);
		//p.poses.push_back(goal);
		g_plan_pub_.publish(p);
		//plan.push_back(targets);
		return true;
		
		
	}
		
	
	
};
