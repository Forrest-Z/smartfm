#include <pluginlib/class_list_macros.h>
#include <math.h> 
#include <simple_gp/gplan.h>
#include <geometry_msgs/Pose2D.h>
PLUGINLIB_DECLARE_CLASS(simple_gp, GlobalPlan, simple_gp::GlobalPlan, nav_core::BaseGlobalPlanner)

namespace simple_gp{
	
	void GlobalPlan::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		ros::NodeHandle global_node;
		g_plan_pub_ = global_node.advertise<nav_msgs::Path>("global_plan", 1);
	}
	
	bool GlobalPlan::makePlan(const geometry_msgs::PoseStamped&  start,	const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) 
	{
		
		plan.push_back(start);
		plan.push_back(goal);
		
		nav_msgs::Path p;
		p.header.stamp = ros::Time::now();
		p.header.frame_id = "/map";
		p.poses.resize(2);
		p.poses[0] = start;
		p.poses[1] = goal;
		g_plan_pub_.publish(p);
		
		return true;
		
		
	}
		
	
	
};
