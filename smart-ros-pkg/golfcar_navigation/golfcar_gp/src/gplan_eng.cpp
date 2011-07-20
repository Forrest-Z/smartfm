#include <pluginlib/class_list_macros.h>
#include <math.h> 
#include <golfcar_gp/gplan.h>
#include <geometry_msgs/Pose2D.h>
PLUGINLIB_DECLARE_CLASS(golfcar_gp, GlobalPlan, golfcar_gp::GlobalPlan, nav_core::BaseGlobalPlanner)

namespace golfcar_gp{
	
	void GlobalPlan::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		ros::NodeHandle global_node;
		g_plan_pub_ = global_node.advertise<nav_msgs::Path>("global_plan", 1);
		direction_pub_ = global_node.advertise<geometry_msgs::Point32>("direction", 1);
		ROS_INFO("gplan initialized");
		firstpart=true;
		stage_ = 0;
	}
	
	bool GlobalPlan::makePlan(const geometry_msgs::PoseStamped&  start,	const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) 
	{
		
		//McDonald's & Eng!!
		const double res = 0.1;
		//all in according to the pixels from global map
		std::vector<geometry_msgs::Pose2D> targets;
		geometry_msgs::Pose2D target;

		switch(stage_)
		{
			case 0:
			//target.x = 1077; target.y = 2158; targets.push_back(target);
			//target.x = 1053; target.y = 2188; targets.push_back(target);
			//target.x = 1047; target.y = 2221; targets.push_back(target);
			target.x = 1067; target.y = 2272; targets.push_back(target);
			
			target.x = 1443; target.y = 3056; targets.push_back(target);
			target.x = 1527; target.y = 3096; targets.push_back(target);
			target.x = 1609; target.y = 3118; targets.push_back(target);
			target.x = 1627; target.y = 3201; targets.push_back(target);
			target.x = 1585; target.y = 3255; targets.push_back(target);
			target.x = 1526; target.y = 3243; targets.push_back(target);
			target.x = 1489; target.y = 3206; targets.push_back(target);
			target.x = 1473; target.y = 3137; targets.push_back(target);
			stage_++;
			break;
			
			case 1:
			target.x = 1432; target.y = 3061; targets.push_back(target);
			target.x = 1093; target.y = 2349; targets.push_back(target);
			target.x = 1067; target.y = 2272; targets.push_back(target);
			target.x = 1015; target.y = 2179; targets.push_back(target);
			target.x = 970; target.y = 2109; targets.push_back(target);
			target.x = 932; target.y = 2056; targets.push_back(target);
			target.x = 885; target.y = 2023; targets.push_back(target);
			target.x = 829; target.y = 2003; targets.push_back(target);
			target.x = 769; target.y = 1998; targets.push_back(target);
			target.x = 728; target.y = 2006; targets.push_back(target);
			target.x = 673; target.y = 2032; targets.push_back(target);
			target.x = 632; target.y = 2064; targets.push_back(target);
			target.x = 598; target.y = 2100; targets.push_back(target);
			target.x = 566; target.y = 2147; targets.push_back(target);
			target.x = 536; target.y = 2192; targets.push_back(target);
			target.x = 517; target.y = 2239; targets.push_back(target);
			target.x = 507; target.y = 2271; targets.push_back(target);
			target.x = 469; target.y = 2318; targets.push_back(target);
			target.x = 412; target.y = 2334; targets.push_back(target);
			target.x = 363; target.y = 2347; targets.push_back(target);
			stage_++;
			break;
			
			case 2:
			target.x = 324; target.y = 2331; targets.push_back(target);
			target.x = 302; target.y = 2285; targets.push_back(target);
			target.x = 300; target.y = 2216; targets.push_back(target);
			target.x = 346; target.y = 1722; targets.push_back(target);
			target.x = 374; target.y = 1224; targets.push_back(target);
			target.x = 386; target.y = 1165; targets.push_back(target);
			target.x = 563; target.y = 672; targets.push_back(target);
			target.x = 726; target.y = 344; targets.push_back(target);
			target.x = 781; target.y = 245; targets.push_back(target);
			target.x = 837; target.y = 213; targets.push_back(target);
			target.x = 894; target.y = 217; targets.push_back(target);
			target.x = 1423; target.y = 415; targets.push_back(target);
			target.x = 1856; target.y = 563; targets.push_back(target);
			target.x = 1938; target.y = 596; targets.push_back(target);
			target.x = 1994; target.y = 612; targets.push_back(target);
			target.x = 2010; target.y = 638; targets.push_back(target);
			target.x = 1986; target.y = 659; targets.push_back(target);
			target.x = 1951; target.y = 672; targets.push_back(target);
			target.x = 1923; target.y = 679; targets.push_back(target);
			target.x = 1881; target.y = 667; targets.push_back(target);
			stage_++;
			break;
			
			case 3:
			target.x = 1833; target.y = 650; targets.push_back(target);
			target.x = 1810; target.y = 610; targets.push_back(target);
			target.x = 1789; target.y = 573; targets.push_back(target);
			target.x = 1740; target.y = 545; targets.push_back(target);
			target.x = 1415; target.y = 434; targets.push_back(target);
			target.x = 888; target.y = 242; targets.push_back(target);
			target.x = 841; target.y = 238; targets.push_back(target);
			target.x = 796; target.y = 267; targets.push_back(target);
			target.x = 746; target.y = 353; targets.push_back(target);
			target.x = 687; target.y = 458; targets.push_back(target);
			target.x = 588; target.y = 683; targets.push_back(target);
			target.x = 497; target.y = 912; targets.push_back(target);
			target.x = 410; target.y = 1175; targets.push_back(target);
			target.x = 399; target.y = 1229; targets.push_back(target);
			target.x = 374; target.y = 1721; targets.push_back(target);
			target.x = 325; target.y = 2217; targets.push_back(target);
			target.x = 334; target.y = 2270; targets.push_back(target);
			target.x = 369; target.y = 2306; targets.push_back(target);
			target.x = 431; target.y = 2286; targets.push_back(target);
			target.x = 481; target.y = 2244; targets.push_back(target);
			target.x = 520; target.y = 2183; targets.push_back(target);
			target.x = 558; target.y = 2131; targets.push_back(target);
			target.x = 588; target.y = 2084; targets.push_back(target);
			target.x = 622; target.y = 2047; targets.push_back(target);
			target.x = 663; target.y = 2010; targets.push_back(target);
			target.x = 720; target.y = 1986; targets.push_back(target);
			target.x = 768; target.y = 1978; targets.push_back(target);
			target.x = 833; target.y = 1982; targets.push_back(target);
			target.x = 892; target.y = 2008; targets.push_back(target);
			target.x = 943; target.y = 2044; targets.push_back(target);
			target.x = 983; target.y = 2099; targets.push_back(target);
			target.x = 1009; target.y = 2130; targets.push_back(target);
			target.x = 1053; target.y = 2122; targets.push_back(target);
			target.x = 1077; target.y = 2159; targets.push_back(target);
		}
		
		std::vector<geometry_msgs::PoseStamped> final_targets;
		final_targets.resize(targets.size());
		for(int i=0; i<targets.size(); i++)
		{
			final_targets[i].header.frame_id = "/map";
			final_targets[i].header.stamp = ros::Time::now();
			
			final_targets[i].pose.position.x = targets[i].x * res;
			final_targets[i].pose.position.y = (3536 - targets[i].y) * res;
			final_targets[i].pose.orientation = tf::createQuaternionMsgFromYaw(targets[i].theta);
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
		//plan.push_back(targets);
		plan = final_targets;
		return true;
		
		
	}
		
	
	
};
