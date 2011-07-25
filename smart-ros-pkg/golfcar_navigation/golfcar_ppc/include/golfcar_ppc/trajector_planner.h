#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/cost_values.h>
#include <golfcar_ppc/golfcar_purepursuit.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

using namespace costmap_2d;
namespace golfcar_purepursuit{
	
	class PurePursuitBase : public nav_core::BaseLocalPlanner {

		public:
	
		void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
		
		bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

		
		bool isGoalReached();

		
		bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
		PurePursuit* pp_;
		
		private:
		void getFillCells(std::vector<MapLocation>& footprint);
		geometry_msgs::PolygonStamped clear_space_;
		costmap_2d::Costmap2DROS* costmap_ros_;
		tf::TransformListener* tf_;
		ros::Subscriber odom_sub_;
		ros::Publisher l_plan_pub_,g_plan_pub_,clear_space_pub_;
		bool path_flag_;
		void UpdatePosition();
		geometry_msgs::PoseStamped robot_pose;
		std::vector<geometry_msgs::PointStamped> lookahead_points;
		double slow_speed_, maximum_speed_, highrisk_speed_;
		bool forward_;
		double steer_angle_;
		void golfcar_direction(geometry_msgs::Point32 p);
		ros::Subscriber golfcar_direction_;
		ros::Time expected_end_;
		bool stopped_;
		bool goalreached_;
		int waypointPassed_;
	};
};
			
				
	
