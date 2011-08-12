#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <string>
#include <cmath>
#include <geometry_msgs/PolygonStamped.h>
#include <math.h>

namespace speed_advisor {


	class SpeedAdvisor{
		
		public:
		
		SpeedAdvisor();
		~SpeedAdvisor();
		
		tf::TransformListener tf_;
		ros::Publisher recommend_speed_;
		ros::Subscriber move_base_speed_;
		ros::Subscriber global_plan_;
		int lastStop_;
		geometry_msgs::Twist move_speed_;
		bool getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const;
		void moveSpeedCallback(geometry_msgs::Twist cmd_vel);
		void globalPlanCallback(nav_msgs::Path path);
		double sqrtDistance(double x1, double y1, double x2, double y2);
	};
};
