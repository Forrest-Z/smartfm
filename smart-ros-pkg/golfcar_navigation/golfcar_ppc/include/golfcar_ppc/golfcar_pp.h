#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <string>
#include <cmath>

using namespace std;

namespace golfcar_purepursuit {


	class PurePursuit{
		
		public:
		
		PurePursuit();
		~PurePursuit();

		bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;
		private:
		ros::NodeHandle n_;
        	ros::Subscriber traj_sub_;
		ros::Publisher cmd_pub_;
		nav_msgs::Path trajectory_;
		void trajCallBack(const nav_msgs::Path::ConstPtr &traj);
		void controlLoop();
		tf::TransformListener tf_;
		double freq_;
	};
};
