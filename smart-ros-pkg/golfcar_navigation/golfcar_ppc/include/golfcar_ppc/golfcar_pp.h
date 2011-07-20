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

	class PurePursuit
    {
    public:
		PurePursuit();
		~PurePursuit();

    private:
        ros::Subscriber traj_sub_;
		ros::Publisher cmd_pub_;
        ros::Timer timer_;

        double normal_speed_;
        double slow_speed_;
        int start_decreasing_;
        double turning_radius_;
        double look_ahead_;
        double look_ahead_bad_;
        double max_steering_;
        double switch_distance_;
        double car_length_;

		tf::TransformListener tf_;
		nav_msgs::Path trajectory_;
        int last_segment_;

		void trajCallBack(const nav_msgs::Path::ConstPtr &traj);
		void controlLoop(const ros::TimerEvent &e);

		bool getRobotPose(tf::Stamped<tf::Pose>& odom_pose) const;
        double get_distance(double x1, double y1, double x2, double y2);
        bool get_center(double tar_x, double tar_y,
                        double ori_x, double ori_y, double inv_R,
                        double center[2]);
        double get_inv_R(int segment);
        bool btwn_points(double tar_x, double tar_y,
                         double ori_x, double ori_y,
                         double inv_R, double x, double y);
        void get_projection(double tar_x, double tar_y,
                            double ori_x, double ori_y,
                            double inv_R, double cur_x, double cur_y,
                            double proj[2]);
        int get_segment(double cur_x, double cur_y);
        double get_steering(int segment, double cur_x, double cur_y, double cur_yaw, double& cmd_vel);
	};
};
