/*
 * local_map.cpp
 *
 *  Created on: Nov 13, 2013
 *      Author: liuwlz
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Metric_Map/Dist_Map.hpp>

#include <pnc_msgs/local_map.h>
#include <pnc_msgs/move_status.h>

using namespace std;

#define GOLFCAR_WIDTH 1.2;
#define GOLFCAR_HEIGHT 2.4;
#define REF_PATH_RES 0.5;
#define MAX_DIST_CAL 2.0;

namespace MPAV{

	class LocalMap{

	public:
		LocalMap();

	private:

		bool useMetric, obstAnalyze, priorInit, updateMapInit, firstStationGoal, CoopTrigger;
		int obstValue;
		DistMetric *obsMetric, *refPathDist;
		int updateMapSkipMax, updateMapSkip;

		//TO get the clear space and check the nearest obst.
		geometry_msgs::PolygonStamped clear_space;
		pnc_msgs::move_status move_status_;
		nav_msgs::Path reference_path;

		nav_msgs::OccupancyGrid local_map_;
		nav_msgs::OccupancyGrid prior_map_;

		geometry_msgs::PoseStamped station_goal;

		sensor_msgs::PointCloud laser1_pts, laser2_pts_, laser3_pts_, replan_check_pts;
		sensor_msgs::PointCloud prior_pts_, local_map_pts_;
		vector<int> prior_obs_master_, prior_obs_;

		int track_dist_;
		double height_, width_, resolution_;
		double extend_dist;

		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		ros::Duration *tf_sleeper_;

		tf::TransformListener tf_;
		tf::TransformBroadcaster broadcaster_;
		tf::StampedTransform transform_;
		string global_frame_, local_frame_, base_frame_;

		ros::Publisher map_pub_, prior_pts_pub_, map_pts_pub_;
		ros::Publisher obst_dist_pub_, clear_space_pub_, obst_view_pub_, prior_map_pub_;
		ros::Subscriber move_status_sub_;
		ros::Subscriber reference_path_sub_;
		ros::Subscriber station_goal_sub_;

		tf::MessageFilter<sensor_msgs::PointCloud2> *pointcloud_filter_;
		message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;

		tf::MessageFilter<sensor_msgs::LaserScan> *laser_filter_, *laser2_filter_, *laser3_filter_;
		message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_, laser2_sub_, laser3_sub_;
		laser_geometry::LaserProjection projector_;

		void laserCallback(sensor_msgs::LaserScanConstPtr pc);
		void laser2Callback(sensor_msgs::LaserScanConstPtr pc);
		void laser3Callback(sensor_msgs::LaserScanConstPtr pc);

		void tfTimerCallback(const ros::TimerEvent &event);
		void movestatusCallBack(const pnc_msgs::move_status move_status);
		void referencePathCallBack(const nav_msgs::Path ref_path);
		void pointcloudCallback(sensor_msgs::PointCloud2ConstPtr pc);
		void StationGoalCannback(const geometry_msgs::PoseStamped goalIn);

		void initPriorPoints();
		void publishLocalMapPts(vector<int> &free_cells);
		void updatePriorObsWithLane(bool norminal);
		void extendReferencePath();

		void getClearSpace();
		void getNearestObst();

		void updateMap(sensor_msgs::PointCloud& pc);
		void addPointToPriorMap(geometry_msgs::Point map_p);
		void addPointToMap(geometry_msgs::Point32 map_p);
		void addPointToMap(geometry_msgs::Point32 map_p, int occ);
		bool getRobotPose(tf::Stamped<tf::Pose> &odom_pose) const;
		void getGradientCost(nav_msgs::OccupancyGrid &gradient_local_map_);
	};
}
