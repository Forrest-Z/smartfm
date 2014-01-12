#ifndef MODT_VEHICLE_TRACKING_ICP_H
#define MODT_VEHICLE_TRACKING_ICP_H

#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include "MODT/segment_pose_batches.h"
#include <mrpt/slam.h>
#include <tf/transform_broadcaster.h>
#include "EKF_tracker/ConstSpeed_EKF_tracker.h"
/*
 * Pay attention here to remove irrelavant header files from ros;
 * It turns out that some header files will mess up the mrpt compilation;
 */

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;

namespace mrpt{
	
	class model_free_track
	{
		public:
		int object_id;
		int type_label;
		ros::Time update_time;
		sensor_msgs::PointCloud contour_points;
		std::vector<geometry_msgs::Point32> anchor_points;
		std::vector<geometry_msgs::Point32> filtered_anchor_points;

		bool 					tracking_inited;

		double					moving_direction;
		double 					velocity;
		double 					omega;

		MODT::segment_pose_batch	last_measurement;
		constspeed_ekf_tracker 		*tracker;

		model_free_track()
		{
			tracker = new constspeed_ekf_tracker();
		}

	};

    class vehicle_tracking
    {
	     public:

		int object_total_id_;
    	//need a datatype to serve as the track history;
		std::vector<model_free_track> object_tracks_;

    	ros::Subscriber segpose_batch_sub_;
    	ros::Publisher contour_cloud_pub_, anchor_point_pub_, filtered_anchor_point_pub_;
    	ros::Time latest_input_time_;
    	vehicle_tracking();
		~vehicle_tracking(){};

		void measurement_callback(const MODT::segment_pose_batches& batches);

		//basically data association step, but it can be removed in the futher with a smarter segmentation code;
		//since data association is already solved implicitly in the pcl segmentation part;
		void register_cluster2history(MODT::segment_pose_batches& batches);

		//perform some tracker's basic operations;
		//1st: initiate new tracks; 2nd: merge and split; 3rd: update with measurements; 4th: delete obsolete tracks;
		void update_motionShape(MODT::segment_pose_batches& batches, std::vector<std::pair<size_t, size_t> > &track_measure_pairs, std::vector<size_t> &unregistered_measurements);

		//update object's position and shape using ICP;
		void ICP_motion2shape(model_free_track &track, MODT::segment_pose_batch& old_meas, MODT::segment_pose_batch& new_meas, tf::Pose& oldMeas_poseinOdom, tf::Pose& newMeas_poseinOdom);
		void construct_ICP_scans(geometry_msgs::Pose &lidar_pose, sensor_msgs::PointCloud &segment_pointcloud, CObservation2DRangeScan& scan);

		void attached_points_transform(geometry_msgs::Point32& pt_input, geometry_msgs::Point32& pt_output, tf::Pose& oldMeas_poseinOdom, tf::Pose& newMeas_poseinOdom);

		void tracks_visualization();
		void constructPtsMap(geometry_msgs::Pose &lidar_pose, sensor_msgs::PointCloud &segment_pointcloud, CSimplePointsMap &map);
		void ICP_deputy_cloud(sensor_msgs::PointCloud &meas_cloud, sensor_msgs::PointCloud &model_cloud,  tf::Pose lidar_pose, sensor_msgs::PointCloud &deputy_meas_cloud, sensor_msgs::PointCloud &deputy_model_cloud);
    };
};

#endif
