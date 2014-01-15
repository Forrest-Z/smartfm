#ifndef MODT_VEHICLE_TRACKING_ICP_H
#define MODT_VEHICLE_TRACKING_ICP_H

#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include "MODT/segment_pose_batches.h"
#include <mrpt/slam.h>
#include <tf/transform_broadcaster.h>

#include "EKF_tracker/ConstSpeed_EKF_tracker.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
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
		std::vector<geometry_msgs::Point32> anchor_points, filtered_anchor_points;

		double					moving_direction;
		double 					velocity;
		double					omega;

		MODT::segment_pose_batch	last_measurement;
		constspeed_ekf_tracker 		*tracker;

		//status 0-normal, 1-forbid orientation, 2-rely on tracker and no measurement update;
		int track_status;

		model_free_track()
		{
			tracker = new constspeed_ekf_tracker();
			track_status = 0;
		}

		void downsample_model()
		{
			pcl::VoxelGrid<pcl::PointXYZ> sor;
			pcl::PointCloud<pcl::PointXYZ> model;
			for(size_t i=0; i<contour_points.points.size(); i++)
			{
				pcl::PointXYZ contour_point_tmp;
				contour_point_tmp.x = contour_points.points[i].x;
				contour_point_tmp.y = contour_points.points[i].y;
				contour_point_tmp.z = 0.0;
				model.push_back(contour_point_tmp);
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr model_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
			float downsample_size_ = 0.05;
			sor.setInputCloud(model.makeShared());
			sor.setLeafSize (downsample_size_, downsample_size_, downsample_size_);
			sor.filter (*model_filtered);
			model = * model_filtered;

			contour_points.points.clear();
			for(size_t i=0; i<model.points.size(); i++)
			{
				geometry_msgs::Point32 contour_point_tmp;
				contour_point_tmp.x = model.points[i].x;
				contour_point_tmp.y = model.points[i].y;
				contour_point_tmp.z = 0.0;
				contour_points.points.push_back(contour_point_tmp);
			}
		}

	};

    class vehicle_tracking
    {
	     public:

		int object_total_id_;
    	//need a datatype to serve as the track history;
		std::vector<model_free_track> object_tracks_;

    	ros::Subscriber segpose_batch_sub_;
    	ros::Publisher contour_cloud_pub_, anchor_point_pub_, filtered_anchor_point_pub_, meas_deputy_pub_, model_deputy_pub_;
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

		void ICP_deputy_cloud(model_free_track& track, tf::Pose lidar_pose, sensor_msgs::PointCloud &meas_cloud, sensor_msgs::PointCloud &model_cloud,  sensor_msgs::PointCloud &deputy_meas_cloud, sensor_msgs::PointCloud &deputy_model_cloud);


		void attached_points_transform(geometry_msgs::Point32& pt_input, geometry_msgs::Point32& pt_output, tf::Pose& oldMeas_poseinOdom, tf::Pose& newMeas_poseinOdom);

		void tracks_visualization();
		void constructPtsMap(geometry_msgs::Pose &lidar_pose, sensor_msgs::PointCloud &segment_pointcloud, CSimplePointsMap &map);
    };
};

#endif
