#ifndef MODT_VEHICLE_TRACKING_BOX_H
#define MODT_VEHICLE_TRACKING_BOX_H

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

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;

class box_model
{
	public:
	//point 0 is the initial refPt;
	geometry_msgs::Point32 corner_points[4];
	float lidar_angle[4];
	int refPt_seial;
	double moving_direction;
	double width, length;
	bool init;

	box_model()
	{
		init = false;
	}
};

class box_model_track
{
	public:
	int object_id;
	int type_label;

	ros::Time update_time;
	sensor_msgs::PointCloud contour_points;
	std::vector<geometry_msgs::Point32> anchor_points, filtered_anchor_points;
	box_model				shape;

	double					moving_direction;
	double 					velocity;
	double					omega;

	MODT::segment_pose_batch	last_measurement;
	constspeed_ekf_tracker 		*tracker;
	double vehicle_evidence;

	box_model_track()
	{
		tracker = new constspeed_ekf_tracker();
		vehicle_evidence = 0;

	}

	void update_object_belief(bool evidence)
	{
		if(evidence){vehicle_evidence = vehicle_evidence + 1.0;}
		else if(!evidence) {vehicle_evidence = vehicle_evidence - 1.0;}
		if(vehicle_evidence > 10.0) vehicle_evidence = DBL_MAX;
	}

};

class vehicle_tracking_box
{
	 public:
	ros::NodeHandle private_nh_;
	int object_total_id_;
	//need a datatype to serve as the track history;
	std::vector<box_model_track> object_tracks_;

	ros::Subscriber segpose_batch_sub_;
	ros::Publisher contour_cloud_pub_, anchor_point_pub_, filtered_anchor_point_pub_, meas_polygon_pub_;
	ros::Time latest_input_time_;
	std::string				odom_frame_id_;
	vehicle_tracking_box();
	~vehicle_tracking_box(){};

	void measurement_callback(const MODT::segment_pose_batches& batches);

	//basically data association step, but it can be removed in the futher with a smarter segmentation code;
	//since data association is already solved implicitly in the pcl segmentation part;
	void register_cluster2history(MODT::segment_pose_batches& batches);

	//perform some tracker's basic operations;
	//1st: initiate new tracks; 2nd: merge and split; 3rd: update with measurements; 4th: delete obsolete tracks;
	void maintain_tracks(MODT::segment_pose_batches& batches, std::vector<std::pair<size_t, size_t> > &track_measure_pairs, std::vector<size_t> &unregistered_measurements);
	void motion_estimation(MODT::segment_pose_batch& new_meas, double &delt_x, double &delt_y);

	void calculate_measurement_box(MODT::segment_pose_batch& new_meas, double current_moving_direction, box_model &measurement_box);
	void update_box_track(box_model_track &track, box_model &measurement_box);
	void measPt_lidarAngle(geometry_msgs::Pose &lidar_pose, geometry_msgs::Point32 measPt, float &lidarAngle);

	void tracks_visualization();
};


#endif
