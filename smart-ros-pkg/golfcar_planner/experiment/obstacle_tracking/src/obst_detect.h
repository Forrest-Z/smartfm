/*
 * obst_detect.h
 *
 *  Created on: Jun 4, 2013
 *      Author: liuwlz
 */

#ifndef OBST_DETECT_H_
#define OBST_DETECT_H_

//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//PCL Segmentation

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <obstacle_tracking/obst_info.h>

#include <pnc_msgs/local_map.h>

#include <dynamic_reconfigure/server.h>
#include "obstacle_tracking/clusterCTRConfig.h"

#include <tf/tf.h>

#include <float.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointClound;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloundPtr;

class ObstDetect{

	//ROS
	ros::NodeHandle nh_;

	tf::TransformListener tf_;

	//SUbscriber of Laser scans and ldmrs pointcloud
    tf::MessageFilter<sensor_msgs::PointCloud2> *pointcloud_filter_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
	tf::MessageFilter<sensor_msgs::LaserScan> *laser1_filter_, *laser2_filter_, *laser3_filter_;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser1_sub_, laser2_sub_, laser3_sub_;

	//Publisher of un/processed pts for visualisation, obst_info for planning reasoning and local_map building
	ros::Publisher unprocessed_pts_pub;
	ros::Publisher processed_pts_pub;
	ros::Publisher interest_pts_pub;
	ros::Publisher obst_polys_pub;
	ros::Publisher obst_marker_pub;

	laser_geometry::LaserProjection projector_;

	string local_frame_;

	//Dynamic reconfigure
	dynamic_reconfigure::Server<obstacle_tracking::clusterCTRConfig>  *srv;
	double cluster_tolerance;
	int min_cluster_size;
	int max_cluster_size;
	int moving_avg_size;

	sensor_msgs::PointCloud obst_pts;
	sensor_msgs::PointCloud laser1_pts;
	sensor_msgs::PointCloud laser2_pts;
	sensor_msgs::PointCloud laser3_pts;

	sensor_msgs::PointCloud filtered_interest_pts;
	sensor_msgs::ChannelFloat32 obst_channel;
	geometry_msgs::PolygonStamped interpreted_ploy;
	vector<geometry_msgs::PolygonStamped> obst_polys;

	visualization_msgs::MarkerArray markerarray;
	uint32_t shape;

public:
	ObstDetect();
	~ObstDetect();

	//Add more laser callback to facilitate cooperative perception
    void Laser1CB(sensor_msgs::LaserScanConstPtr scan_in);
    void Laser2CB(sensor_msgs::LaserScanConstPtr scan_in);
    void Laser3CB(sensor_msgs::LaserScanConstPtr scan_in);
    void UpdateObst(sensor_msgs::PointCloud pc_in);
    void PointcloudCB(sensor_msgs::PointCloud2ConstPtr pc);

    void ParamReconfigure(obstacle_tracking::clusterCTRConfig &config, uint32_t level);

	void ObstClustering(sensor_msgs::PointCloud obst_pts);
	void ObstExtraction(sensor_msgs::PointCloud clustered_pcl);
	void ObstInterpretate(sensor_msgs::PointCloud obst_interest_pts);
	void ObstMarkerVisualize(vector<geometry_msgs::PolygonStamped> polys);
	void RegionFilling(geometry_msgs::PolygonStamped polygon);
	double InnerAngleCal(geometry_msgs::Point32 point_pre, geometry_msgs::Point32 point_curr, geometry_msgs::Point32 point_post);
};
#endif /* OBST_DETECT_H_ */
