/*
 * ped_clustering.h
 *
 *  Created on: Jun 1, 2012
 *      Author: demian
 */

#include <ros/ros.h>

#include <pcl16/ModelCoefficients.h>
#include <pcl16/point_types.h>
#include <pcl16/io/pcd_io.h>
#include <pcl16/features/normal_3d.h>
#include <pcl16/filters/extract_indices.h>
#include <pcl16/filters/voxel_grid.h>
#include <pcl16/kdtree/kdtree.h>
#include <pcl16/sample_consensus/method_types.h>
#include <pcl16/sample_consensus/model_types.h>
#include <pcl16/segmentation/sac_segmentation.h>
#include <pcl16/segmentation/extract_clusters.h>
//#include <pcl16/segmentation/extract_clusters.h>
#include <pcl16/common/common.h>
#include <pcl16/common/pca.h>
#include <pcl16/octree/octree_search.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <fmutil/fm_math.h>
#include <std_msgs/Int64.h>

#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <feature_detection/clusters.h>
#include <feature_detection/cluster.h>

//#include <octomap_ros/OctomapROS.h>
//#include <octomap_msgs/GetOctomap.h>
//#include <octomap_msgs/OctomapBinary.h>
//#include <octomap/octomap.h>
#include <pcl16/filters/radius_outlier_removal.h>

#include "read_svg.h"
#include "pnpoly.h"
using namespace std;


class ped_clustering
{
public:
    ped_clustering();
private:
    void scanCallback(const sensor_msgs::PointCloud2ConstPtr& pc2);
    void clustering(const sensor_msgs::PointCloud2& pc2, sensor_msgs::PointCloud &ped_poi,
                    double tolerance, int minSize, int maxSize, bool publish);
    void filterLines(pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_in, pcl16::PointCloud<pcl16::PointXYZ>& cloud_out);
    void extractCluster(pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_filtered, std::vector<pcl16::PointIndices>& cluster_indices,
                        double clusterTolerance, int minSize,int maxSize);
    void laserCallback(const sensor_msgs::LaserScanConstPtr& scan_in);
    //void filterPriorMap(octomap::OcTree& priorMap, sensor_msgs::PointCloud& pc_in, sensor_msgs::PointCloud& pc_out, ros::Publisher& pub_, double threshold);
    //void octomapTreeToPCLoctree(octomap_msgs::OctomapBinary& octomap, pcl16::octree::OctreePointCloudSearch<pcl16::PointXYZ>* pcl_octree);
    //void filterPCLOctreeNN(pcl16::octree::OctreePointCloudSearch<pcl16::PointXYZ> &priorMap, sensor_msgs::PointCloud& pc_in, sensor_msgs::PointCloud& pc_out, ros::Publisher& pub_, double threshold);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    ros::Publisher cloud_pub_, filter_pub_, after_line_filter_pub_, after_prior_filter_pub_;
    ros::Publisher poi_pub_, line_filtered_pub_;
    ros::Publisher clusters_pub_, boundary_pub_;

    tf::TransformListener tf_;
    tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pc2_filter_;
    double prior_distance_filter_;
    laser_geometry::LaserProjection projector_;
    string laser_frame_id_, global_frame_;
    bool bounding_boxfilter_, line_filter_;
    bool sequential_clustering_, use_octomap_, use_boundary_;
    //octomap::OcTree* global_octMap;
    //pcl16::octree::OctreePointCloudSearch<pcl16::PointXYZ>* global_pclOctree_;
    void getOctomap();
    ros::NodeHandle nh;

    vector<Point32> boundary_;
    sensor_msgs::PointCloud laser_global_pc_;
};



