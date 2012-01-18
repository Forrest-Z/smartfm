/*
 * pcl_clustering.cpp
 *
 *  Created on: Jan 15, 2012
 *      Author: golfcar
 */
#include <ros/ros.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <fmutil/fm_math.h>
#include <std_msgs/Int64.h>
using namespace std;

class ped_clustering
{
public:
    ped_clustering();
private:
    void scanCallback(const sensor_msgs::PointCloud2ConstPtr& pc2);
    void clustering(const sensor_msgs::PointCloud2ConstPtr& pc2);
    void filterLines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
    void extractCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<pcl::PointIndices>& cluster_indices);
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher poi_pub_, line_filtered_pub_;
    ros::Publisher ped_candNo_pub_;
};

ped_clustering::ped_clustering()
{
    ros::NodeHandle nh;
    cloud_sub_ = nh.subscribe("sickldmrs/cloud", 10, &ped_clustering::scanCallback, this);
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud>("pedestrian_cluster", 1);
    poi_pub_= nh.advertise<sensor_msgs::PointCloud>("pedestrian_poi",1);
    line_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("line_filtered",1);
    ped_candNo_pub_ = nh.advertise<std_msgs::Int64>("ped_candNo",1);
    ros::spin();
}

void ped_clustering::scanCallback(const sensor_msgs::PointCloud2ConstPtr& pc2)
{

    clustering(pc2);
}

void ped_clustering::clustering(const sensor_msgs::PointCloud2ConstPtr& pc)
{
    // Create the filtering object: downsample the dataset using a leaf size of 1cm

    pcl::PointCloud<pcl::PointXYZ> cloud_temp;
    pcl::fromROSMsg(*pc, cloud_temp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = cloud_temp.makeShared();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    int i = 0;

    //line filtered quite nicely, but then the side effect is the points become more cluttered

    std::vector<pcl::PointIndices> cluster_indices;
    extractCluster(cloud_filtered, cluster_indices);
    //cloud_cluster = (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
    }


    //now line filters
    filterLines(cloud_cluster);
    sensor_msgs::PointCloud2 line_filtered;
    pcl::toROSMsg(*cloud_cluster, line_filtered);
    line_filtered.header = pc->header;
    line_filtered_pub_.publish(line_filtered);

    //then cluster again
    cluster_indices.clear();
    extractCluster(cloud_cluster, cluster_indices);
    sensor_msgs::PointCloud total_clusters, ped_poi;
    int bounded_filter = 0;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        /**/
        Eigen::Vector4f min_pt, max_pt,mid_pt, abs_distance;
        pcl::getMinMax3D(*cloud_cluster, it->indices, min_pt, max_pt);
        //cout<<i<<" indices min_pt: "<<min_pt<<" max_pt: "<<max_pt<<endl;
        abs_distance = max_pt - min_pt;
        mid_pt = (max_pt + min_pt)/2.0;

        bool max_dist = fmutil::distance(mid_pt[0], mid_pt[1], 0, 0)<20;
        if(abs_distance[0]< 1.5 && abs_distance[1]<1.5 && abs_distance[2]>0.1 && max_dist)
        {
            geometry_msgs::Point32 p;
            p.x = mid_pt[0];
            p.y = mid_pt[1];
            p.z = mid_pt[2];
            ped_poi.points.push_back(p);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            {

                p.x = cloud_cluster->points[*pit].x;
                p.y = cloud_cluster->points[*pit].y;
                p.z = i;
                total_clusters.points.push_back(p);
            }

            bounded_filter++;
        }

        i++;
    }
    cout<<"After bounding box filtering = "<<bounded_filter<<endl;
    total_clusters.header = pc->header;
    ped_poi.header = pc->header;
    cloud_pub_.publish(total_clusters);
    poi_pub_.publish(ped_poi);
    std_msgs::Int64 ped_no;
    ped_no.data = bounded_filter;
    ped_candNo_pub_.publish(ped_no);
    /* Verified that the following code would produce the exact same pointcloud sensor msg
    sensor_msgs::PointCloud total_clusters;
    for(int j=0;j<cloud_filtered->points.size();j++)
    {
        geometry_msgs::Point32 p;
                    p.x = cloud_filtered->points[j].x;
                    p.y = cloud_filtered->points[j].y;
                    p.z = cloud_filtered->points[j].z;
        total_clusters.points.push_back(p);
    }
    total_clusters.header = pc->header;
    cloud_pub_.publish(total_clusters);*/

}

void ped_clustering::extractCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<pcl::PointIndices>& cluster_indices)
{
    if(cloud_filtered->size()==0) return;
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud (cloud_filtered);


    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.5);
    ec.setMinClusterSize (3);
    ec.setMaxClusterSize (100);
    ec.setSearchMethod (tree);
    ec.setInputCloud( cloud_filtered);

    ec.extract (cluster_indices);
}
void ped_clustering::filterLines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.05);

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment (*inliers, *coefficients); //*
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Write the planar inliers to disk
        extract.filter (*cloud_plane); //*
        //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_filtered); //*
    }
}
int
main (int argc, char** argv)
{
    ros::init(argc, argv, "ped_clustering");
    ped_clustering ped_cluster;
    return (0);
}
