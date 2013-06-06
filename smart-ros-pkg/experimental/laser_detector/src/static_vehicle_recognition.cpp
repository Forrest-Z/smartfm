/*
 * static_vehicle_recognition.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: Shen Xiaotong
 */

#include "static_vehicle_recognition.h"


// for estimating the normals
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/features/impl/normal_3d.hpp>

#include <assert.h>
#include <fmutil/fm_stopwatch.h>

#include <geometry_msgs/PoseArray.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/common/concatenate.h>
#include <geometry_msgs/Quaternion.h>

#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>
#include <tf/transform_datatypes.h>

//plane segmentation
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>

//extract indices
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

//euclidean extraction
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

#include <sensor_msgs/PointCloud2.h>

golfcar_perception::static_vehicle_recognition::static_vehicle_recognition(void):nh_("")
{
	accu_pc_sub_ = nh_.subscribe("rolling_window_pcl",10,&static_vehicle_recognition::accu_pc_callback,this);
	normals_pub_ = nh_.advertise<geometry_msgs::PoseArray>("pc_normals_vis",5);

	plane_inlier_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("plane_inlier_pts",5);
	plane_outlier_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("plane_outlier_pts",5);
	ec_clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ec_clusters",5);

	publish_normal_flag_ = false;

}

void golfcar_perception::static_vehicle_recognition::accu_pc_callback(const RollingPointCloud::ConstPtr & accu_pc_in)
{

      ROS_INFO("accu_pc_callback function is invoked now !----!");
      // estimate the normals of the rolling window point cloud
      fmutil::Stopwatch sw;
      sw.start("computing point cloud normals");
      pcl::NormalEstimation<RollingPointXYZ,pcl::Normal> ne;
      ne.setInputCloud(accu_pc_in->makeShared());

      pcl::search::KdTree<RollingPointXYZ>::Ptr tree (new pcl::search::KdTree<RollingPointXYZ> ());
      ne.setSearchMethod(tree);

      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      ne.setRadiusSearch(0.1);
      ne.setViewPoint(0,0,1);
      ne.compute(*cloud_normals);
      sw.end();

      assert(cloud_normals->points.size() == accu_pc_in->points.size());
      RollingPointCloudNormal accu_pc_normal;
      pcl::concatenateFields(*accu_pc_in,*cloud_normals,accu_pc_normal);

      //try to publish the normals and visualize them in rviz
      if(publish_normal_flag_)
      {
		  geometry_msgs::PoseArray normals_poses;
		  normals_poses.header = accu_pc_normal.header;
		  for(unsigned int i = 0; i < accu_pc_normal.points.size();i++)
		  {
			  geometry_msgs::Pose normals_pose;
			  geometry_msgs::Point pos;
			  pos.x = accu_pc_normal.points[i].x;
			  pos.y = accu_pc_normal.points[i].y;
			  pos.z = accu_pc_normal.points[i].z;
			  normals_pose.position = pos;

			  if(accu_pc_normal.points[i].curvature < 0.05)
				  continue;

			  btVector3 dir(accu_pc_normal.points[i].normal_x,accu_pc_normal.points[i].normal_y,accu_pc_normal.points[i].normal_z);
			  btVector3 x_axis(1,0,0);
			  if(isnan(accu_pc_normal.points[i].normal_x) || isnan(accu_pc_normal.points[i].normal_y) || isnan(accu_pc_normal.points[i].normal_z) )
				  continue;
			  btQuaternion qt(x_axis.cross(dir), x_axis.angle(dir));
			  double yaw,pitch,roll;
			  btMatrix3x3(qt).getEulerYPR(yaw,pitch,roll);
			  geometry_msgs::Quaternion quat_msg;
			  tf::quaternionTFToMsg(qt,quat_msg);

			  //test what is the identity quaternion
			  normals_pose.orientation = quat_msg;
			  normals_poses.poses.push_back(normals_pose);

		  }
		  normals_pub_.publish(normals_poses);
      }

      //plane segmentation
      sw.start("plane segmentation");
      pcl::PointCloud<pcl::PointXYZ> temp_pc;
      temp_pc.header = accu_pc_in->header;
      temp_pc.height = accu_pc_in->height;
      temp_pc.is_dense = accu_pc_in->is_dense;
      temp_pc.points.resize(accu_pc_in->points.size());
      for(unsigned int k = 0; k < accu_pc_in->points.size(); k++)
      {
    	  temp_pc.points[k].x = accu_pc_in->points[k].x;
    	  temp_pc.points[k].y = accu_pc_in->points[k].y;
    	  temp_pc.points[k].z = accu_pc_in->points[k].z;
      }
      pcl::ModelCoefficients::Ptr plane_coef (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);

      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.1);
      seg.setInputCloud(temp_pc.makeShared());
      seg.segment(*plane_inliers,*plane_coef);

      seg.setDistanceThreshold (0.15);

	  if (plane_inliers->indices.size () == 0)
	  {
		  PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		  return;
	  }
      std::cerr << "Model inliers: " << plane_inliers->indices.size () << std::endl;

      //sensor_msgs::PointCloud2 plane_inlier_pts;
      //sensor_msgs::PointCloud2 plane_outlier_pts;
      PointCloud  plane_inlier_pts;
      PointCloud plane_outlier_pts;

      pcl::ExtractIndices<pcl::PointXYZ> extract_ind;
      extract_ind.setInputCloud(temp_pc.makeShared());
      extract_ind.setIndices(plane_inliers);
      extract_ind.setNegative(false);
      extract_ind.filter(plane_inlier_pts);

      extract_ind.setNegative(true);
      extract_ind.filter(plane_outlier_pts);

      plane_inlier_pub_.publish(plane_inlier_pts);
      plane_outlier_pub_.publish(plane_outlier_pts);
      sw.end();

      //do the euclidean clustering on the outlier points
      sw.start("euclidean clustering");
      pcl::search::KdTree<pcl::PointXYZ>::Ptr ec_tree (new pcl::search::KdTree<pcl::PointXYZ>);
      ec_tree->setInputCloud(plane_outlier_pts.makeShared());

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

      ec.setClusterTolerance(0.2);
      ec.setMaxClusterSize(1000);
      ec.setMinClusterSize(40);
      ec.setSearchMethod(ec_tree);
      ec.setInputCloud(plane_outlier_pts.makeShared());
      ec.extract(cluster_indices);

      pcl::PointCloud<pcl::PointXYZI> clusters_pcl;
      clusters_pcl.header = plane_outlier_pts.header;
      //clusters_pcl.height = plane_outlier_pts.height;
      clusters_pcl.is_dense = plane_outlier_pts.is_dense;

      int cluster_label = 0;
      std::cout<<"there are "<<cluster_indices.size()<<" clusters"<<endl;
      for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
      {
    	  cluster_label++;
    	  pcl::PointXYZI point_xyzi;
    	  std::cout<<"this cluster has "<<it->indices.size()<<" points"<<endl;
    	  for(std::vector<int>::const_iterator pit = it->indices.begin();pit!=it->indices.end();pit++)
    	  {
    		  point_xyzi.x = plane_outlier_pts.points[*pit].x;
    		  point_xyzi.y = plane_outlier_pts.points[*pit].y;
    		  point_xyzi.z = plane_outlier_pts.points[*pit].z;
    		  point_xyzi.intensity = cluster_label;
    		  clusters_pcl.points.push_back(point_xyzi);
    	  }
      }
      std::cout<<"there are "<<clusters_pcl.points.size()<<" points in the outliers"<<endl;
      //sensor_msgs::PointCloud2 clusters_pc2;
      //pcl::toROSMsg(clusters_pcl,clusters_pc2);
      ec_clusters_pub_.publish(clusters_pcl);
      sw.end();




}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "static_vehicle_recognition");
    golfcar_perception::static_vehicle_recognition svr;
	ros::spin();
	return 0;
}
