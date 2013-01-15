/*
 * pcl_downsample.h
 *
 *  Created on: Dec 28, 2012
 *      Author: demian
 */


#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>


using namespace std;


vector<geometry_msgs::Point32> pcl_downsample(vector<geometry_msgs::Point32> &query_pts, double size_x, double size_y, double size_z)
			{
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	point_cloud.resize(query_pts.size());
	for(size_t i=0; i<query_pts.size(); i++)
	{
		point_cloud[i].x = query_pts[i].x;
		point_cloud[i].y = query_pts[i].y;
		point_cloud[i].z = query_pts[i].z;
	}



	if(point_cloud.size()<100) return query_pts;
	//cout<<"Inside pcl downsample: "<<point_cloud[point_cloud.size()-1].z<<endl;
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	// always good not to use in place filtering as stated in
	// http://www.pcl-users.org/strange-effect-of-the-downsampling-td3857829.html
	pcl::PointCloud<pcl::PointXYZ> input_msg_filtered = *(new pcl::PointCloud<pcl::PointXYZ> ());
	//float downsample_size_ = 0.05;
	sor.setInputCloud(point_cloud.makeShared());
	sor.setLeafSize (size_x, size_y, size_z);
	pcl::PointIndicesPtr pi;
	sor.filter (input_msg_filtered);
	/*dbg<<"Downsampled colors ";
		        for(size_t i=0; i<input_msg_filtered->points.size(); i++)
		        {
		            dbg<<input_msg_filtered->points[i].rgb<<" ";
		        }
		        dbg<<endl;*/
	point_cloud = input_msg_filtered;
	//cout<<"After pcl downsample: "<<point_cloud[point_cloud.size()-1].z<<endl;
	vector<geometry_msgs::Point32> after_downsample;
	after_downsample.resize(point_cloud.size());
	for(size_t i=0; i<point_cloud.size(); i++)
	{
		after_downsample[i].x = point_cloud[i].x;
		after_downsample[i].y = point_cloud[i].y;
		after_downsample[i].z = point_cloud[i].z;
	}
	return after_downsample;

			}

vector<cv::Point2f> pcl_downsample(vector<cv::Point2f> const &query_pts, double size_x, double size_y, double size_z)
			{
	vector<geometry_msgs::Point32> input_pt, output_pt;
	vector<cv::Point2f> output_2d_pt;
	input_pt.resize(query_pts.size());
	for(size_t i=0; i<query_pts.size(); i++)
	{
		input_pt[i].x = query_pts[i].x;
		input_pt[i].y = query_pts[i].y;
	}
	output_pt = pcl_downsample(input_pt, size_x, size_y, size_z);
	output_2d_pt.resize(output_pt.size());
	for(size_t i=0; i<output_pt.size(); i++)
	{
		output_2d_pt[i].x = output_pt[i].x;
		output_2d_pt[i].y = output_pt[i].y;
	}
	return output_2d_pt;
			}

