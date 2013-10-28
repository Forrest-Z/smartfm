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

template <class T>
pcl::PointCloud<T> pcl_downsample(pcl::PointCloud<T> query_pts, double size_x, double size_y, double size_z)
{
	pcl::PointCloud<T> point_cloud = query_pts;



	if(point_cloud.size()<100) return query_pts;
	//cout<<"Inside pcl downsample: "<<point_cloud[point_cloud.size()-1].z<<endl;
	pcl::VoxelGrid<T> sor;
	// always good not to use in place filtering as stated in
	// http://www.pcl-users.org/strange-effect-of-the-downsampling-td3857829.html
	pcl::PointCloud<T> input_msg_filtered = *(new pcl::PointCloud<T> ());
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
	return point_cloud;
}
vector<geometry_msgs::Point32> pcl_downsample(vector<geometry_msgs::Point32> &query_pts, double size_x, double size_y, double size_z)
				{
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	point_cloud.points.resize(query_pts.size());
	for(size_t i=0; i<query_pts.size(); i++)
	{
		point_cloud[i].x = query_pts[i].x;
		point_cloud[i].y = query_pts[i].y;
		point_cloud[i].z = query_pts[i].z;
	}
	point_cloud = pcl_downsample(point_cloud, size_x, size_y, size_z);
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



double normal_thres_ = 0.5;
double norm_radius_search_ = 0.2;
double density_radius_search_ = 0.2;
double density_min_neighbors_ = 5;

pcl::PointCloud<pcl::PointNormal> normalEstimation(pcl::PointCloud<pcl::PointXYZ>& input)
				{
	//if(input.points.size() == 0) return;
	//down sampling moved to rolling windows
	fmutil::Stopwatch sw;

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

	ne.setInputCloud (input.makeShared());

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal> cloud_normals;

	// Use all neighbors in a sphere of radius 10cm
	ne.setRadiusSearch (norm_radius_search_);

	// Set view point
	ne.setViewPoint(0.,0.,0.);//viewpoint[0], viewpoint[1], viewpoint[2]);
	// Compute the features
	ne.compute (cloud_normals);

	// concatentate the fileds
	pcl::PointCloud<pcl::PointNormal> point_normals;
	sw.start("Concatenate fields");
	pcl::concatenateFields(input, cloud_normals, point_normals);
	sw.end();
	//cout<<"Point normal "<<point_normals.points[0].normal_x<<' '<<point_normals.points[0].normal_y<<' '<<point_normals.points[0].normal_z<<endl;

	//publish normals as pose arrays if needed
	return point_normals;

				}

pcl::PointCloud<pcl::PointXYZ> filterHeight(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, double height, double range)
				{
	fmutil::Stopwatch sw;
	stringstream ss;
	ss<<"Filter clouds with "<<pcl_cloud.size()<<" pts";
	sw.start(ss.str());

	//absolutely stunningly quick!
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
			pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, height+range)));
	//range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	//      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
	pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
	condrem.setInputCloud (pcl_cloud.makeShared());
	condrem.filter (pcl_cloud);

	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond2 (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
			pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, height-range)));
	//range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	//      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
	pcl::ConditionalRemoval<pcl::PointXYZ> condrem2 (range_cond2);
	condrem2.setInputCloud (pcl_cloud.makeShared());
	condrem2.filter (pcl_cloud);
	sw.end();
	cout<<"After height filter "<<pcl_cloud.size()<<endl;
	return pcl_cloud;
				}

pcl::PointCloud<pcl::PointNormal> filterNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud)
				{
	fmutil::Stopwatch sw;
	stringstream ss;
	ss<<"Filter clouds with "<<pcl_cloud.size()<<" pts";
	sw.start(ss.str());

	//absolutely stunningly quick!
	pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointNormal> ());
	range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
			pcl::FieldComparison<pcl::PointNormal> ("normal_z", pcl::ComparisonOps::LT, normal_thres_)));
	//range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
	//      pcl::FieldComparison<pcl::PointNormal> ("z", pcl::ComparisonOps::GT, 0.0)));
	pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond);
	condrem.setInputCloud (pcl_cloud.makeShared());
	condrem.filter (pcl_cloud);

	pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond2 (new pcl::ConditionAnd<pcl::PointNormal> ());
	range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
			pcl::FieldComparison<pcl::PointNormal> ("normal_z", pcl::ComparisonOps::GT, -normal_thres_)));
	//range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
	//      pcl::FieldComparison<pcl::PointNormal> ("z", pcl::ComparisonOps::GT, 0.0)));
	pcl::ConditionalRemoval<pcl::PointNormal> condrem2 (range_cond2);
	condrem2.setInputCloud (pcl_cloud.makeShared());
	condrem2.filter (pcl_cloud);
	sw.end();
	cout<<"After normal filter "<<pcl_cloud.size()<<endl;

	if(pcl_cloud.size()==0) return pcl_cloud;

	//perform density based filtering
	sw.start("Density filtering");
	pcl::PointCloud<pcl::PointNormal> radius_filtered_pcl2;// = pcl_cloud;
	pcl::RadiusOutlierRemoval<pcl::PointNormal> outrem;
	outrem.setInputCloud(pcl_cloud.makeShared());
	outrem.setRadiusSearch(density_radius_search_);
	outrem.setMinNeighborsInRadius(density_min_neighbors_);
	outrem.filter(radius_filtered_pcl2);
	sw.end();
	cout<<"Remaining clouds: "<<radius_filtered_pcl2.size()<<endl;
	return radius_filtered_pcl2;
				}

vector<geometry_msgs::Point32> processNormals(vector<geometry_msgs::Point32> src)
				{
	sensor_msgs::PointCloud2 src_pc2;
	sensor_msgs::PointCloud out_pc;
	out_pc.points = src;
	pcl::PointCloud<pcl::PointXYZ> src_pcl;
	sensor_msgs::convertPointCloudToPointCloud2(out_pc, src_pc2);
	pcl::fromROSMsg(src_pc2, src_pcl);
	pcl::PointCloud<pcl::PointNormal> src_pointnorm = normalEstimation(src_pcl);
	src_pointnorm = filterNormal(src_pointnorm);

	pcl::toROSMsg(src_pointnorm, src_pc2);
	sensor_msgs::convertPointCloud2ToPointCloud(src_pc2, out_pc);
	cout<<"before downsample "<<out_pc.points.size()<<endl;
	out_pc.points = pcl_downsample(out_pc.points, 0.1, 0.1, 20.);
	cout<<"after downsample "<<out_pc.points.size()<<endl;
	return out_pc.points;
				}

void processNormals(vector<geometry_msgs::Point32> src, vector<geometry_msgs::Point32> &output)
{
	sensor_msgs::PointCloud2 src_pc2;
	sensor_msgs::PointCloud out_pc;
	out_pc.points = src;
	pcl::PointCloud<pcl::PointXYZ> src_pcl;
	sensor_msgs::convertPointCloudToPointCloud2(out_pc, src_pc2);
	pcl::fromROSMsg(src_pc2, src_pcl);
	pcl::PointCloud<pcl::PointNormal> src_pointnorm = normalEstimation(src_pcl);
	src_pointnorm = filterNormal(src_pointnorm);

	pcl::toROSMsg(src_pointnorm, src_pc2);
	sensor_msgs::convertPointCloud2ToPointCloud(src_pc2, out_pc);
	cout<<"before downsample "<<out_pc.points.size()<<endl;
	out_pc.points = pcl_downsample(out_pc.points, 0.1, 0.1, 20.);
	cout<<"after downsample "<<out_pc.points.size()<<endl;
	output = out_pc.points;
}
