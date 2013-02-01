/*
 * RasterMapImageOptSample.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <fmutil/fm_stopwatch.h>
#include <fmutil/fm_math.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "RasterMapPCL.h"
#include <fstream>
#include <boost/thread/thread.hpp>
#include "pcl_downsample.h"
pcl::PointCloud<pcl::PointNormal> input_cloud, matching_cloud;
double res_ = 0.1;
RasterMapImage rmi(res_, 0.2);

boost::shared_ptr<pcl::visualization::PCLVisualizer> initVisualizer ()
				{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	return (viewer);
				}

void addPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, pcl::PointCloud<pcl::PointNormal> &cloud_in, string cloud_name, string normal_name, bool red_color)
{
	cout<<cloud_in.points.size()<<" loaded."<<endl;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> *color;
	if(red_color) color = new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_in.makeShared(), 255, 0, 0);
	else color = new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_in.makeShared(), 0, 255, 0);
	viewer->addPointCloud<pcl::PointNormal> (cloud_in.makeShared(), *color, cloud_name);
	viewer->addPointCloudNormals<pcl::PointNormal>(cloud_in.makeShared(), 2 ,0.1, normal_name);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
}

double offset_x, offset_y, rotation=180.;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
		void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if(event.keyDown())
	{
		viewer->removePointCloud("matching_cloud");
		viewer->removePointCloud("matching_normal_cloud");
		string keySym = event.getKeySym();
		if(keySym == "j") offset_y-=res_;
		else if(keySym == "l") offset_y+=res_;
		else if(keySym == "k") offset_x-=res_;
		else if(keySym == "i") offset_x+=res_;
		else if(keySym == "y") rotation++;
		else if(keySym == "p") rotation--;
		vector<cv::Point2f> search_pt;
		vector<double> normal_pt;
		pcl::PointCloud<pcl::PointNormal> matching_cloud_tf = matching_cloud;
		double yaw_rotate = rotation/180.*M_PI;
		Eigen::Quaternionf bl_rotation (cos(yaw_rotate/2.), 0, 0, -sin(yaw_rotate/2.) );
		Eigen::Vector3f bl_trans(offset_x, offset_y, 0.);
		pcl_utils::transformPointCloudWithNormals<pcl::PointNormal>(matching_cloud_tf, matching_cloud_tf, bl_trans, bl_rotation);
		search_pt.resize(matching_cloud_tf.points.size());
		normal_pt.resize(matching_cloud_tf.points.size());
		for(size_t i=0; i<matching_cloud_tf.points.size(); i++)
		{
			search_pt[i].x = matching_cloud_tf.points[i].x;
			search_pt[i].y = matching_cloud_tf.points[i].y;
			normal_pt[i] = atan2(matching_cloud_tf.points[i].normal_y, matching_cloud_tf.points[i].normal_x);
		}
		ScoreDetails sd;
		double best_score = rmi.getScoreWithNormal(search_pt, normal_pt, sd);

		cout<<"Distance:"<<sd.dist_score<<" Norm:"<< sd.normal_score<<" NNorm:"<<sd.norm_norm_score<<
							" WorstNorm:"<<sd.worst_norm_score<<" Final:"<<best_score<<endl;

		addPointCloud(viewer, matching_cloud_tf, "matching_cloud", "matching_normal_cloud", false);
		cout<<offset_x<<" "<<offset_y<<" "<<rotation<<endl;

	}
}

void bruteForceSearch()
{
	cout<<"Inside bruteForceSearch"<<endl;
	double best_x, best_y;
	int best_rotation;
	double best_score = 0;
	ScoreDetails best_score_details;
	for(int i=160; i<200; i+=2)
	{
		pcl::PointCloud<pcl::PointNormal> matching_cloud_tf = matching_cloud;
		double yaw_rotate = i/180.*M_PI;
		Eigen::Vector3f bl_trans(0, 0, 0.);
		Eigen::Quaternionf bl_rotation (cos(yaw_rotate/2.), 0, 0, -sin(yaw_rotate/2.) );
		pcl_utils::transformPointCloudWithNormals<pcl::PointNormal>(matching_cloud_tf, matching_cloud_tf, bl_trans, bl_rotation);
    //matching_cloud_tf = pcl_downsample(matching_cloud_tf, 0.5, 0.5, 0.5);
		vector<double> angular_normals; angular_normals.resize(matching_cloud_tf.size());
		for(size_t idx=0; idx<matching_cloud_tf.size(); idx++)
		{
			angular_normals[idx] = atan2(matching_cloud_tf.points[idx].normal_y, matching_cloud_tf.points[idx].normal_x);
		}
		for(double j=-15; j<15; j+=0.5)//=res_)
		{
			for(double k=-15; k<15; k+=0.5)//res_)
			{
				vector<cv::Point2f> search_pt;
				search_pt.resize(matching_cloud_tf.points.size());
				for(size_t idx=0; idx<matching_cloud_tf.points.size(); idx++)
				{
					search_pt[idx].x = matching_cloud_tf.points[idx].x+j;
					search_pt[idx].y = matching_cloud_tf.points[idx].y+k;
				}
				ScoreDetails sd;
				double score = rmi.getScoreWithNormal(search_pt, angular_normals, sd);
				if(score>best_score)
				{
					best_x = j;
					best_y = k;
					best_rotation = i;
					best_score = score;
					best_score_details = sd;
				}

			}
			cout<<setiosflags (ios::fixed | ios::showpoint | ios::right) <<setprecision(3)<<setfill('0')<<setw(3)<<"At offset "<<j<<" "<<" "<<i<<", best tf found so far: "<<best_x<<" "<<best_y<<" "<<best_rotation<<
					" Distance:"<<best_score_details.dist_score<<" Norm:"<< best_score_details.normal_score<<" NNorm:"<<best_score_details.norm_norm_score<<
					" WorstNorm:"<<best_score_details.worst_norm_score<<" Final:"<<best_score<<"      \xd"<<flush;
		}


	}
	cout<<endl;
	offset_x = best_x;
	offset_y = best_y;
	rotation = best_rotation;
}

map<int,int> getHistogram(pcl::PointCloud<pcl::PointNormal> &pcl)
{
	map<int,int> histogram;
	for(size_t i=0; i<pcl.points.size(); i++)
	{
		histogram[int(atan2(pcl.points[i].normal_y, pcl.points[i].normal_x)/M_PI*360)]++;
	}
	for(int i=-360; i<=360; i++)
	{
		if(histogram.find(i) == histogram.end())
			histogram[i] = 0;
	}
	return histogram;

}

double getHistogramSSE(map<int,int> &hist1, map<int,int> &hist2)
{
	assert(hist1.size() == hist2.size());
	double error = 0;
	for(map<int,int>::iterator it1 = hist1.begin(), it2 = hist2.begin(); it1!=hist1.end(); it1++, it2++)
	{
		double temp = it1->second - it2->second;
		error += temp*temp;
	}
	return error;
}

void getBestRotationWithHistogram()
{
	map<int,int> input_hist = getHistogram(input_cloud);
	vector<map<int,int> > histograms;
	double smallest_error = 1e999;
	int best_rotation=-360;
	for(int i=-180; i<180; i+=2)
	{
		pcl::PointCloud<pcl::PointNormal> matching_cloud_tf = matching_cloud;
		double yaw_rotate = i/180.*M_PI;
		Eigen::Vector3f bl_trans(0, 0, 0.);
		Eigen::Quaternionf bl_rotation (cos(yaw_rotate/2.), 0, 0, -sin(yaw_rotate/2.) );
		pcl_utils::transformPointCloudWithNormals<pcl::PointNormal>(matching_cloud_tf, matching_cloud_tf, bl_trans, bl_rotation);
		map<int,int> matching_hist = getHistogram(matching_cloud_tf);
		double error_now = getHistogramSSE(matching_hist, input_hist);
		cout<<error_now<<" ";
		if( error_now < smallest_error)
		{
			smallest_error = error_now;
			best_rotation = i;
		}
	}
	cout<<endl;
	cout<<"Best rotation found: "<<best_rotation<<" with error: "<<smallest_error<<endl;
}
#include "readfrontend.h"
int main(int argc, char **argv)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = initVisualizer ();
  
	pcl::io::loadPCDFile(argv[2], input_cloud);
  append_input_cloud(input_cloud, string(argv[1]), string(argv[2]));
	pcl::io::loadPCDFile(argv[3], matching_cloud);

	input_cloud = pcl_downsample(input_cloud, res_,res_,res_);
	matching_cloud = pcl_downsample(matching_cloud, res_,res_,res_);
	addPointCloud(viewer, input_cloud, "input_cloud", "input_normal_cloud", true);

	vector<cv::Point2f> input_pts, input_normals;
	input_pts.resize(input_cloud.points.size());
	input_normals.resize(input_cloud.points.size());
	for(size_t i=0; i<input_cloud.points.size(); i++)
	{
		input_pts[i].x = input_cloud.points[i].x;
		input_pts[i].y = input_cloud.points[i].y;
		input_normals[i].x = input_cloud.points[i].normal_x;
		input_normals[i].y = input_cloud.points[i].normal_y;
	}
	rmi.getInputPoints(input_pts, input_normals);
	bruteForceSearch();
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);


	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	return 0;
}
