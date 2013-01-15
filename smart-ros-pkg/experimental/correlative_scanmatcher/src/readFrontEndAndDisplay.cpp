/*
 * RasterMapImageOptSample.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "pcl/ros/conversions.h"
#include "RasterMapPCL.h"
#include "ReadFileHelper.h"
#include <isam/isam.h>
#include "GraphPF.h"
#include "mysql_helper.h"

vector<geometry_msgs::Point32> getTransformedPts(geometry_msgs::Point32 pose, vector<geometry_msgs::Point32>& pts)
{
	double ct = cos(pose.z), st = sin(pose.z);
	vector<geometry_msgs::Point32> final_pt;
	final_pt.resize(pts.size());
	for(size_t j=0; j<pts.size(); j++)
	{

		geometry_msgs::Point32 pt = pts[j], rot_pt;
		rot_pt.x = ct * pt.x - st * pt.y + pose.x;
		rot_pt.y = st * pt.x + ct * pt.y + pose.y;
		final_pt[j] = rot_pt;
	}
	return final_pt;
}

vector<geometry_msgs::Point32> getTransformedPts(geometry_msgs::Pose pose, vector<geometry_msgs::Point32>& pts)
				{
	double ct = cos(pose.orientation.z), st = sin(pose.orientation.z);
	vector<geometry_msgs::Point32> final_pt;
	final_pt.resize(pts.size());
	for(size_t j=0; j<pts.size(); j++)
	{

		geometry_msgs::Point32 pt = pts[j], rot_pt;
		rot_pt.x = ct * pt.x - st * pt.y + pose.position.x;
		rot_pt.y = st * pt.x + ct * pt.y + pose.position.y;
		rot_pt.z = pose.position.z;
		final_pt[j] = rot_pt;
	}
	return final_pt;
				}
geometry_msgs::Point32 ominus(geometry_msgs::Point32 point2, geometry_msgs::Point32 point1)
{
	double ctheta = cos(point1.z), stheta = sin(point1.z);
	geometry_msgs::Point32 relative_tf;
	relative_tf.x  = (point2.x - point1.x) * ctheta + (point2.y - point1.y) * stheta;
	relative_tf.y =  -(point2.x - point1.x) * stheta + (point2.y - point1.y) * ctheta;
	relative_tf.z = point2.z - point1.z;
	//cout<<relative_tf<<endl;
	return relative_tf;
}

geometry_msgs::Pose ominus(geometry_msgs::Pose point2, geometry_msgs::Pose point1)
{
	double ctheta = cos(point1.orientation.z), stheta = sin(point1.orientation.z);
	geometry_msgs::Pose relative_tf;
	relative_tf.position.x  = (point2.position.x - point1.position.x) * ctheta + (point2.position.y - point1.position.y) * stheta;
	relative_tf.position.y =  -(point2.position.x - point1.position.x) * stheta + (point2.position.y - point1.position.y) * ctheta;
	relative_tf.position.z = point2.position.z - point1.position.z;
	relative_tf.orientation.z = point2.orientation.z - point1.orientation.z;
	relative_tf.orientation.y = point2.orientation.y - point1.orientation.y;
	//cout<<relative_tf<<endl;
	return relative_tf;
}

int main(int argc, char **argcv)
{

	fmutil::Stopwatch sw;
	sw.start("isam_full");
	ros::init(argc, argcv, "RasterMapImage");
	ros::NodeHandle nh;
	ros::Publisher src_pub, dst_pub, query_pub, overall_pub;
	src_pub = nh.advertise<sensor_msgs::PointCloud>("src_pc", 5);
	dst_pub = nh.advertise<sensor_msgs::PointCloud>("dst_pc", 5);
	query_pub = nh.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 5);
	overall_pub = nh.advertise<sensor_msgs::PointCloud>("pc_graph_overall", 5);
	sensor_msgs::PointCloud src_pc, dst_pc, query_pc;
	src_pc.header.frame_id = dst_pc.header.frame_id = query_pc.header.frame_id = "scan_odo";

	istream *pc_data_in = NULL;         // input for data points

	vector<sensor_msgs::PointCloud> pc_vec;
	sensor_msgs::PointCloud pc;
	ifstream dataStreamSrc, pcStreamSrc;

	int skip_reading = 1;


	pcStreamSrc.open(argcv[1], ios::in);// open data file
	if (!pcStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	pc_data_in = &pcStreamSrc;
	vector<geometry_msgs::Pose> poses;
	if(!readFrontEndFile(*pc_data_in, pc_vec, poses)) cout<<"Failed read front end file"<<endl;

	sensor_msgs::PointCloud overall_pts;
	overall_pts.header.frame_id = "scan_odo";

	for(size_t i=skip_reading; i<pc_vec.size(); i+=skip_reading)
	{

		geometry_msgs::Pose estimated_pt;
		estimated_pt = poses[i/skip_reading];
		vector<geometry_msgs::Point32> tfed_pts = getTransformedPts(estimated_pt, pc_vec[i/skip_reading].points);
		overall_pts.points.insert(overall_pts.points.end(), tfed_pts.begin(), tfed_pts.end());

	}
	fmutil::Stopwatch sws_ds("downsampling", true);
	RasterMapImage rmi(1,1);
	vector<geometry_msgs::Point32> downsampled_pt = rmi.pcl_downsample(overall_pts.points, 0.05, 0.05, 0.05);

	overall_pts.points = downsampled_pt;
	cout<<"After downsampling = "<<overall_pts.points[overall_pts.points.size()-1].z<<endl;

	for(int k=0; k<3; k++)
	{
		overall_pts.header.stamp = ros::Time::now();
		overall_pub.publish(overall_pts);

		ros::spinOnce();
	}
	RenderMap rm;
	rm.drawMap(overall_pts.points, 0.1, "rawfrontend_map.png");
	sw.end();
	cout<<"***********************************"<<endl;


	return 0;
}
