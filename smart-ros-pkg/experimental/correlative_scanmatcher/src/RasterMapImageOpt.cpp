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
#include <fstream>
bool readPt(istream &in, cv::Point2f &p)            // read point (false on EOF)
{

	if(!(in >> p.x)) return false;
	if(!(in >> p.y)) return false;

	return true;
}


int main(int argc, char **argcv)
{
	istream*        data_in         = NULL;         // input for data points
	istream*        query_in         = NULL;         // input for query points
	vector<cv::Point2f> raster_pts, query_pts;
	cv::Point2f data_pts;
	ifstream dataStreamSrc, dataStreamDst;
	dataStreamSrc.open(argcv[1], ios::in);// open data file
	if (!dataStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	data_in = &dataStreamSrc;

	dataStreamDst.open(argcv[2], ios::in);// open data file
	if (!dataStreamDst) {
		cerr << "Cannot open query file\n";
		exit(1);
	}
	query_in = &dataStreamDst;

	while (readPt(*data_in, data_pts)) {
		raster_pts.push_back(data_pts);
	}
	cout << raster_pts.size() << "points read"<<endl;

	while (readPt(*query_in, data_pts)) {         // read query points
		query_pts.push_back(data_pts);
	}
	cout << query_pts.size() << "points read"<<endl;




	ros::init(argc, argcv, "RasterMapImage");
	ros::NodeHandle nh;
	ros::Publisher src_pub, dst_pub, query_pub;
	src_pub = nh.advertise<sensor_msgs::PointCloud>("src_pc", 5);
	dst_pub = nh.advertise<sensor_msgs::PointCloud>("dst_pc", 5);
	query_pub = nh.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 5);
	ros::Rate rate(10);

	sensor_msgs::PointCloud src_pc, dst_pc, query_pc;

	src_pc.header.frame_id = dst_pc.header.frame_id = query_pc.header.frame_id = "scan_odo";
	for(size_t i=0; i<raster_pts.size();i++)
	{
		geometry_msgs::Point32 pt;
		pt.x = raster_pts[i].x;
		pt.y = raster_pts[i].y;
		src_pc.points.push_back(pt);
	}


	for(size_t i=0; i<query_pts.size();i++)
	{
		geometry_msgs::Point32 pt;
		pt.x = query_pts[i].x;
		pt.y = query_pts[i].y;
		query_pc.points.push_back(pt);
	}

	fmutil::Stopwatch sw("overall");
	RasterMapPCL rmpcl,rmpcl_ver;
	rmpcl.setInputPts(src_pc.points);

	transform_info best_tf = rmpcl.getBestTf(query_pc);
	//verification
	rmpcl_ver.setInputPts(best_tf.real_pts, true);
	double temp_score = rmpcl_ver.getScore(src_pc.points);
	double ver_score = sqrt(temp_score * best_tf.score);
	sw.end();
	cv::Mat cov = best_tf.covariance;
			cout<<"score_scorev "<<best_tf.score<<"_"<<ver_score<<" cov_x="<<sqrt(cov.at<float>(0,0))<<" cov_y="<<sqrt(cov.at<float>(1,1))<<" cov_t="<<sqrt(cov.at<float>(2,2))/M_PI*180<<endl;
			cout<<best_tf.translation_2d<<" "<< best_tf.rotation<<endl;
	for(size_t i=0; i<best_tf.real_pts.size();i++)
	{
		geometry_msgs::Point32 pt;
		pt.x = best_tf.real_pts[i].x;
		pt.y = best_tf.real_pts[i].y;
		dst_pc.points.push_back(pt);
	}

	while(ros::ok())
	{
		src_pc.header.stamp = dst_pc.header.stamp = query_pc.header.stamp = ros::Time::now();
		src_pub.publish(src_pc);
		dst_pub.publish(dst_pc);
		query_pub.publish(query_pc);
		rate.sleep();
	}
	return 0;
}
