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

bool readPts(istream &in, sensor_msgs::PointCloud &p)            // read point (false on EOF)
{

	uint64_t time;
	int pc_size;

	if(!(in >> time)) return false;
	if(!(in >> pc_size)) return false;
	p.points.clear();
	p.points.resize(pc_size);

	for(int i=0; i<pc_size; i++)
	{
		if(!(in >> p.points[i].x))
		{
			cout<<"Error reading pt"<<endl;
			exit(1);
		}
		if(!(in >> p.points[i].y))
		{
			cout<<"Error reading pt"<<endl;
			exit(2);
		}
	}
	return true;
}


int main(int argc, char **argcv)
{
	ros::init(argc, argcv, "RasterMapImage");
	ros::NodeHandle nh;
	ros::Publisher src_pub, dst_pub, query_pub;
	src_pub = nh.advertise<sensor_msgs::PointCloud>("src_pc", 5);
	dst_pub = nh.advertise<sensor_msgs::PointCloud>("dst_pc", 5);
	query_pub = nh.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 5);
	sensor_msgs::PointCloud src_pc, dst_pc, query_pc;
	src_pc.header.frame_id = dst_pc.header.frame_id = query_pc.header.frame_id = "scan_odo";

	istream*        data_in         = NULL;         // input for data points

	vector<sensor_msgs::PointCloud> pc_vec;
	sensor_msgs::PointCloud pc;
	ifstream dataStreamSrc, dataStreamDst;
	dataStreamSrc.open(argcv[1], ios::in);// open data file
	if (!dataStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	data_in = &dataStreamSrc;

	while(readPts(*data_in, pc))
		pc_vec.push_back(pc);
	cout<<"Successfully read "<<pc_vec.size()<<" vectors of data pts"<<endl;



	int size = pc_vec.size();
	cv::Mat scores = cv::Mat::zeros(size, size, CV_32F);
fmutil::Stopwatch sw;
sw.start("matching...");
//#pragma omp parallel for
	for(int i=22; i<size; i++)
	{
		RasterMapPCL rmpcl;
		rmpcl.setInputPts(pc_vec[i]);
		for(int j=0; j<size; j++)
		{
			if(abs(j-i)<10) continue;
			cout<<i<<":"<<j<<'\xd'<<flush;
			transform_info best_tf = rmpcl.getBestTf(pc_vec[j]);
//#pragma omp critical
			scores.at<float>(i,j) = best_tf.score;

			if(best_tf.score > 55.)
			{
				src_pc.points = pc_vec[i].points;
				query_pc.points = pc_vec[j].points;

				cv::Mat cov = best_tf.covariance;
				dst_pc.points.clear();
				for(size_t k=0; k<best_tf.pts.size();k++)
				{

					geometry_msgs::Point32 pt;
					pt.x = best_tf.real_pts[k].x;
					pt.y = best_tf.real_pts[k].y;
					dst_pc.points.push_back(pt);
				}

				src_pc.header.stamp = dst_pc.header.stamp = query_pc.header.stamp = ros::Time::now();
				src_pub.publish(src_pc);
				dst_pub.publish(dst_pc);
				query_pub.publish(query_pc);
				ros::spinOnce();
				src_pub.publish(src_pc);
				dst_pub.publish(dst_pc);
				query_pub.publish(query_pc);
				ros::spinOnce();
				char enter_char;
				cout<<"Match found at "<<i<<" "<<j<<" with score "<<scores.at<float>(i,j);
				cout<<"cov_x="<<sqrt(cov.at<float>(0,0))<<" cov_y="<<sqrt(cov.at<float>(1,1))<<" cov_t="<<sqrt(cov.at<float>(2,2))/M_PI*180<<endl;
				cout<<best_tf.translation_2d<<" "<< best_tf.rotation/M_PI*180<<endl;
				cin >> enter_char;
			}
			//cout<<" "<<best_tf.score;
		}
		//cout<<endl;
	}
	sw.end();
	//cout<<endl;
	for(int i=0; i<size; i++)
	{
		for(int j=0; j<size; j++)
		{
			cout<<" "<<scores.at<float>(i,j);
		}
		cout<<endl;
	}






	return 0;
}
