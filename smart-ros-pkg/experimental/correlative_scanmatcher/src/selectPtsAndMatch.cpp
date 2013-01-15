/*
 * RasterMapImageOptSample.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include "pcl/ros/conversions.h"
#include "RasterMapPCL.h"
#include "ReadFileHelper.h"
#include <isam/isam.h>
#include "mysql_helper.h"
#include "geometry_helper.h"


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
	  vector<geometry_msgs::Pose> poses;
	vector<vector<sensor_msgs::PointCloud> > pc_vecs;
	for(int i=0; i<3; i++)
	{
		stringstream ss;
		ss<<argcv[1]<<"_"<<i;
		istream*        data_in         = NULL;         // input for data points
		ifstream dataStreamSrc;
		cout<<"Opening "<<ss.str()<<endl;
		dataStreamSrc.open(ss.str().c_str(), ios::in);// open data file
		if (!dataStreamSrc) {
			cerr << "Cannot open data file\n";
			exit(1);
		}
		data_in = &dataStreamSrc;

		vector<sensor_msgs::PointCloud> pc_vec;
		if(!readFrontEndFile(*data_in, pc_vec, poses)) cout<<"Failed read front end file"<<endl;
		pc_vecs.push_back(pc_vec);
	}
	assert(pc_vecs[0].size() == pc_vecs[1].size());
	assert(pc_vecs[0].size() == pc_vecs[2].size());
	ros::Rate rate(2);

	bool repeat = true;

	if(argc > 2)
	{
		int source_pts_idx = atoi(argcv[2]);
		int dest_pts_idx = atoi(argcv[3]);
		RasterMapPCL rmpcl;
		vector<sensor_msgs::PointCloud> input_pcs, dst_pcs;
		fmutil::Stopwatch sw("pushes pts");
		for(int i=0; i<3; i++) 
		{
			input_pcs.push_back(pc_vecs[i][source_pts_idx]);
			dst_pcs.push_back(pc_vecs[i][dest_pts_idx]);
		}
		sw.end();
		sw.start("Set input");
		rmpcl.setInputPts(input_pcs);
		sw.end();
		sw.start("getbesttf");
		RasterMapPCL rmpcl_ver;
		transform_info best_tf = rmpcl.getBestTf(dst_pcs);
		sw.end();
		sw.start("verification");
		//verification
		rmpcl_ver.setInputPts(best_tf.real_pts, true);
		double temp_score = rmpcl_ver.getScore(pc_vecs[2][source_pts_idx].points);
		double ver_score = sqrt(temp_score * best_tf.score);
		sw.end();
		sw.start("setting up ros output");
		src_pc.points = pc_vecs[2][source_pts_idx].points;
		query_pc.points = pc_vecs[2][dest_pts_idx].points;
		src_pc.header.frame_id = dst_pc.header.frame_id = query_pc.header.frame_id = "scan_odo";
		src_pc.header.stamp = dst_pc.header.stamp = query_pc.header.stamp = ros::Time::now();

		cv::Mat cov = best_tf.covariance;
		dst_pc.points.clear();
		for(size_t k=0; k<best_tf.real_pts.size();k++)
		{

			geometry_msgs::Point32 pt;
			pt.x = best_tf.real_pts[k].x;
			pt.y = best_tf.real_pts[k].y;
			dst_pc.points.push_back(pt);
		}
		sw.end();
		cout<<"Total time: "<<sw.total_/1000<<" ms"<<endl;
		for(int k=0; k<3; k++)
		{
			src_pub.publish(src_pc);
			dst_pub.publish(dst_pc);
			query_pub.publish(query_pc);
			ros::spinOnce();
		}

		cout<<"Match found at "<<source_pts_idx<<" "<<dest_pts_idx<<" with score "<<best_tf.score <<" ver_score "<<ver_score<<" "<<temp_score<<endl;
		cout<<best_tf.translation_2d.x<<" "<<best_tf.translation_2d.y<<" "<<best_tf.rotation<<" ";
		cout<<cov.at<float>(0,0)<<" "<<cov.at<float>(0,1)<<" "<<cov.at<float>(0,2)<<" "<<cov.at<float>(1, 1)<<" "<<cov.at<float>(1,2)<<" "<<cov.at<float>(2,2);
		cout<<endl;
	}
	else
	{
		while(repeat)
		{
			int source_pts_idx, dest_pts_idx;
			bool source_repeat = true, dst_repeat = true;
			vector<sensor_msgs::PointCloud> input_pc;
			while(source_repeat)
			{
				cout<<"Please select a matching source: (0-"<<pc_vecs[0].size()-1<<"): use y to confirm "<<flush;

				string matching_pts;
				getline(cin, matching_pts);
				if(matching_pts.size()>0)
				{
					if(matching_pts[0] == 'y')
					{
						source_repeat = false;
						continue;
					}
					if(matching_pts[0] == 'x')
						exit(0);
					source_pts_idx = atoi(matching_pts.c_str());
				}
				else source_pts_idx++;
				cout<<"Matching source "<<source_pts_idx<<" selected"<<endl;
				if(input_pc.size()==3)
					for(int i=0; i<3; i++) input_pc[i].points.clear();
				sensor_msgs::PointCloud src_pc;

				input_pc.resize(3);
				for(int i=-1; i<=0; i++)
				{
					geometry_msgs::Pose odo = ominus(poses[source_pts_idx+i*2], poses[source_pts_idx]);
					for(int j=0; j<3; j++)
					{
						vector<geometry_msgs::Point32> input_pt_tran= getTransformedPts(odo, pc_vecs[j][source_pts_idx+i*2].points);
						input_pc[j].points.insert(input_pc[j].points.end(), input_pt_tran.begin(), input_pt_tran.end());
					}

				}
				dst_pc.points = input_pc[1].points;
				query_pc.points = input_pc[0].points;
				src_pc.points = input_pc[2].points;

				dst_pc.header.frame_id = query_pc.header.frame_id = src_pc.header.frame_id = "scan_odo";
				dst_pc.header.stamp = query_pc.header.stamp = src_pc.header.stamp = ros::Time::now();

				for(int i=0; i<3; i++)
				{
					src_pub.publish(src_pc);
					dst_pub.publish(dst_pc);
					query_pub.publish(query_pc);
					ros::spinOnce();
				}

			}
			RasterMapPCL rmpcl;

			rmpcl.setInputPts(input_pc);

			while(dst_repeat)
			{
				cout<<"Please select a matching destination (0-"<<pc_vecs[0].size()-1<<"): use x to exit "<<flush;
				string matching_pts;
				getline(cin, matching_pts);
				if(matching_pts.size()>0)
				{
					if(matching_pts[0] == 'x')
					{
						dst_repeat = false;
						continue;
					}
					dest_pts_idx = atoi(matching_pts.c_str());
				}
				else dest_pts_idx++;
				cout<<"Destination source "<<dest_pts_idx<<" selected"<<endl;
				sensor_msgs::PointCloud dst_pc;



				fmutil::Stopwatch sw("matching...");
				vector<sensor_msgs::PointCloud> dest_pc;
				for(int i=0; i<3; i++) dest_pc.push_back(pc_vecs[i][dest_pts_idx]);
				transform_info best_tf = rmpcl.getBestTf(dest_pc);

				sw.end();
				query_pc.points = pc_vecs[2][dest_pts_idx].points;
				dst_pc.header.frame_id = query_pc.header.frame_id = "scan_odo";
				dst_pc.header.stamp = query_pc.header.stamp = ros::Time::now();

				cv::Mat cov = best_tf.covariance;
				dst_pc.points.clear();
				for(size_t k=0; k<best_tf.real_pts.size();k++)
				{

					geometry_msgs::Point32 pt;
					pt.x = best_tf.real_pts[k].x;
					pt.y = best_tf.real_pts[k].y;
					dst_pc.points.push_back(pt);
				}

				for(int k=0; k<3; k++)
				{
					src_pub.publish(src_pc);
					dst_pub.publish(dst_pc);
					query_pub.publish(query_pc);
					ros::spinOnce();
				}

				cout<<"Match found at "<<source_pts_idx<<" "<<dest_pts_idx<<" with score "<<best_tf.score<<endl;
				cout<<best_tf.translation_2d.x<<" "<<best_tf.translation_2d.y<<" "<<best_tf.rotation<<" ";
				cout<<cov.at<float>(0,0)<<" "<<cov.at<float>(0,1)<<" "<<cov.at<float>(0,2)<<" "<<cov.at<float>(1, 1)<<" "<<cov.at<float>(1,2)<<" "<<cov.at<float>(2,2);
			}
		}
	}

	return 0;
}
