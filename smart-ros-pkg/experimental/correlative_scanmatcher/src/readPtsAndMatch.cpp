/*
 * RasterMapImageOptSample.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: demian
 */

//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud.h>
//#include "pcl/ros/conversions.h"
#include <string>
#include <stdint.h>
#include <iostream>

#include "omp.h"
#include "ros_header.h"
#include "RasterMapPCL.h"
#include "ReadFileHelper.h"
#include "mysql_helper.h"
#include "geometry_helper.h"


int main(int argc, char **argcv)
{


	vector<string> input_files;
	vector<vector<sensor_msgs::PointCloud> > pc_vecs;
	vector<geometry_msgs::Pose> poses;
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



	int size = pc_vecs[0].size();

	fmutil::Stopwatch sw;
	sw.start("matching...");
	int skip_reading = 2;

	MySQLHelper sql(skip_reading, "scanmatch_result", argcv[1]);
	sql.create_2dtable(size);
	cout<<"pc_vec.size() "<<size<<endl;
	//omp_set_num_threads(6);
#pragma omp parallel for
	for(int i=0; i<size; i+=skip_reading)
	{
		double first_score = -1;
		#pragma omp critical
		first_score= sql.retrieve_first_score(i);
		if(first_score == -1)
		{
			RasterMapPCL rmpcl;
			vector<sensor_msgs::PointCloud> pc_vecs_3;
			//for(int j=0; j<3; j++) pc_vecs_3.push_back(pc_vecs[j][i]);
			//pc_vecs_3.clear();
			pc_vecs_3.resize(3);
			for(int k=-2; k<=0; k++)
			{
				if(i+k*skip_reading < 0 || i+k*skip_reading > size) continue;
				geometry_msgs::Pose odo = ominus(poses[i+k*skip_reading], poses[i]);
				for(int j=0; j<3; j++)
				{
					vector<geometry_msgs::Point32> input_pt_tran= getTransformedPts(odo, pc_vecs[j][i+k*skip_reading].points);
					pc_vecs_3[j].points.insert(pc_vecs_3[j].points.end(), input_pt_tran.begin(), input_pt_tran.end());
				}

			}
			rmpcl.setInputPts(pc_vecs_3);
			cout<<"Matching node "<<i<<endl;
			vector<double> score_row, rotation_row;
			//score_row.resize(size/skip_reading);
			for(int j=0; j<size; j+=skip_reading)
			{
				if(j>i) 
				{
					score_row.push_back(0.01);
					rotation_row.push_back(0.01);
					continue;
				}

				//if(abs(j-i)<5) continue;
				//cout<<i<<":"<<j<<"      \xd"<<flush;

				transform_info best_tf;
				vector<sensor_msgs::PointCloud> pc_vecs_3;
				for(int k=0; k<3; k++) pc_vecs_3.push_back(pc_vecs[k][j]);
				best_tf = rmpcl.getBestTf(pc_vecs_3);

				score_row.push_back(best_tf.score);
				rotation_row.push_back(best_tf.rotation);
				/*
			string s;



			if(best_tf.score > 55.)
			{
				cout<<"Match found at "<<i<<" "<<j<<" with score "<<best_tf.score <<" ver_score "<<ver_score<<" "<<temp_score<<endl;
				cv::Mat cov = best_tf.covariance;
				cout<<" cov_x="<<sqrt(cov.at<float>(0,0))<<" cov_y="<<sqrt(cov.at<float>(1,1))<<" cov_t="<<sqrt(cov.at<float>(2,2))/M_PI*180<<endl;
				cout<<best_tf.translation_2d<<" "<< best_tf.rotation/M_PI*180<<endl;

				src_pc.points = pc_vec[i].points;
				query_pc.points = pc_vec[j].points;


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


				getline(cin, s);
				if(s.size() > 0)
				{
					if(s[0] == 'x') return 0;

				}

			}*/
				//cout<<" "<<best_tf.score;
			}
			//cout<<"Size="<<score_row.size()<<endl;
			/*for(int k=0; k<score_row.size(); k++)
			cout<<score_row[k]<<" ";
		cout<<endl;*/
#pragma omp critical
			{
				sql.update_data(score_row, i);
				sql.update_rotation(rotation_row, i);
			}
		}
	}
	sw.end();
	//cout<<endl;







	return 0;
}
