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
#include "mysql_helper.h"


int main(int argc, char **argcv)
{
	ros::init(argc, argcv, "readPtsAndMatch");
	ros::NodeHandle nh;
	ros::Publisher src_pub, dst_pub, query_pub;
	src_pub = nh.advertise<sensor_msgs::PointCloud>("src_pc", 5);
	dst_pub = nh.advertise<sensor_msgs::PointCloud>("dst_pc", 5);
	query_pub = nh.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 5);
	sensor_msgs::PointCloud src_pc, dst_pc, query_pc;
	src_pc.header.frame_id = dst_pc.header.frame_id = query_pc.header.frame_id = "scan_odo";

	istream*        data_in         = NULL;         // input for data points



	ifstream dataStreamSrc, dataStreamDst;
	dataStreamSrc.open(argcv[1], ios::in);// open data file
	if (!dataStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	data_in = &dataStreamSrc;

	vector<sensor_msgs::PointCloud> pc_vec;
	if(!readFrontEndFile(*data_in, pc_vec)) cout<<"Failed read front end file"<<endl;



	int size = pc_vec.size();
	cv::Mat scores = cv::Mat::zeros(size, size, CV_32F);
	fmutil::Stopwatch sw;
	sw.start("matching...");
	int skip_reading = 5;

	MySQLHelper sql(skip_reading, "scanmatch_result", argcv[1]);
	sql.create_2dtable(size);
	cout<<"pc_vec.size() "<<size<<endl;
	omp_set_num_threads(5);
#pragma omp parallel for
	for(int i=0; i<size; i+=skip_reading)
	{
		double first_score = -1;
		#pragma omp critical
		first_score= sql.retrieve_first_score(i);
		if(first_score == -1)
		{
			RasterMapPCL rmpcl;
			rmpcl.setInputPts(pc_vec[i].points);
			cout<<"Matching node "<<i<<endl;
			vector<double> score_row;
			//score_row.resize(size/skip_reading);
			for(int j=0; j<size; j+=skip_reading)
			{
				if(j>i) 
				{
					score_row.push_back(0.01);
					continue;
				}
				//if(abs(j-i)<5) continue;
				//cout<<i<<":"<<j<<"      \xd"<<flush;

				transform_info best_tf;
				best_tf = rmpcl.getBestTf(pc_vec[j]);
				RasterMapPCL rmpcl_ver;
				rmpcl_ver.setInputPts(best_tf.real_pts, true);
				double temp_score = rmpcl_ver.getScore(pc_vec[i].points);
				double ver_score = sqrt( temp_score * best_tf.score);


				score_row.push_back(ver_score);

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
			sql.update_data(score_row, i);
		}
	}
	sw.end();
	//cout<<endl;







	return 0;
}
