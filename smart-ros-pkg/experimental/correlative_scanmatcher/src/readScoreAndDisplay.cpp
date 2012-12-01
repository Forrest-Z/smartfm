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

bool readHeader(istream &in, double& vec_no)
{
	string temp;
	in >> temp; in>> temp;
	in >> vec_no;
	for(int i=0; i<7; i++)
		in >> temp;
	cout<<vec_no<<endl;
	if(vec_no>0) return true;
	else return false;
}

bool readScores(istream &in, vector<double>& scores)            // read point (false on EOF)
{
	for(size_t i=0; i<scores.size(); i++)
	{
		if(!(in >> scores[i]))
		{
			cout<<"Read scores stop at "<<i<<endl;
			return false;
		}
		//cout<<scores[i]<<" ";
	}
	//cout<<endl;
	//cout<<"First score:"<<scores[0]<< " last score:"<<scores[scores.size()]<<endl;
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

	istream* data_in = NULL, *pc_data_in = NULL;         // input for data points

	vector<sensor_msgs::PointCloud> pc_vec;
	sensor_msgs::PointCloud pc;
	ifstream dataStreamSrc, pcStreamSrc;
	dataStreamSrc.open(argcv[1], ios::in);// open data file
	if (!dataStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	pcStreamSrc.open(argcv[2], ios::in);// open data file
	if (!pcStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	data_in = &dataStreamSrc;
	pc_data_in = &pcStreamSrc;

	double vec_no;
	readHeader(*data_in, vec_no);
	vector< vector<double> > scores_array;

	for(size_t i=0; i<vec_no; i++)
	{
		vector<double> scores_temp;
		scores_temp.resize(vec_no);
		if(!readScores(*data_in, scores_temp))
		{
			cout<<"Unexpected end of data at score line "<<i<<endl;
			exit(1);
		}
		scores_array.push_back(scores_temp);
	}

	cout<<"Successfully read all the scores"<<endl;

	int size = vec_no;

	if(!readFrontEndFile(*pc_data_in, pc_vec)) cout<<"Failed read front end file"<<endl;

	if(size == (int)pc_vec.size())
		cout <<"All system green, going for matching"<<endl;
	else
		cout << "Failed in checking size of pc_vec and scores"<<endl;

	int skip_reading = 1;

	ros::Rate rate(2);
	for(int i=0; i<size; i+=skip_reading)
	{
		RasterMapPCL rmpcl;
		rmpcl.setInputPts(pc_vec[i]);
		for(int j=0; j<size; j+=skip_reading)
		{
			//skip the first scan, only contain straight line
			if(j==0) continue;
			if(abs(j-i)<10) continue;
			//cout<<i<<":"<<j<<"      \xd"<<flush;

			if(scores_array[i][j]  > 80.)
			{
				transform_info best_tf = rmpcl.getBestTf(pc_vec[j]);
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

				for(int k=0; k<3; k++)
				{
					src_pub.publish(src_pc);
					dst_pub.publish(dst_pc);
					query_pub.publish(query_pc);
					ros::spinOnce();
				}

				char enter_char;
				cout<<"Match found at "<<i<<" "<<j<<" with score "<<best_tf.score <<" recorded "<<scores_array[i][j];
				cout<<" Cov "<<cov;
				cout<<" cov_x="<<sqrt(cov.at<float>(0,0))<<" cov_y="<<sqrt(cov.at<float>(1,1))<<" cov_t="<<sqrt(cov.at<float>(2,2))/M_PI*180<<endl;
				cout<<best_tf.translation_2d<<" "<< best_tf.rotation/M_PI*180<<endl;
				//cin >> enter_char;
				//if(enter_char == 'x') return 0;
				rate.sleep();
			}
		}
	}


	return 0;
}
