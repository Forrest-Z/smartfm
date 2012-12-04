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

geometry_msgs::Point32 ominus(geometry_msgs::Point32 point2, geometry_msgs::Point32 point1)
{
    double ctheta = cos(point1.z), stheta = sin(point1.z);
    geometry_msgs::Point32 relative_tf;
    relative_tf.x  = (point2.x - point1.x) * ctheta + (point2.y - point1.y) * stheta;
    relative_tf.y =  -(point2.x - point1.x) * stheta + (point2.y - point1.y) * ctheta;
    relative_tf.z = point2.z - point1.z;
    cout<<relative_tf<<endl;
	return relative_tf;
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

	int skip_reading = 5;
	uint scores_size = ceil(vec_no/skip_reading);
	for(size_t i=0; i<scores_size; i++)
	{
		vector<double> scores_temp;
		scores_temp.resize(scores_size);
		if(!readScores(*data_in, scores_temp))
		{
			cout<<"Unexpected end of data at score line "<<i<<endl;
			exit(1);
		}
		scores_array.push_back(scores_temp);
	}
	cout<<scores_array[0][0]<<" "<<scores_array[scores_size-1][scores_size-1]<<endl;
	cout<<"Successfully read all the scores"<<endl;

	int size = vec_no;
	vector<geometry_msgs::Point32> poses;
	if(!readFrontEndFile(*pc_data_in, pc_vec, poses)) cout<<"Failed read front end file"<<endl;

	if(size == (int)pc_vec.size())
		cout <<"All system green, going for matching"<<endl;
	else
		cout << "Failed in checking size of pc_vec and scores"<<endl;



	ros::Rate rate(2);
	for(int i=0; i<size; i+=skip_reading)
	{
		RasterMapPCL rmpcl;
		vector<geometry_msgs::Point32> combines_prior, prior_m5, prior_p5;
		combines_prior = pc_vec[i].points;
		//prior_m5 = getTransformedPts(ominus(poses[i-5], poses[i]), pc_vec[i-5].points);
		//prior_p5 = getTransformedPts(ominus(poses[i+5], poses[i]), pc_vec[i+5].points);
		//combines_prior.insert(combines_prior.begin(), prior_m5.begin(), prior_m5.end());
		//combines_prior.insert(combines_prior.begin(), prior_p5.begin(), prior_p5.end());
		rmpcl.setInputPts(combines_prior);
		for(int j=0; j<size; j+=skip_reading)
		{
			bool overwrite = false;

			if(j-i == 5) overwrite = true;
			if(j-i>0 && !overwrite) continue;

			if(abs(j-i)<20 && !overwrite) continue;

			//cout<<i<<":"<<j<<"      \xd"<<flush;

			if(scores_array[i/skip_reading][j/skip_reading]  > 70. || overwrite)
			{
				//putting rmpcl_ver declaration in line with the
				//rmpcl causes problem where getScore function produce inconsistent result, strange
				RasterMapPCL rmpcl_ver;
				transform_info best_tf = rmpcl.getBestTf(pc_vec[j]);

				//verification
				rmpcl_ver.setInputPts(best_tf.real_pts, true);
				double temp_score = rmpcl_ver.getScore(pc_vec[i].points);
				double ver_score = sqrt(temp_score * best_tf.score);

				src_pc.points = combines_prior;
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
				cout<<"Match found at "<<i<<" "<<j<<" with score "<<best_tf.score <<" recorded "<<scores_array[i/skip_reading][j/skip_reading] <<" ver_score "<<ver_score<<" "<<temp_score<<endl;

				//cout<<" cov_x="<<sqrt(cov.at<float>(0,0))<<" cov_y="<<sqrt(cov.at<float>(1,1))<<" cov_t="<<sqrt(cov.at<float>(2,2))/M_PI*180<<endl;
				//cout<<best_tf.translation_2d<<" "<< best_tf.rotation/M_PI*180<<endl;
				cout<<i<<" "<<j<<" "<<best_tf.translation_2d.x<<" "<<best_tf.translation_2d.y<<" "<<best_tf.rotation<<" ";
				cout<<cov.at<float>(0,0)<<" "<<cov.at<float>(0,1)<<" "<<cov.at<float>(0,2)<<" "<<cov.at<float>(1, 1)<<" "<<cov.at<float>(1,2)<<" "<<cov.at<float>(2,2);
				cout<<endl;
				cin >> enter_char;
				if(enter_char == 'x') return 0;
				//rate.sleep();
			}
		}
	}


	return 0;
}
