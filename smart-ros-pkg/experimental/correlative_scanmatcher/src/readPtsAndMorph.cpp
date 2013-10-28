/*
 * RasterMapImageOptSample.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: demian
 */

//#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include "pcl/ros/conversions.h"
#include "RasterMapPCL.h"
#include "ReadFileHelper.h"
#include "pcl_downsample.h"
#include "geometry_helper.h"

int main(int argc, char **argcv)
{

	istream*        data_in         = NULL;         // input for data points
	ifstream dataStreamSrc, dataStreamDst;
	dataStreamSrc.open(argcv[1], ios::in);// open data file
	if (!dataStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	data_in = &dataStreamSrc;

	vector<sensor_msgs::PointCloud> pc_vec;
	vector<geometry_msgs::Pose> poses;
	vector<uint64_t> times;
	if(!readFrontEndFile(*data_in, pc_vec, poses, times)) cout<<"Failed read front end file"<<endl;


	int size = pc_vec.size();
	cv::Mat scores = cv::Mat::zeros(size, size, CV_32F);
	fmutil::Stopwatch sw;
	sw.start("matching...");

	vector<ostream*> myfile;
	//myfile.resize(3);
	double res[] = {0.5, 0.025, 1.0};

	stringstream ss0;
	ss0<<argcv[2]<<"_"<<res[0];
	cout<<ss0.str().c_str()<<endl;
	ofstream file0(ss0.str().c_str());
	myfile.push_back(&file0);

	stringstream ss1;
	ss1<<argcv[2]<<"_"<<res[1];
	cout<<ss1.str().c_str()<<endl;
	ofstream file1(ss1.str().c_str());
	myfile.push_back(&file1);

	stringstream ss2;
	ss2<<argcv[2]<<"_"<<res[2];
	cout<<ss2.str().c_str()<<endl;
	ofstream file2(ss2.str().c_str());
	myfile.push_back(&file2);



	for(int i=0; i<pc_vec.size(); i++)
	{
		RenderMap rm;
		cout<<"Compressing points... "<<i<<'\xd'<<flush;
		//cout<<"Size of pc_vec: "<<pc_vec[i].points.size()<<endl;
		vector<geometry_msgs::Point32> input_points;
		input_points = pc_vec[i].points;
		if(i>1)
		{
			geometry_msgs::Pose odo = ominus(poses[i-2], poses[i]);
			vector<geometry_msgs::Point32> input_pt_tran= getTransformedPts(odo, pc_vec[i-2].points);
			input_points.insert(input_points.end(), input_pt_tran.begin(), input_pt_tran.end());
		}
		rm.drawMap(input_points, 0.025);
		vector<cv::Point2f>  query_pts = rm.mapToRealPts();
		/*query_pts.resize(pc_vec[i].points.size());
		for(size_t k=0; k<pc_vec[i].points.size(); i++)
		{
			query_pts[k].x = pc_vec[i].points[k].x;
			query_pts[k].y = pc_vec[i].points[k].y;
		}*/

		vector<vector<cv::Point2f> > query_pts_ds;
		for(int k=0; k<3; k++)
		{
			query_pts_ds.push_back(pcl_downsample(query_pts, res[k], res[k], res[k]));
		}

		for(size_t k=0; k<query_pts_ds.size(); k++)
		{
			(*myfile[k]) << poses[i].position.x << " " << poses[i].position.y <<" " << poses[i].position.z << " " << poses[i].orientation.x <<" " << poses[i].orientation.y<< " "<<poses[i].orientation.z<<" ";
			(*myfile[k]) << times[i]<<" "<<query_pts_ds[k].size();
			for(size_t j=0; j<query_pts_ds[k].size(); j++)
				(*myfile[k])<<" "<<query_pts_ds[k][j].x<<" "<<query_pts_ds[k][j].y;
			(*myfile[k]) <<endl;
		}

	}
	sw.end();
	file0.close();
	file1.close();
	file2.close();


	return 0;
}
