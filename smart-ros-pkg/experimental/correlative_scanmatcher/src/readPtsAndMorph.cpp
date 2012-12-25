/*
 * RasterMapImageOptSample.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: demian
 */

//#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "pcl/ros/conversions.h"
#include "RasterMapPCL.h"
#include "ReadFileHelper.h"



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

	ofstream myfile;
	myfile.open (argcv[2]);

	for(int i=0; i<size; i++)
	{
		RenderMap rm;
		cout<<"Compressing points... "<<i<<'\xd'<<flush;
		rm.drawMap(pc_vec[i].points, 0.02);
		vector<cv::Point2f>  query_pts = rm.mapToRealPts();

		myfile << poses[i].position.x << " " << poses[i].position.y <<" " << poses[i].position.z << " " << poses[i].orientation.x <<" " << poses[i].orientation.y<< " "<<poses[i].orientation.z<<" ";
		myfile<< times[i]<<" "<<query_pts.size();
		for(size_t j=0; j<query_pts.size(); j++)
			myfile<<" "<<query_pts[j].x<<" "<<query_pts[j].y;
		myfile <<endl;

	}
	sw.end();
	myfile.close();


	return 0;
}
