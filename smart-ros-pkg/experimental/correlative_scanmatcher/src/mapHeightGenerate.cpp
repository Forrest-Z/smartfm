#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <iostream>
#include <fstream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <fmutil/fm_math.h>
#include <fmutil/fm_stopwatch.h>
#include <cv.h>
#include <highgui.h>
using namespace std;
#include "renderMap.h"
#include "ReadFileHelper.h"
#include "geometry_helper.h"
#include "pcl_downsample.h"

int main(int argc, char** argcv)
{
	istream*        data_in         = NULL;         // input for data points
	ifstream dataStreamSrc;
	cout<<"Opening "<<argcv[1]<<endl;
	dataStreamSrc.open(argcv[1], ios::in);// open data file
	if (!dataStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	data_in = &dataStreamSrc;
	uint64_t time;
	sensor_msgs::PointCloud pc;
	geometry_msgs::Pose pose;
	int skip_read = 1;
	size_t accumulate_read =1;
	int read_line =0;

	vector<geometry_msgs::Pose> poses;
	vector<sensor_msgs::PointCloud> pc_vecs;
	while(readPts3D(*data_in, pc, pose, time))
	{
		if(read_line%skip_read == 0)
		{
			poses.push_back(pose);
			pc_vecs.push_back(pc);
			//cout<<pc.points.size()<<endl;

			if(poses.size()>accumulate_read)
			{
				poses.erase(poses.begin());
				pc_vecs.erase(pc_vecs.begin());
			}
			if(pc_vecs.size() == accumulate_read)
			{
				//cout<<pose<<endl;
				//cout<<pc_vecs[0]<<endl;
				RenderMap rm;
				vector<geometry_msgs::Point32> acc_pc;
				for(size_t i=0; i<accumulate_read; i++)
				{

					geometry_msgs::Pose rel_pose = ominus( poses[i], poses[accumulate_read-1]);
					vector<geometry_msgs::Point32> pc_transformed = getTransformedPts(rel_pose, pc_vecs[i].points);
					acc_pc.insert(acc_pc.begin(), pc_transformed.begin(), pc_transformed.end());
				}
				acc_pc = pcl_downsample(acc_pc, 0.03, 0.03, 0.01);
				rm.drawHeightMap(acc_pc, 0.2, 140., 140.);
				stringstream ss;
				ss<<"nus_"<<setfill('0') << setw(5)<<read_line<<".png";
				cv::imwrite(ss.str(), rm.image_);
				cv::imshow("HeightMap", rm.image_);
				cv::waitKey();
				cout<<ss.str()<<"\xd"<<flush;/*
				if(read_line == 1588)
				{
					cout<<acc_pc.size()<<" ";
					for(size_t i=0; i<acc_pc.size(); i++)
					{
						cout<<acc_pc[i].x<<" "<<acc_pc[i].y<<" "<<acc_pc[i].z<<" ";
					}
					cout<<endl;
				}*/
			}

		}
		read_line++;
	}
}
