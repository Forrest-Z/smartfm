/*
 * renderMap.cpp
 *
 *  Created on: Dec 1, 2012
 *      Author: demian
 */


#include "ReadFileHelper.h"
#include "RasterMapPCL.h"
#include "renderMap.h"

int main(int argc, char** argcv)
{
	istream* data_in = NULL;
	ifstream dataStreamSrc;
	dataStreamSrc.open(argcv[1], ios::in);// open data file
	if (!dataStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	data_in = &dataStreamSrc;
	vector<sensor_msgs::PointCloud> pc_vec;
	vector<geometry_msgs::Point32> poses, final_pt;
	if(!readFrontEndFile(*data_in, pc_vec, poses)) cout<<"Failed read front end file"<<endl;

	assert(pc_vec.size() == poses.size());

	for(size_t i=0; i<poses.size(); i++)
	{
		double ct = cos(poses[i].z), st = sin(poses[i].z);
		cout<<"Transform pt to odom frame "<<i<<"/"<<poses.size()<<"      \xd"<<flush;
		for(size_t j=0; j<pc_vec[i].points.size(); j++)
		{

			geometry_msgs::Point32 pt = pc_vec[i].points[j], rot_pt;
			rot_pt.x = ct * pt.x - st * pt.y + poses[i].x;
			rot_pt.y = st * pt.x + ct * pt.y + poses[i].y;
			final_pt.push_back(rot_pt);
		}
	}

	cout<<"Rendering map for "<<final_pt.size()<<" pts"<<endl;
	drawMap(final_pt, 0.1, string(argcv[2]));



	return 0;
}
