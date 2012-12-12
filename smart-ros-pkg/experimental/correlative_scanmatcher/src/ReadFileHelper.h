/*
 * readFileHelper.h
 *
 *  Created on: Dec 1, 2012
 *      Author: demian
 */
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <fstream>

using namespace std;

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

bool readPts(istream &in, sensor_msgs::PointCloud &p, geometry_msgs::Point32 &pose)            // read point (false on EOF)
{

	uint64_t time;
	int pc_size;

	if(!(in >> pose.x )) return false;
	if(!(in >> pose.y)) return false;
	if(!(in >> pose.z)) return false;
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


bool readFrontEndFile(istream& data_in, vector<sensor_msgs::PointCloud>& pc_vec, vector<geometry_msgs::Point32>& poses)
{
	sensor_msgs::PointCloud pc;
	geometry_msgs::Point32 pose;
	while(readPts(data_in, pc, pose))
	{
		pc_vec.push_back(pc);
		poses.push_back(pose);
	}
	cout<<"Successfully read "<<pc_vec.size()<<" vectors of data pts"<<endl;
	return true;
}

bool readFrontEndFile(istream& data_in, vector<sensor_msgs::PointCloud>& pc_vec)
{
	vector<geometry_msgs::Point32> poses;

	return readFrontEndFile(data_in, pc_vec, poses);
}
