/*
 * readFileHelper.h
 *
 *  Created on: Dec 1, 2012
 *      Author: demian
 */
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

bool readPts3D(istream &in, vector<geometry_msgs::Point32> *p, geometry_msgs::Pose &pose, uint64_t &time)
{
	int pc_size;

	if(!(in >> pose.position.x )) return false;
	if(!(in >> pose.position.y)) return false;
	if(!(in >> pose.position.z)) return false;
	if(!(in >> pose.orientation.x)) return false;
	if(!(in >> pose.orientation.y)) return false;
	if(!(in >> pose.orientation.z)) return false;
	if(!(in >> time)) return false;
	if(!(in >> pc_size)) return false;
	//if(p.points.size()>0) p.points.clear();


	p->resize(pc_size);


	for(int i=0; i<pc_size; i++)
	{
		if(!(in >> (*p)[i].x))
		{
			cout<<"Error reading pt x"<<endl;
			exit(1);
		}
		if(!(in >> (*p)[i].y))
		{
			cout<<"Error reading pt y"<<endl;
			exit(2);
		}
		if(!(in >> (*p)[i].z))
		{
			cout<<"Error reading pt z"<<endl;
			exit(3);
		}
	}

	return true;
}

bool readPts3D(istream &in, sensor_msgs::PointCloud &p, geometry_msgs::Pose &pose, uint64_t &time)
{
	int pc_size;

	if(!(in >> pose.position.x )) return false;
	if(!(in >> pose.position.y)) return false;
	if(!(in >> pose.position.z)) return false;
	if(!(in >> pose.orientation.x)) return false;
	if(!(in >> pose.orientation.y)) return false;
	if(!(in >> pose.orientation.z)) return false;
	if(!(in >> time)) return false;
	if(!(in >> pc_size)) return false;
	//if(p.points.size()>0) p.points.clear();


	p.points.resize(pc_size);


	for(int i=0; i<pc_size; i++)
	{
		if(!(in >> p.points[i].x))
		{
			cout<<"Error reading pt x"<<endl;
			exit(1);
		}
		if(!(in >> p.points[i].y))
		{
			cout<<"Error reading pt y"<<endl;
			exit(2);
		}
		if(!(in >> p.points[i].z))
		{
			cout<<"Error reading pt z"<<endl;
			exit(3);
		}
	}

	return true;
}
bool readPts(istream &in, sensor_msgs::PointCloud &p, geometry_msgs::Pose &pose, uint64_t &time)
{
	int pc_size;

	if(!(in >> pose.position.x )) return false;
	if(!(in >> pose.position.y)) return false;
	if(!(in >> pose.position.z)) return false;
	if(!(in >> pose.orientation.x)) return false;
	if(!(in >> pose.orientation.y)) return false;
	if(!(in >> pose.orientation.z)) return false;
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

bool readPts(istream &in, sensor_msgs::PointCloud &p, geometry_msgs::Pose &pose)            // read point (false on EOF)
{
	uint64_t time;
	return readPts(in, p, pose, time);
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

bool readFrontEndFile(istream& data_in, vector<sensor_msgs::PointCloud>& pc_vec, vector<geometry_msgs::Pose>& poses, vector<uint64_t> &times)
{
	sensor_msgs::PointCloud pc;
  poses.clear();
	geometry_msgs::Pose pose;
	uint64_t time;
	while(readPts(data_in, pc, pose, time))
	{
		pc_vec.push_back(pc);
		poses.push_back(pose);
		times.push_back(time);
	}
	assert(pc_vec.size() == poses.size());
	assert(pc_vec.size() == times.size());
	cout<<"Successfully read "<<pc_vec.size()<<" vectors of data pts"<<endl;
	return true;
}

bool readFrontEndFile(istream& data_in, vector<sensor_msgs::PointCloud>& pc_vec, vector<geometry_msgs::Pose>& poses)
{
	vector<uint64_t> time;
	return readFrontEndFile(data_in, pc_vec, poses, time);
}

bool readFrontEndFile(istream& data_in, vector<sensor_msgs::PointCloud>& pc_vec)
{
	vector<geometry_msgs::Pose> poses;

	return readFrontEndFile(data_in, pc_vec, poses);
}
