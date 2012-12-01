/*
 * readFileHelper.h
 *
 *  Created on: Dec 1, 2012
 *      Author: demian
 */

bool readPts(istream &in, sensor_msgs::PointCloud &p)            // read point (false on EOF)
{

	uint64_t time;
	int pc_size;
	double x,y,t;
	if(!(in >> x)) return false;
	if(!(in >> y)) return false;
	if(!(in >> t)) return false;
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

bool readFrontEndFile(istream& data_in, vector<sensor_msgs::PointCloud>& pc_vec)
{
	sensor_msgs::PointCloud pc;
	while(readPts(data_in, pc))
			pc_vec.push_back(pc);
	cout<<"Successfully read "<<pc_vec.size()<<" vectors of data pts"<<endl;
	return true;
}
