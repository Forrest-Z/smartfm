#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pomdp_path_planner/GetPomdpPath.h>
#include <pomdp_path_planner/PomdpPath.h>
#include <vector>
#include <fstream>
using namespace std;
vector<pomdp_path_planner::PomdpPath> paths;
void LoadPath(const char*path_name)
{
	std::ifstream in(path_name);
	int pathLength;
	in>>pathLength;
	std::cout<<"path length "<<pathLength<<std::endl;
	//pathLength=
	pomdp_path_planner::PomdpPath path;		
	path.points.resize(pathLength);
	for(int i=0;i<pathLength;i++)
	{
		in>>path.points[i].x;
		in>>path.points[i].y;
	}	
	paths.push_back(path);
}

bool sendPaths(pomdp_path_planner::GetPomdpPath::Request  &req,  pomdp_path_planner::GetPomdpPath::Response &res)
{
	res.CurrPaths=paths;
	return true;
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"PathPlanner");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("get_pomdp_paths", sendPaths);		
	LoadPath("path1");
	LoadPath("path2");
	ros::spin();
}
