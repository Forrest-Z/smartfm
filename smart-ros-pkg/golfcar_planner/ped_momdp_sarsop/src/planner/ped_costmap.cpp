#include<ros/ros.h>
#include "ped_momdp_sarsop/peds_believes.h"
#include "ped_momdp_sarsop/ped_belief.h"
#include "WorldModel.h"
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>
int simLen=50;
int simNum=100;
const int pedN=10;
const int SIZE_X=1000,SIZE_Y=1000;
int cost[pedN][SIZE_X][SIZE_Y];
//Costmap2D costmap;
WorldModel world;
double RESOLUTION=0.1;


nav_msgs::GridCells gco;
ros::Publisher cell_pub;
void Trial(PedStruct ped)
{
	Random random((rand()+0.0)/RAND_MAX);
	cout<<"trial"<<endl;
	for(int i=0;i<simLen;i++)  {
		int mx,my;
		mx=ped.pos.x/RESOLUTION;
		my=ped.pos.y/RESOLUTION;
		cout<<mx<<" "<<my<<endl;
		//costmap.worldToMap(ped.pos.x,ped.pos.y,mx,my);
		cost[ped.id][mx][my]++;	
		geometry_msgs::Point point;
		point.x=ped.pos.x;
		point.y=ped.pos.y;
		point.z=0;
		gco.cells.push_back(point);
		world.PedStep(ped, random);	
	}
}
void GenCostMap(PedStruct ped, int n)
{
	for(int j=0;j<n;j++)  {
		Trial(ped);
	}
}

void believeCallBack(ped_momdp_sarsop::peds_believesConstPtr pbs)
{
	cout<<"in the callback"<<endl;
	int n_ped=pbs->believes.size();
	vector<PedBelief> believes;
	for(int i=0;i<n_ped;i++) {
		PedBelief pb;
		pb.pos.x=pbs->believes[i].ped_x;
		pb.pos.y=pbs->believes[i].ped_y;
		pb.prob_goals=pbs->believes[i].belief_value;
	}
	cout<<"start simiulation"<<endl;
	for(int i=0;i<n_ped;i++) {
	//simulation for each pedestrian	
		PedStruct ped;
		ped.pos.x=believes[i].pos.x;
		ped.pos.y=believes[i].pos.y;
		ped.goal=believes[i].sample_goal();
		ped.id=i;
		GenCostMap(ped,simNum);
	}
	cell_pub.publish(gco);
}
int main(int argc, char**argv)
{
	ros::init(argc,argv,"ped_costmap");
	ros::NodeHandle nh;
	ros::Subscriber peds_believes_sub = nh.subscribe("peds_believes", 1, believeCallBack);
	cell_pub=nh.advertise<nav_msgs::GridCells>("ped_costmap",1);
    gco.header.frame_id = "/map";
	gco.header.stamp = ros::Time();
	gco.cell_width = RESOLUTION;
	gco.cell_height = RESOLUTION;
	//cmd_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
	//ros::Timer timer = nh.createTimer(ros::Duration(0.1), publishSpeed);
	ros::spin();
}
