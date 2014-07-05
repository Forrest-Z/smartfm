/*
 * pedestrian_momdp.cpp
 *
 *  Created on: Sep 15, 2011
 *      Author: golfcar
 */

#include <fenv.h>
#include "pedestrian_momdp_realPed.h"
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
//#include <pomdp_path_planner/GetPomdpPath.h>
//#include <pomdp_path_planner/PomdpPath.h>
#include <ped_navfn/MakeNavPlan.h>

pedestrian_momdp::pedestrian_momdp()
{
    ROS_INFO("Starting Pedestrian Avoidance ... ");

    

	cerr << "DEBUG: Setting up subscription..." << endl;
    ros::NodeHandle nh;
    /// Setting up subsciption
    speedSub_ = nh.subscribe("odom", 1, &pedestrian_momdp::speedCallback, this);
    pedSub_ = nh.subscribe("ped_local_frame_vector", 1, &pedestrian_momdp::pedPoseCallback, this); 
	//pathSub_=nh.subscribe("global_plan", 1, &pedestrian_momdp::pathCallback,this);
	pc_pub=nh.advertise<sensor_msgs::PointCloud>("confident_objects_momdp",1);
	path_pub=nh.advertise<nav_msgs::Path>("momdp_path",10);
    ros::NodeHandle n("~");

	pathPublished=false;
    bool simulation;
    n.param("simulation", simulation, false);
    ModelParams::init_params(simulation);
	cout << "simulation = " << simulation << endl;
	cout << "rosns = " << ModelParams::rosns << endl;

	bool fixed_path;
	double pruning_constant;
    n.param("pruning_constant", pruning_constant, 0.0);
	n.param("fixed_path", fixed_path, false);

    move_base_speed_=nh.subscribe("momdp_speed_dummy",1, &pedestrian_momdp::moveSpeedCallback, this);
    //goalPub_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);

	cerr << "DEBUG: Creating ped_momdp instance" << endl;
	momdp = new ped_momdp(nh, fixed_path, pruning_constant);

	momdp->window_pub=nh.advertise<geometry_msgs::PolygonStamped>("/my_window",1000);
	momdp->pa_pub=nh.advertise<geometry_msgs::PoseArray>("my_poses",1000);
	momdp->car_pub=nh.advertise<geometry_msgs::PoseStamped>("car_pose",1000);
	//momdp->path_client=nh.serviceClient<pomdp_path_planner::GetPomdpPath>("get_pomdp_paths");

	momdp->path_client=nh.serviceClient<nav_msgs::GetPlan>(ModelParams::rosns + "/ped_path_planner/planner/make_plan");


	//momdp->simLoop();

    ros::spin();
}
extern double marker_colors[20][3];
void pedestrian_momdp::publishPath()
{
	cerr << "DEBUG: Call publishPath() " << endl;
	nav_msgs::Path msg;

	msg.header.frame_id=ModelParams::rosns+"/map";
	msg.header.stamp=ros::Time::now();
	vector<COORD> path=momdp->worldModel.path;
	msg.poses.resize(path.size());
	for(int i=0;i<path.size();i++)
	{
		msg.poses[i].pose.position.x=momdp->worldModel.path[i].x;
		msg.poses[i].pose.position.y=momdp->worldModel.path[i].y;
		msg.poses[i].pose.position.z=0;
		msg.poses[i].header.frame_id=msg.header.frame_id;
		msg.poses[i].header.stamp=ros::Time::now();
	}
	path_pub.publish(msg);
	cout<<"path with length "<<length<<" published"<<endl;

	cerr << "DEBUG: Done publishPath() " << endl;
}
pedestrian_momdp::~pedestrian_momdp()
{
    //momdp->~ped_momdp();
}

void pedestrian_momdp::speedCallback(nav_msgs::Odometry odo)
{
	//cout<<"speed callback!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
    //momdp->updateRobotSpeed(odo.twist.twist.linear.x);
    momdp->worldStateTracker.updateVel(odo.twist.twist.linear.x);
	momdp->real_speed_=odo.twist.twist.linear.x;
}

void pedestrian_momdp::moveSpeedCallback(geometry_msgs::Twist speed)
{
    momdp->updateSteerAnglePublishSpeed(speed);
}

bool sortFn(Pedestrian p1,Pedestrian p2)
{
	return p1.id<p2.id;
}

void pedestrian_momdp::pedPoseCallback(ped_momdp_sarsop::ped_local_frame_vector lPedLocal)
{

	if(lPedLocal.ped_local.size()==0) return;

	sensor_msgs::PointCloud pc;

	pc.header.frame_id=ModelParams::rosns+"/map";
	pc.header.stamp=lPedLocal.ped_local[0].header.stamp;
	//RealWorld.ped_list.clear();
	vector<Pedestrian> ped_list;
    for(int ii=0; ii< lPedLocal.ped_local.size(); ii++)
    {
		geometry_msgs::Point32 p;
		Pedestrian world_ped;
		ped_momdp_sarsop::ped_local_frame ped=lPedLocal.ped_local[ii];
		world_ped.id=ped.ped_id;
		world_ped.w = ped.ped_pose.x;
		world_ped.h = ped.ped_pose.y;
		//TODO : goal
		p.x=ped.ped_pose.x;
		p.y=ped.ped_pose.y;
		p.z=1.0;
		pc.points.push_back(p);

		//cout<<"ped pose "<<ped.ped_pose.x<<" "<<ped.ped_pose.y<<" "<<world_ped.id<<endl;
			
        /// search for proper pedestrian to update
        //bool foundPed = momdp->updatePedRobPose(lPedLocal.ped_local[ii]);;
		ped_list.push_back(world_ped);
        //if(!foundPed)
        //{
            ///if ped_id does not match the old one create a new pomdp problem.
            //ROS_INFO(" Creating  a new pedestrian problem #%d", lPedLocal.ped_local[ii].ped_id);
            //momdp->addNewPed(lPedLocal.ped_local[ii]);
			//ROS_INFO("Create a new pedestrian!", lPedLocal.ped_local[ii].ped_id);
			//world.addNewPed(lPedLocal.ped_local[ii]);                                                  }
    }
	std::sort(ped_list.begin(),ped_list.end(),sortFn);
	for(int i=0;i<ped_list.size();i++)
	{
		momdp->worldStateTracker.updatePed(ped_list[i]);
		//cout<<ped_list[i].id<<" ";
	}
	//cout<<endl;
	pc_pub.publish(pc);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mdp");

    // raise error for floating point exceptions rather than silently return nan
    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

    srand(unsigned(time(0)));
    pedestrian_momdp mdp_node;
}
