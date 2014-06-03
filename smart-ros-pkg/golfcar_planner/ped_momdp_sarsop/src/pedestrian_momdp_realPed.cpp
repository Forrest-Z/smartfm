/*
 * pedestrian_momdp.cpp
 *
 *  Created on: Sep 15, 2011
 *      Author: golfcar
 */

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
    /// Setting up subsciption
    ros::NodeHandle nh;
    speedSub_ = nh.subscribe("odom", 1, &pedestrian_momdp::speedCallback, this);
    pedSub_ = nh.subscribe("ped_local_frame_vector", 1, &pedestrian_momdp::pedPoseCallback, this); 
	//pathSub_=nh.subscribe("global_plan", 1, &pedestrian_momdp::pathCallback,this);
	pc_pub=nh.advertise<sensor_msgs::PointCloud>("confident_objects_momdp",1);
	path_pub=nh.advertise<nav_msgs::Path>("momdp_path",10);
	goal_pub=nh.advertise<visualization_msgs::MarkerArray> ("pomdp_goals",1);
    ros::NodeHandle n("~");

	pathPublished=false;
    string  ped_id_file, policy_file, model_file;
    double frequency;
    int simLen, simNum;
    bool use_sim_time, stationary;
    n.param("pedestrian_id_file", ped_id_file, string(""));
    n.param("policy_file", policy_file, string(""));
    n.param("model_file", model_file, string(""));
    n.param("simLen", simLen, 100);
    n.param("simNum", simNum, 100);
    n.param("frequency", frequency, 2.0);
    n.param("stationary",stationary, false);
    nh.param("use_sim_time", use_sim_time, false);

    move_base_speed_=nh.subscribe("momdp_speed_dummy",1, &pedestrian_momdp::moveSpeedCallback, this);
    //goalPub_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);

	cerr << "DEBUG: Creating ped_momdp instance" << endl;
	momdp = new ped_momdp(model_file, policy_file, simLen, simNum, stationary, frequency, use_sim_time, nh);

	momdp->window_pub=nh.advertise<geometry_msgs::PolygonStamped>("/my_window",1000);
	momdp->pa_pub=nh.advertise<geometry_msgs::PoseArray>("my_poses",1000);
	momdp->car_pub=nh.advertise<geometry_msgs::PoseStamped>("car_pose",1000);
	//momdp->path_client=nh.serviceClient<pomdp_path_planner::GetPomdpPath>("get_pomdp_paths");
	momdp->path_client=nh.serviceClient<nav_msgs::GetPlan>("/ped_path_planner/planner/make_plan");

	

	//momdp->simLoop();

    ros::spin();
}
extern double marker_colors[20][3];
void pedestrian_momdp::publishPath()
{
	cerr << "DEBUG: Call publishPath() " << endl;
	nav_msgs::Path msg;

	char buf[100];
	sprintf(buf,"%s%s",ModelParams::rosns,"/map");
	msg.header.frame_id=buf;
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

	//publish goals also
	visualization_msgs::MarkerArray markers;
	uint32_t shape = visualization_msgs::Marker::CYLINDER;

	for(int i=0;i<ModelParams::NGOAL;i++)
	{
		visualization_msgs::Marker marker;			

		char buf[100];
		sprintf(buf,"%s%s",ModelParams::rosns,"/map");
		marker.header.frame_id=buf;
		marker.header.stamp=ros::Time::now();
		marker.ns="basic_shapes";
		marker.id=i;
		marker.type=shape;
		marker.action = visualization_msgs::Marker::ADD;


		marker.pose.position.x = momdp->worldModel.goals[i].x;
		marker.pose.position.y = momdp->worldModel.goals[i].y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		marker.scale.x = 1;
		marker.scale.y = 1;
		marker.scale.z = 1;
		marker.color.r = marker_colors[i][0];
		marker.color.g = marker_colors[i][1];
		marker.color.b = marker_colors[i][2];
		marker.color.a = 1.0;
		
		markers.markers.push_back(marker);
	}
	goal_pub.publish(markers);
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

	char buf[100];
	sprintf(buf,"%s%s",ModelParams::rosns,"/map");
	pc.header.frame_id=buf;
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
	
    srand(unsigned(time(0)));
    pedestrian_momdp mdp_node;
}
