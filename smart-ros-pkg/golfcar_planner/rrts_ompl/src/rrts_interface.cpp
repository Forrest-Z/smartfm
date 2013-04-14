/*
 * rrts_interface.cpp
 *
 *  Created on: Jan 24, 2013
 *      Author: liuwlz
 */

#include <rrts_ompl/rrts_interface.h>

rrts_interface::rrts_interface(){
	/*
	map_sub_ = nh_.subscribe("local_map", 2, &rrts_interface::map_CB, this);
	goal_sub_ = nh_.subscribe("pnc_nextpose", 2, &rrts_interface::goal_CB, this);
	resolution = 0;
	*/
	rrtstar = new rrts();
}

rrts_interface::~rrts_interface(){
}

void rrts_interface::map_CB(const nav_msgs::OccupancyGrid map){
	local_map = map;
}

void rrts_interface::goal_CB(const geometry_msgs::PoseStamped goal){

	tf::StampedTransform transform;
	double _angle[3];
	double _goal[3];
	try{
		goal_tf.transformPose("/base_link", goal, sub_goal);
	}
	catch (tf::TransformException ex) {
	    ROS_ERROR("%s",ex.what());
	}

    tf::Quaternion q;
    tf::quaternionMsgToTF(sub_goal.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(_angle[0], _angle[1], _angle[2]);

	_goal[0] = sub_goal.pose.position.x;
	_goal[1] = sub_goal.pose.position.y;
	_goal[2] = _angle[2];

	//rrtstar->setGoal(_goal);
}

int rrts_interface::local_from_global(){
	return 1;
}

int rrts_interface::global_from_local(){
	return 1;
}

int rrts_interface::got_plan(){
	rrtstar->initplanner(50,50);
	double root[3] = {1, 1, 0};
	double goal[3] = {20, 20, 0};
	rrtstar->setRoot(root);
	rrtstar->setGoal(goal);
	rrtstar->planning(path_reals);
	return 1;
}

int rrts_interface::visualize(){

	rrtstar->visualize(rrts_tree_visual);

	cout << "debug rrts_tree_vertex "<< rrts_tree_visual[0].vertex_value[4]<<endl;
	cout << "debug rrts_tree_edge "<< rrts_tree_visual[0].edge_list.size()<<endl;

	return 1;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "rrts");
	rrts_interface rrts_interface;
	rrts_interface.got_plan();
	rrts_interface.visualize();
	ros::spin();
	return 1;
}


