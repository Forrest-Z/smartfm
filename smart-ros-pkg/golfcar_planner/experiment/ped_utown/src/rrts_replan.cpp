/*
 * rrts_replan.cpp
 *
 *  Created on: Sep 17, 2013
 *      Author: liuwlz
 */

#include "rrts_replan.h"

RRTSReplan::RRTSReplan(){
	srand(0);
	planner_dt = 0.5;
	got_map = false;
	got_goal = false;

	local_map_sub_ = nh_.subscribe("local_map", 2, &RRTSReplan::Get_Map, this);

	rrts_status_pub_ = nh_.advertise<rrts_exp::rrts_status>("rrts_status",1);
	rrts_traj_pub_ = nh_.advertise<nav_msgs::Path>("rrts_traj",1);
	rrts_tree_pub_ = nh_.advertise<sensor_msgs::PointCloud>("rrts_tree",1);
	rrts_vertex_pub_ = nh_.advertise<sensor_msgs::PointCloud>("rrts_vertex",1);

	planner_timer = nh_.createTimer(ros::Duration(planner_dt), &RRTSReplan::Planner_Timer, this);
	tree_pub_timer = nh_.createTimer(ros::Duration(planner_dt), &RRTSReplan::Tree_Pub_Timer, this);
}

RRTSReplan::~RRTSReplan(){

}

int RRTSReplan::Get_Robot_Pose(){
	tf::Stamped<tf::Pose> map_pose;
	map_pose.setIdentity();
	tf::Stamped<tf::Pose> base_pose;
	base_pose.setIdentity();
	base_pose.frame_id_ = "base_link";
	base_pose.stamp_ = ros::Time();
	ros::Time current_time = ros::Time::now();

	bool transform_is_correct = false;
	try {
		tf_.transformPose("map", base_pose, map_pose);
	}
	catch(tf::LookupException& ex) {
		ROS_ERROR("No Transform available Error: %s\n", ex.what());
	   	transform_is_correct = false;
	}
	catch(tf::ConnectivityException& ex) {
		ROS_ERROR("Connectivity Error: %s\n", ex.what());
	    transform_is_correct = false;
	}
	catch(tf::ExtrapolationException& ex) {
	    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
	    transform_is_correct = false;
	}
	if(transform_is_correct){
		geometry_msgs::PoseStamped tmp;
		tf::poseStampedTFToMsg(map_pose, tmp);

	    double roll=0, pitch=0, yaw=0;
	    tf::Quaternion q;
	    tf::quaternionMsgToTF(tmp.pose.orientation, q);
	    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	    robot_pose.x = tmp.pose.position.x;
	    robot_pose.y = tmp.pose.position.y;
	    robot_pose.z = yaw;
	    return 1;
	}
	return 0;
}

void RRTSReplan::Get_Map(const pnc_msgs::local_map::ConstPtr local_map){
	ROS_INFO("Got Map");
	map = local_map->occupancy;
	system.map = local_map->occupancy;
	system.free_cells = local_map->free_cells;
	if (Get_Robot_Pose() != 0){
		system.map_origin[0] = robot_pose.x;
		system.map_origin[1] = robot_pose.y;
		system.map_origin[2] = robot_pose.z;
	}
	got_map = true;
}

void RRTSReplan::Get_Goal(geometry_msgs::PoseStamped sub_goal){
	double roll=0, pitch=0, yaw=0;
	tf::Quaternion q;
	tf::quaternionMsgToTF(sub_goal.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	goal.x = sub_goal.pose.position.x;
	goal.y = sub_goal.pose.position.y;
	goal.z = yaw;
	Set_Goal_Region();
	got_goal = true;
}

int RRTSReplan::Set_Goal_Region(){
	system.regionGoal.center[0] = goal.x;
	system.regionGoal.center[1] = goal.y;
	system.regionGoal.center[2] = goal.z;
	system.regionGoal.size[0] = 1.5;
	system.regionGoal.size[1] = 1.5;
	system.regionGoal.size[2] = 20.0/180.0*M_PI;
	return 1;
}

bool RRTSReplan::Robot_In_Goal(){
	state_t curr_state;
	curr_state.x[0] = robot_pose.x;
	curr_state.x[1] = robot_pose.y;
	curr_state.x[2] = robot_pose.z;
	bool res = system.isReachingTarget(curr_state);
	rrts_status.root_in_goal = res;
	return res;
}

bool RRTSReplan::Robot_In_Collision(){
	Get_Robot_Pose();
	double tmp_pose[3] = {robot_pose.x, robot_pose.y, robot_pose.z};
	bool res = rrtstar.system->IsInCollision(tmp_pose);
	rrts_status.robot_in_collision = res;
	return res;
}

int RRTSReplan::Initalize(){
	// Get robot position
	if(Get_Robot_Pose() == 0)
		return 0;

	//Get the pointer of "system"
	rrtstar.setSystem(system);
	vertex_t &root = rrtstar.getRootVertex();
	state_t &rootState = root.getState();
	rootState[0] = robot_pose.x;
	rootState[1] = robot_pose.y;
	rootState[2] = robot_pose.z;

	system.regionOperating.center[0] = 0;
	system.regionOperating.center[1] = 0;
	system.regionOperating.center[2] = 0;

	system.regionOperating.size[0] = system.map.info.height*system.map.info.resolution;
	system.regionOperating.size[1] = system.map.info.width*system.map.info.resolution;
	system.regionOperating.size[2] = 2.0 * M_PI;

	system.regionCell.center[0] = 0;
	system.regionCell.center[1] = 0;
	system.regionCell.center[2] = 0;
	system.regionCell.size[0] = map.info.resolution;
	system.regionCell.size[1] = map.info.resolution;
	system.regionCell.size[2] = 2.0*M_PI;

	Set_Goal_Region();

	rrtstar.setGamma (2.0);
	rrtstar.setGoalSampleFrequency (0.2);
	rrtstar.initialize();

	rrts_status.trajectory_found = false;
	return 1;
}

int RRTSReplan::Safe_React(){
	for(list<double*>::iterator i = committed_traj.begin(); i!=committed_traj.end(); i++){
		double* stateRef = *i;
		delete[] stateRef;
	}
	committed_traj.clear();
	rrts_status.trajectory_found = false;
	return 1;
}

int RRTSReplan::Replan(){

	ROS_DEBUG("Replan Process");
	//rrtstar.checkTree();

	if(Robot_In_Goal()){
		return 0;
	}

	if(Robot_In_Collision()){
		ROS_DEBUG("robot in collision");
		Safe_React();
	    return 0;
	}

	bool optimal_traj_found = false;
	double optimal_cost;
	double prev_optimal_cost = rrtstar.getBestVertexCost();
	int num_trials_loop = 0;

	flush(cout);

	std::vector<double> sample_view;

	ros::Time start_current_call_back = ros::Time::now();

	while((!optimal_traj_found) || (num_trials_loop < 20)){
		num_trials_loop += rrtstar.iteration(sample_view);
		sample_view.clear();
		optimal_cost = rrtstar.getBestVertexCost();

		if(optimal_cost < 1000){
			if( (fabs(prev_optimal_cost - optimal_cost) < 0.05) && (rrtstar.numVertices > 10))
				optimal_traj_found = true;
		}
		if(num_trials_loop % 2 == 0){
			prev_optimal_cost = optimal_cost;
		}

		//Jump out of the loop to communicate with SMACH
	    ros::Duration dt = ros::Time::now() - start_current_call_back;
	    if(dt.toSec() > 0.8*planner_dt)
	    	break;
	}
	if (optimal_cost < DBL_MAX/2)
		ROS_INFO(" Number of Vertex: %d --- Optimal Cost: %f ", rrtstar.numVertices , optimal_cost);
	if(optimal_traj_found){
		if(rrtstar.getBestTrajectory(committed_traj, committed_control) >0){
			rrts_status.trajectory_found = true;
			Publish_Committed_Traj();
		}
		rrts_status.goal_infeasible = false;
	}
	else{
		Safe_React();
		if(rrtstar.numVertices > 300){
			rrts_status.goal_infeasible = true;
			Initalize();
		}
	}
	return 1;
}

void RRTSReplan::Planner_Timer(const ros::TimerEvent &e){
	if (rrts_status.trajectory_found){
		if (!rrtstar.isSafeTrajectory(committed_traj)){
			ROS_DEBUG("Committed traj not safe");
			Safe_React();
			Initalize();
		}
	}
	else{
		if( (got_goal == true) && (got_map == true) ){
			Replan();
		    return;
		}
	}
}

int RRTSReplan::Publish_Committed_Traj(){
	traj_msg.header.stamp = ros::Time::now();
	traj_msg.header.frame_id = "map";

	list<float>::iterator committed_control_iter = committed_control.begin();
	for (list<double*>::iterator iter = committed_traj.begin(); iter != committed_traj.end(); iter++){
		double* stateRef = *iter;
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "map";

		pose.pose.position.x = stateRef[0];
		pose.pose.position.y = stateRef[1];
		pose.pose.position.z = *committed_control_iter;        // send control as the third state
		pose.pose.orientation.w = 1.0;
		traj_msg.poses.push_back(pose);

		ROS_DEBUG(" [%f, %f, %f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
		committed_control_iter++;
	}
	rrts_traj_pub_.publish(traj_msg);
	traj_msg.poses.clear();
	return 1;
}

void RRTSReplan::Tree_Pub_Timer(const ros::TimerEvent &e){
	rrts_status_pub_.publish(rrts_status);
	int num_nodes = rrtstar.numVertices;

	sensor_msgs::PointCloud vertex;
	vertex.header.stamp = ros::Time::now();
	vertex.header.frame_id = "map";

	sensor_msgs::PointCloud tree;
	tree.header.stamp = ros::Time::now();
	vertex.header.frame_id = "map";

	if (num_nodes > 0){
		for (list<vertex_t*>::iterator iter = rrtstar.listVertices.begin(); iter != rrtstar.listVertices.end(); iter++){
			vertex_t &vertexCurr = **iter;
			state_t &stateCurr = vertexCurr.getState ();

			geometry_msgs::Point32 vpts;
			vpts.x = stateCurr[0];
			vpts.y = stateCurr[1];
			vpts.z = 0.0;
			vertex.points.push_back(vpts);
			tree.points.push_back(vpts);
			vertex_t& vertexParent = vertexCurr.getParent();
			if (&vertexParent != NULL){
				state_t& stateParent = vertexParent.getState();
				list<double*> trajectory;
				list<float> control;
				if (system.getTrajectory (stateParent, stateCurr, trajectory, control, true)){
					int par_num_states = trajectory.size();
					if (par_num_states){
						int stateIndex = 0;
						for (list<double*>::iterator it_state = trajectory.begin(); it_state != trajectory.end(); it_state++){
							double *stateTraj = *it_state;
							geometry_msgs::Point32 tpts;
							tpts.x = stateTraj[0];
							tpts.y = stateTraj[1];
							tpts.z = 0.0;
							tree.points.push_back(tpts);
							stateIndex++;
							delete [] stateTraj;
						}
					}
				}
			}
		}
	}
	rrts_vertex_pub_.publish(vertex);
	rrts_tree_pub_.publish(tree);
}

