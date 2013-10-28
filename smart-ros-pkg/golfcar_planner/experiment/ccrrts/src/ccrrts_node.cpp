/*
 * ccrrts_node.cpp
 *
 *  Created on: Aug 3, 2013
 *      Author: liuwlz
 */

#include "ccrrts_node.h"

Planner::Planner(){

	srand(time(0));

	ros::Rate wait_rate(0.2);
	while(get_robot_pose()){
		cout<<"Waiting for robot pose"<<endl;
		ros::spinOnce();
		wait_rate.sleep();
	}

	state_last_clear[0] = car_position.x;
	state_last_clear[1] = car_position.y;
	state_last_clear[2] = car_position.z;

	//clear_committed_trajectory();
	is_updating_committed_trajectory = false;
	is_updating_rrt_tree = false;
	is_system_setup = false;
	max_length_committed_trajectory = 200.0;

	planner_dt = 0.5;
	gamma = 1.8;
	risk_limit = 0.3;
	GoalSampleFreq = 0.2;

	planner_timer = nh.createTimer(ros::Duration(planner_dt), &Planner::on_planner_timer, this);

	committed_trajectory_view_pub = nh.advertise<sensor_msgs::PointCloud>("pncview_trajectory", 2);
	committed_trajectory_pub = nh.advertise<nav_msgs::Path>("pnc_trajectory", 2);

	tree_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_tree", 2);
	vertex_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_vertex", 2);
	obs_check_pub = nh.advertise<sensor_msgs::PointCloud>("obs_check", 2);
	planning_result_pub = nh.advertise<ccrrts::rrts_result>("planning_result",2);

	map_sub = nh.subscribe("local_map", 2, &Planner::on_map, this);
	goal_sub = nh.subscribe("pnc_nextpose", 2, &Planner::on_goal, this);
	sampling_view_pub = nh.advertise<sensor_msgs::PointCloud>("samples",2);

	is_first_goal = true;
	is_first_map = true;
	for(int i=0; i<NUM_STATUS; i++)
		rrts_status[i] = false;

	perform_criteria.risk = 0.5;
	perform_criteria.metric_cost = 100;

	optimal_criteria.risk = 0.1;
	optimal_criteria.metric_cost = 0.2;

	result_.risk = 0;
	result_.cost = 0;
	result_.num_vertex = 0;

	cylinder = visualization_msgs::Marker::CYLINDER;
	vertex_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("vertex_conv_marker", 1);

	srv = new dynamic_reconfigure::Server<ccrrts::ccrrtsCTRConfig> (ros::NodeHandle("~"));
	dynamic_reconfigure::Server<ccrrts::ccrrtsCTRConfig>::CallbackType f = boost::bind(&Planner::ParamReconfig, this, _1, _2);
	srv->setCallback(f);
}

Planner::~Planner(){
	clear_committed_trajectory();
}

void Planner::ParamReconfig(ccrrts::ccrrtsCTRConfig &config, uint32_t level){
	gamma = config.Gamma;
	GoalSampleFreq = config.GoalSamplingFreq;
	planner_dt = config.Planner_DT;
	risk_limit = config.RiskLimit;
	ccrrts.setGamma(gamma);
	ccrrts.setGoalSampleFrequency(GoalSampleFreq);
}

int Planner::clear_committed_trajectory(){

	is_updating_committed_trajectory = true;
	for(list<double*>::iterator i=committed_trajectory.begin(); i!=committed_trajectory.end(); i++){
		double* stateRef = *i;
		delete[] stateRef;
	}
	committed_trajectory.clear();
	committed_control.clear();

	publish_committed_trajectory();

	is_updating_committed_trajectory = false;

	if(get_robot_pose() == 1)
		cout<<"robot_pose failed"<<endl;

	state_last_clear[0] = car_position.x;
	state_last_clear[1] = car_position.y;
	state_last_clear[2] = car_position.z;

	return 0;
}

// keep doing pop_front on the committed_trajectory until length
int Planner::clear_committed_trajectory_length(){

	if(get_robot_pose() == 1)
		cout<<"robot_pose failed"<<endl;

	state_last_clear[0] = car_position.x;
	state_last_clear[1] = car_position.y;
	state_last_clear[2] = car_position.z;

	if(committed_trajectory.empty())
		return 0;

	bool reached_end = false;
	list<double*>::iterator iter = committed_trajectory.begin();
	int num_delete = 0;
	while (!reached_end){
		double *s1 = *iter;
		if(dist(s1[0], s1[1], 0, car_position.x, car_position.y, 0) < 1.0){
			reached_end = true;
			break;
		}
		iter++;
		num_delete++;
	}
	is_updating_committed_trajectory = true;
	if(reached_end){
		int n = 0;
		while(n < num_delete){
			committed_trajectory.pop_front();
			committed_control.pop_front();
			n++;
		}
	}
	is_updating_committed_trajectory = false;
	return 0;
}

void Planner::on_goal(const geometry_msgs::PoseStamped::ConstPtr ps){
	double roll=0, pitch=0, yaw=0;
	tf::Quaternion q;
	tf::quaternionMsgToTF(ps->pose.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	if (is_first_goal){
		is_first_goal = false;
		ROS_INFO("got first goal");
		goal.x = ps->pose.position.x;
		goal.y = ps->pose.position.y;
		goal.z = yaw;
		double goal_state[7] = {goal.x, goal.y, goal.z, 0, 0, 0, 0};
		if(is_first_map == false){
			setup_rrts();
			if(ccrrts.system->IsInCollisionLazy(goal_state)){
				cout<<"goal in collision: stopping"<<endl;
				rrts_status[ginc] = true;
			}
			rrts_status[ginc] = false;
		}
	}
	if((!is_first_map) && (!is_first_goal)){
		if( dist(goal.x, goal.y, 0., ps->pose.position.x, ps->pose.position.y, 0.) > 0.5){
			goal.x = ps->pose.position.x;
			goal.y = ps->pose.position.y;
			goal.z = yaw;
			double goal_state[7] = {goal.x, goal.y, goal.z, 0, 0, 0, 0};
			if(ccrrts.system->IsInCollisionLazy(goal_state)){
				cout<<"goal in collision: sending collision"<<endl;
				rrts_status[ginc] = true;
			}
			change_goal_region();
			rrts_status[ginc] = false;
		}
	}
}

int Planner::get_robot_pose(){
	tf::Stamped<tf::Pose> map_pose;
	map_pose.setIdentity();
	tf::Stamped<tf::Pose> robot_pose;
	robot_pose.setIdentity();
	robot_pose.frame_id_ = "base_link";
	robot_pose.stamp_ = ros::Time();
	ros::Time current_time = ros::Time::now();

	bool transform_is_correct = false;
	try {
		tf_.transformPose("map", robot_pose, map_pose);
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
	if (current_time.toSec() - map_pose.stamp_.toSec() > 0.1) {
		ROS_WARN("Get robot pose transform timeout. Current time: %.4f, odom_pose stamp: %.4f, tolerance: %.4f",
				current_time.toSec(), map_pose.stamp_.toSec(), 0.1);
		transform_is_correct = false;
	}
	transform_is_correct = true;

	if(transform_is_correct){
		geometry_msgs::PoseStamped tmp;
		tf::poseStampedTFToMsg(map_pose, tmp);

		double roll=0, pitch=0, yaw=0;
		tf::Quaternion q;
		tf::quaternionMsgToTF(tmp.pose.orientation, q);
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
		car_position.x = tmp.pose.position.x;
		car_position.y = tmp.pose.position.y;
		car_position.z = yaw;
		return 0;
	}
	return 1;
}

void Planner::on_map(const pnc_msgs::local_map::ConstPtr lm){

	system.map = lm->occupancy;
	system.free_cells = lm->free_cells;

	bool got_pose = false;
	// 2. get car_position
	if(get_robot_pose() == 0)
		got_pose = true;

	if(is_first_map){
		is_first_map = false;
		cout<<"got first map"<<endl;
		if(is_first_goal == false){
			setup_rrts();
    	}
	}
	if(got_pose){
		system.map_origin[0] = car_position.x;
		system.map_origin[1] = car_position.y;
		system.map_origin[2] = car_position.z;
	}
}

bool Planner::root_in_goal(){
	vertex_t &rootVertex = ccrrts.getRootVertex();
	state_t &curr_state = rootVertex.getState();
	bool res = system.isReachingTarget(curr_state);
	rrts_status[ring] = res;
	return res;
}

void Planner::setup_rrts(){
	// get car_position
	if(get_robot_pose() == 1)
		ROS_INFO("robot_pose failed");

	ccrrts.setSystem(system);

	vertex_t &root = ccrrts.getRootVertex();
	state_t &rootState = root.getState();

	rootState[0] = car_position.x;
	rootState[1] = car_position.y;
	rootState[2] = car_position.z;
	rootState[3]= 0.16;
	rootState[4]= 0.16;
	rootState[5]= M_PI/180* M_PI/180;
	rootState[6]= 0.16*0.3;

	system.regionOperating.center[0] = 0;
	system.regionOperating.center[1] = 0;
	system.regionOperating.center[2] = 0;

	// just create a large operating region around the car irrespective of the orientation in /map frame, yaw is 2*M_PI
	system.regionOperating.size[0] = system.map.info.height*system.map.info.resolution;
	system.regionOperating.size[1] = system.map.info.width*system.map.info.resolution;
	system.regionOperating.size[2] = 2.0 * M_PI;

	system.regionCell.center[0] = 0;
	system.regionCell.center[1] = 0;
	system.regionCell.center[2] = 0;
	system.regionCell.size[0] = map.info.resolution;
	system.regionCell.size[1] = map.info.resolution;
	system.regionCell.size[2] = 1.0*M_PI;

	change_goal_region();

	//ccrrts.setRiskLimit(risk_limit);
	ccrrts.setGamma (gamma);
	ccrrts.setGoalSampleFrequency (GoalSampleFreq);
	ccrrts.initialize ();

	should_send_new_committed_trajectory = false;
	is_first_committed_trajectory = true;
	is_system_setup = true;
}

void Planner::change_goal_region(){
	ROS_INFO("change_goal_region");
	system.regionGoal.center[0] = (double)goal.x;
	system.regionGoal.center[1] = (double)goal.y;
	system.regionGoal.center[2] = (double)goal.z;
	system.regionGoal.size[0] = 1.5;
	system.regionGoal.size[1] = 1.5;
	system.regionGoal.size[2] = 20.0/180.0*M_PI;
	//cout<<"region_goal: "<< system.regionGoal.center[0]<<" "<<system.regionGoal.center[1]<<" "<<system.regionGoal.center[2]<<endl;
}

bool Planner::is_robot_in_collision(){
	if((!is_first_map) && (!is_first_goal)){
		get_robot_pose();
		double tmp[7] = {car_position.x, car_position.y, car_position.z, 0, 0, 0, 0};
		bool res = ccrrts.system->IsInCollisionLazy(tmp);
		rrts_status[rinc] = res;
		return res;
	}
	else{
		rrts_status[rinc] = false;
		return false;
	}
}

int Planner::get_plan(){

	ROS_INFO("Get Plan");

	is_updating_committed_trajectory = true;

	bool found_best_path = false;
	Level_OF_Risk best_lor=ccrrts.getBestVertexLor();
	Level_OF_Risk prev_best_lor=best_lor;

	int samples_this_loop = 0;
	ros::Time start_current_call_back = ros::Time::now();
	flush(cout);

	std::vector<double> sample_view;
	sensor_msgs::PointCloud sample_view_;
	sample_view_.header.frame_id = "/map";

	while((!found_best_path)){

		ROS_INFO("Number of Veretex: %d", ccrrts.numVertices);
		ros::Duration dt = ros::Time::now() - start_current_call_back;

		samples_this_loop += ccrrts.iteration(sample_view);

		if (sample_view.size()!=0){
			geometry_msgs::Point32 p;
			p.x = sample_view[0]; p.y = sample_view[1]; p.z = sample_view[2];
			sample_view_.points.push_back(p);
			sample_view.clear();
		}

		best_lor = ccrrts.getBestVertexLor();
		ROS_INFO("Best Lor: Risk: %f, Cost: %f", best_lor.risk, best_lor.metric_cost);

		if(best_lor < perform_criteria){
			result_.cost = best_lor.metric_cost;
			result_.risk = best_lor.risk;
			committed_control.clear();
			committed_trajectory.clear();
			ccrrts.getBestTrajectory(committed_trajectory, committed_control);
			publish_committed_trajectory();
			ROS_INFO(" [Committed Traj] Vertex: %d --- Best Cost: %f ---Best Risk: %f ", ccrrts.numVertices ,best_lor.metric_cost, best_lor.risk);
				//found_best_path = true;
			prev_best_lor = best_lor;
		}

		sample_view_.header.stamp = ros::Time::now();
		sampling_view_pub.publish(sample_view_);
		sample_view_.points.clear();

		result_.head.stamp = ros::Time::now();
		result_.num_vertex = ccrrts.numVertices;
		planning_result_pub.publish(result_);

		if(ccrrts.numVertices % 1 == 0){
			publish_tree();
		}

		start_current_call_back = ros::Time::now();
		//}
		/*
		 * Number of vertexs: 500 for dubins path and 2000 for 2D is the best, experimentallu
		 */
		if (ccrrts.numVertices > 500)
			exit(0);
		}

	return 0;
}

bool Planner::is_near_end_committed_trajectory(){
	if(!committed_trajectory.empty()){
		if(get_robot_pose() == 1)
			cout<<"robot_pose failed"<<endl;

		list<double*>::reverse_iterator riter = committed_trajectory.rbegin();
		double* last_committed_state = *riter;
		double delyaw = car_position.x - last_committed_state[2];
		while(delyaw > M_PI)
			delyaw -= 2.0*M_PI;
		while(delyaw < -M_PI)
			delyaw += 2.0*M_PI;

		bool res = false;
		if(dist(car_position.x, car_position.y, 0, last_committed_state[0], last_committed_state[1], 0) < 6.0)
			res = true;
		else
			res = false;
		rrts_status[rnr] = res;
		return res;
	}
	return false;
}

void Planner::on_planner_timer(const ros::TimerEvent &e){

	if( (is_first_goal == false) && (is_first_map == false) ){
		if(!is_updating_committed_trajectory)
			get_plan();
		return;
	}
}

void Planner::publish_committed_trajectory(){

	if (committed_trajectory.size() == 0)
		rrts_status[trjf] = false;
	else
		rrts_status[trjf] = true;

	if (! is_updating_committed_trajectory)
		return;

	nav_msgs::Path traj_msg;
	traj_msg.header.stamp = ros::Time::now();
	traj_msg.header.frame_id = "map";

	list<float>::iterator committed_control_iter = committed_control.begin();
	for (list<double*>::iterator iter = committed_trajectory.begin(); iter != committed_trajectory.end(); iter++){
		double* stateRef = *iter;
		geometry_msgs::PoseStamped p;
		p.header.stamp = ros::Time::now();
		p.header.frame_id = "map";

		p.pose.position.x = stateRef[0];
		p.pose.position.y = stateRef[1];
		p.pose.position.z = *committed_control_iter;        // send control as the third state
		p.pose.orientation.w = 1.0;
		traj_msg.poses.push_back(p);

		  //printf(" [%f, %f, %f]", p.pose.position.x, p.pose.position.y, p.pose.position.z);
		ROS_DEBUG(" [%f, %f, %f]", p.pose.position.x, p.pose.position.y, p.pose.position.z);

		committed_control_iter++;
	}
	committed_trajectory_pub.publish(traj_msg);


	sensor_msgs::PointCloud traj_view;
	traj_view.header.frame_id = "map";
	traj_view.header.stamp = ros::Time::now();

	for (list<double*>::iterator iter = committed_trajectory.begin(); iter != committed_trajectory.end(); iter++){
		double* stateRef = *iter;
		geometry_msgs::Point32 p;

		p.x = stateRef[0];
		p.y = stateRef[1];
		p.z = 0;
		traj_view.points.push_back(p);
	}
	committed_trajectory_view_pub.publish(traj_view);
}

void Planner::publish_tree(){

	if(is_updating_rrt_tree)
		return;

	int num_nodes = ccrrts.numVertices;

	tree.header.stamp = ros::Time::now();
	tree.header.frame_id = "map";

	vertex.header.stamp = ros::Time::now();
	vertex.header.frame_id = "map";

	int id = 0;

	if (num_nodes > 0) {
		for (list<vertex_t*>::iterator iter = ccrrts.listVertices.begin(); iter != ccrrts.listVertices.end(); iter++){
			vertex_t &vertexCurr = **iter;
			state_t &stateCurr = vertexCurr.getState ();

			geometry_msgs::Point32 p;
			p.x = stateCurr[0];
			p.y = stateCurr[1];
			p.z = 0.0;
			tree.points.push_back(p);
			vertex.points.push_back(p);

			publish_vertex_conv(stateCurr, vertex_markers, id);
			id ++;

			vertex_t& vertexParent = vertexCurr.getParent();
			if (&vertexParent != NULL){
				state_t& stateParent = vertexParent.getState();
				list<double*> trajectory;
				list<float> control;
				if (system.getTrajectory (stateParent, stateCurr, trajectory, control, vertexCurr.getTurningRadius()) >0){
					int par_num_states = trajectory.size();
					if (par_num_states){
						int stateIndex = 0;
						for (list<double*>::iterator it_state = trajectory.begin(); it_state != trajectory.end(); it_state++){
							double *stateTraj = *it_state;

							geometry_msgs::Point32 p2;
							p2.x = stateTraj[0];
							p2.y = stateTraj[1];
							p2.z = 0.0;
							tree.points.push_back(p2);
							stateIndex++;
							delete [] stateTraj;
						}
					}
				}
			}
		}
	}
	tree_pub.publish(tree);
	vertex_pub.publish(vertex);
	vertex_marker_pub.publish(vertex_markers);
	tree.points.clear();
	vertex.points.clear();
	vertex_markers.markers.clear();
}

void Planner::publish_vertex_conv(state_t stateIn, visualization_msgs::MarkerArray&vertex_markers,int id){

	Eigen::Matrix2d covMatrix;

	covMatrix << stateIn[3], stateIn[6], stateIn[6], stateIn[4];

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(covMatrix);

	const Eigen::Vector2d& eignValues (eig.eigenvalues());
	const Eigen::Matrix2d& eignVectors (eig.eigenvectors());

	double angle = atan2((double)eignVectors(1, 0), (double)eignVectors(0, 0));
	double lengthX = sqrt((double)eignValues[0]);
	double lengthY = sqrt((double)eignValues[1]);

	//cout <<"x: "<< (eignVectors(1, 0)) << "y: " << (eignVectors(0, 0)) << "angle: " << angle <<endl;
	visualization_msgs::Marker marker;
	marker.id = id;
	marker.ns = "Vertex";
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = cylinder;

	marker.scale.x = lengthX;
	marker.scale.y = lengthY;
	marker.scale.z = 0.001;

	marker.pose.position.x =stateIn[0];
	marker.pose.position.y =stateIn[1] ;
	marker.pose.position.z = 0;

	marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,angle);

	marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f; marker.color.a = 0.4;
	marker.lifetime = ros::Duration(0.45);
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	vertex_markers.markers.push_back(marker);
}

int main(int argc, char**argv){
	ros::init(argc, argv, "ccrrts_node");
	Planner ccrrts_planner;
	ros::spin();
}
