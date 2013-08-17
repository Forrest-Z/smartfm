/*
 * norm_system.cpp
 *
 *  Created on: Aug 17, 2013
 *      Author: liuwlz
 */

//TODO: Modify it to 2D environment

#include "norm_system.h"

bool first_debug = true;

NormState::NormState () {

}

NormState::~NormState() {

}

NormState::NormState (const NormState &stateIn) {

  for (int i = 0; i < 5; i++)
    x[i] = stateIn.x[i];
}

NormState& NormState::operator=(const NormState &stateIn){

  if (this == &stateIn)
    return *this;

  for (int i = 0; i < 5; i++)
    x[i] = stateIn.x[i];

  return *this;
}

NormTrajectory::NormTrajectory () {
	lor.metric_cost = -1.0;
	lor.risk = -1.0;
}

NormTrajectory::~NormTrajectory () {

}

NormTrajectory::NormTrajectory (const NormTrajectory &trajectoryIn) : endState(trajectoryIn.getEndState()) {

}

NormTrajectory& NormTrajectory::operator=(const NormTrajectory &trajectoryIn) {

	if (this == &trajectoryIn)
		return *this;

	endState = trajectoryIn.getEndState();

	lor = trajectoryIn.lor;

	return *this;
}

int NormTrajectory::getEndState (NormState &getEndStateOut) {

  getEndStateOut = endState;

  return 1;
}

double NormTrajectory::evaluateCost () {

  return lor.metric_cost;
}

double NormTrajectory::evaluateRisk() {
	return lor.risk;
}

NormSystem::NormSystem (){

	distance_limit = 100;
	delta_distance = 0.05;

	risk_limit = 0.5;
	risk_evaluate = new RiskEvaluate();
	double factor_reduce_size = 1.0;
	car_width = 1.2/factor_reduce_size;
	car_height = 2.28/factor_reduce_size;
	safe_distance = 0.0/factor_reduce_size;    // car footprint is blown up by this distance for collision checking
	distance_rear_axis_rear = 0.45/factor_reduce_size; // dist between center of rear axis and rear end
}

NormSystem::~NormSystem () {

}

int NormSystem::setRiskLimit(double risk){
	risk_limit = risk;
	return 1;
}

int NormSystem::propagatePose(double stateIn[5], double (&stateOut)[5], double theta, double increment){

	double linear_vel = 2.0;

	double td = increment / linear_vel;
	double gamma;

	stateOut[0] = stateIn[0] + linear_vel * td * cos(theta);
	stateOut[1] = stateIn[1] + linear_vel* td  * sin(theta);

	double x_conv = stateIn[2];
	double y_conv = stateIn[3];
	double xy_conv = stateIn[4];

	double temp_A_1 = -linear_vel*td*sin(theta);
	double temp_A_2 = linear_vel*td*cos(theta);

	double temp_V_1 = cos(theta);
	double temp_V_2 = sin(theta);

	Matrix2d A,  P_In, P_Out;
	double M = 0.15;
	MatrixXd V(2,1);

	//Eigen Matrix multiplication
	A << 1, 0, 0 , 1;
	P_In << x_conv, xy_conv, xy_conv, y_conv;
	V << temp_V_1 ,temp_V_2;

	P_Out = A * P_In * A.transpose() + V * M * V.transpose();

	stateOut [2] = P_Out(0,0);
	stateOut [3] = P_Out(1,1);
	stateOut [4] = P_Out(0,1);

	if (false){
		cout << "Debug" <<endl;
		cout << "x_conv_in: "<< x_conv <<" y_conv_in: " << y_conv<<endl;
		cout << "A_1: " <<temp_A_1 <<" A_2: " <<temp_A_2<< endl;
		cout << "V_1:" <<temp_V_1 << " V_2: " << temp_V_2 <<endl;
		cout << "x_conv_out: "<< stateOut [3] <<" y_conv_out: " << stateOut [4] <<"t_conv_out: "<< stateOut [5] <<endl;

		first_debug = false;
	}
	return 1;
}

int NormSystem::evaluateRisk(double stateIn[5], vector<double>& risk){
	//double temp_x, temp_y, temp_t;
	//transform_map_to_local_map(stateIn, temp_x, temp_y, temp_t);
	risk_evaluate->sample_pose.x = stateIn[0];//temp_x + map.info.height*map.info.resolution/4;
	risk_evaluate->sample_pose.y = stateIn[1];//temp_y + map.info.width*map.info.resolution/2;
	risk_evaluate->sample_pose.r = 0;//temp_t;
	risk_evaluate->sample_pose.conv_x = stateIn[2];
	risk_evaluate->sample_pose.conv_y = stateIn[3];
	risk_evaluate->sample_pose.conv_xy = stateIn[4];
	risk_evaluate->CalcaulateRisk(risk_evaluate->constraints, risk_evaluate->sample_pose, risk);
	return 1;
}

int NormSystem::getStateKey (NormState &stateIn, double *stateKey) {

	for (int i = 0; i < 2; i++)
		stateKey[i] =  (stateIn.x[i] - map_origin[i]);
	return 1;
}

#define SQ(x)   ((x)*(x))
float NormSystem::getGoalCost(const double x[5]){
	return (sqrt(SQ(x[0]-regionGoal.center[0]) + SQ(x[1]-regionGoal.center[1])));
}

bool NormSystem::isReachingTarget (NormState &stateIn) {
	for (int i = 0; i < 2; i++) {
		if (fabs(stateIn.x[i] - regionGoal.center[i]) > regionGoal.size[i]/3.0 )
			return false;
	}
	return true;
}

inline
int NormSystem::transform_map_to_local_map(const double stateIn[5], double &zlx, double &zly){
  /*
   * X_map
   * |     Y_local    X_car|
   * |      |              |
   * |      |   Y_car------|
   * |      |
   * |		|-----------------X_local
   * |
   * |
   * |--------- Y_map
   */
  // map frame z, yaw
	double zm[2] = {stateIn[0], stateIn[1]};

	double cos_map_yaw = cos(map_origin[2]);
	double sin_map_yaw = sin(map_origin[2]);

	// rotate zm by yaw, subtract map_origin to get zlocal
	zlx = (zm[0]-map_origin[0])*cos_map_yaw + (zm[1]-map_origin[1])*sin_map_yaw;
	zly = -(zm[0]-map_origin[0])*sin_map_yaw + (zm[1]-map_origin[1])*cos_map_yaw;
 	return 0;
}

inline
int NormSystem::get_cell_index(double x, double y, int &map_index){
	// find cells corresponding to (x,y)
	// car is placed at (height/4, width/2) according to the local_map

	int cellx = x/map.info.resolution + map.info.height/4.0;
	int celly = map.info.width/2.0 - y/map.info.resolution;

	//int cellx = x/map.info.resolution;
	//int celly = y/map.info.resolution;

	if( (cellx >=0) && (cellx < (int)map.info.height) && (celly >= 0) && (celly < (int)map.info.width)){
		map_index = cellx*map.info.width + celly;
		return 0;
	}
	else{
		map_index = -1;
		return 1;
	}
}

inline
int NormSystem::getxy_from_index(double &x, double &y, const int index){
	//Process the index in /base_link frame
	if( (index >= (int)map.info.height*map.info.width) || (index < 0))
		return 1;

	double cx = (index/map.info.width)*map.info.resolution;
	double cy = (index%map.info.width)*map.info.resolution;

	x = cx - map.info.height*map.info.resolution/4.0;
	y  = map.info.width*map.info.resolution/2.0 - cy;

	return 0;
}

bool NormSystem::IsInCollisionLazy (const double stateIn[5]){
	// (x,y) in local_map frame
	double zl[2] = {0};
	// yaw in local frame
	transform_map_to_local_map(stateIn, zl[0], zl[1]);
	//cout<<"zl: "<< zl[0]<<" "<<zl[1]<<" "<<yl<<endl;

	bool is_obstructed = false;

	int map_index = -1;

	if(get_cell_index(zl[0],zl[1], map_index) == 0){
		int to_check = map.data[map_index];
		if(to_check == 127){
			is_obstructed = true;
			return is_obstructed;
		}
	}
	else{
		is_obstructed = false;
	}
	return is_obstructed;
}

double NormSystem::extend_line (
		double state_ini[5], double state_fin[5],
		bool return_trajectory,bool check_risk,vector<double>& obst_risk,
		bool& fully_extends, double*& end_state,
		list<double*>* trajectory, list<float> &control) {

	double x_start = state_ini[0];
	double y_start = state_ini[1];
	double x_end = state_fin[0];
	double y_end = state_fin[1];

	double theta = atan2((y_end-y_start), (x_end-x_start));

	double line_distance;
	line_distance = sqrt ((x_start-x_end)*(x_start-x_end) + (y_start-y_end)*(y_start-y_end) );

	fully_extends = false;

	// Generate states/inputs
	double del_d = delta_distance;
	int max_counter = map.info.resolution/del_d;

	double state_curr[5] = {0.0};
	double state_in[5];
	double state_out[5];
	double temp_d;

	vector<double> temp_risk;

	for (int i =0; i < 5; i++)
		state_in[i] = state_ini[i];
	int obs_check_counter = 0;

	double d_inc_curr = 0.0;
	while (d_inc_curr < line_distance){
		d_inc_curr += del_d;
		if (d_inc_curr > line_distance){
			temp_d = d_inc_curr = del_d;
			d_inc_curr = line_distance;
		}
		state_curr[0] = (x_end - x_start) * d_inc_curr / line_distance + x_start;
		state_curr[1] = (y_end - y_start) * d_inc_curr / line_distance + y_start;

		obs_check_counter++;

		if(obs_check_counter == max_counter || d_inc_curr == line_distance){

			if (IsInCollisionLazy (state_curr))
				return -2.0;

			if (check_risk){

				vector<double> risk;
				double dist_inc = 0;

				if (obs_check_counter == max_counter)
					dist_inc = del_d * max_counter;
				if (d_inc_curr == line_distance)
					dist_inc = obs_check_counter*del_d;

				propagatePose(state_in, state_out, theta, dist_inc);

				evaluateRisk(state_out, risk);

				if (temp_risk.size() == 0){
					for (int i = 0 ; i < risk.size(); i++){
						temp_risk.push_back(risk[i]);
					}
				}

				for (int i =0 ; i < risk.size(); i++){
					if (risk[i] > risk_limit)
						return -3.0;
					if (temp_risk[i] < risk[i])
						temp_risk[i] = risk[i];
				}
				for (int i = 0; i < 5; i++)
					state_in[i] = state_out[i];
			}
			obs_check_counter = 0;
		}

		if (trajectory || return_trajectory){
			double *state_new = new double[5];
			for (int i = 0; i < 5; i++)
				state_new[i] = state_curr[i];
			if (trajectory)
				trajectory->push_front(state_new);
			if (return_trajectory)
				control.push_front (0.0);
		}
	}

	fully_extends = true;

	for (int i = 0; i < 5; i++)
		end_state[i] = state_out[i];
	for (int i = 0; i < temp_risk.size(); i++){
		obst_risk.push_back(temp_risk[i]);
	}
	return line_distance;
}

int NormSystem::extendTo (
		NormState &stateFromIn, NormState &stateTowardsIn,
		bool check_obstacles, bool check_risk,
		NormTrajectory &trajectoryOut, bool &exactConnectionOut, list<float> &controlOut) {

	double *end_state = new double [5];

	double *tmp_end_state = new double [5];
	bool tmp_exact_connection = false;
	list<float> tmp_control;
	vector<double> tmp_risk;

	Level_OF_Risk temp_lor;
	temp_lor.metric_cost = DBL_MAX;
	temp_lor.risk = DBL_MAX;

	double time_cost = extend_line (
			stateFromIn.x, stateTowardsIn.x,
			false, check_risk, tmp_risk,
			tmp_exact_connection, tmp_end_state, NULL, tmp_control);

	if (time_cost > 0){
		double max_risk = 0;
		for (int i = 0 ; i < tmp_risk.size() ; i ++){
			if (tmp_risk[i] > max_risk)
				max_risk = tmp_risk[i];
		}
		temp_lor.metric_cost = time_cost;
		temp_lor.risk = max_risk;
	}

	//ROS_INFO("Enxtend to Function risk check again, %f", temp_lor.risk);

	for(int j=0; j<5; j++)
		end_state[j] = tmp_end_state[j];
	exactConnectionOut = tmp_exact_connection;
	controlOut = tmp_control;
	delete[] tmp_end_state;

	if((temp_lor.metric_cost <= 0.0) || (temp_lor.metric_cost > DBL_MAX/2)){
		delete[] end_state;
		return -1;
	}

	for (int i = 0; i < 5; i++) {
		trajectoryOut.endState.x[i] = end_state[i];
	}

	trajectoryOut.lor = temp_lor;

	delete [] end_state;
	return 1;
}

Level_OF_Risk NormSystem::evaluateExtension (NormState &stateFromIn, NormState &stateTowardsIn, bool &exactConnectionOut, bool check_risk){

	double *end_state = new double[5];

	bool tmp_exact_connection = false;
	vector<double> temp_risk;
	list<float> tmp_control;

	Level_OF_Risk temp_lor;
	temp_lor.metric_cost = DBL_MAX;
	temp_lor.risk = DBL_MAX;

	double time_cost = extend_line (
			stateFromIn.x, stateTowardsIn.x,
			true, check_risk, temp_risk,
			tmp_exact_connection, end_state, NULL, tmp_control);

	if (time_cost > 0){
		double max_risk = 0;
		for (int i = 0 ; i < temp_risk.size() ; i ++){
			if (temp_risk[i] > max_risk)
				max_risk = temp_risk[i];
		}
		temp_lor.metric_cost = time_cost;
		temp_lor.risk = max_risk;
	}
	exactConnectionOut = tmp_exact_connection;

	delete[] end_state;

	return temp_lor;
}

int NormSystem::getTrajectory (
		NormState& stateFromIn, NormState& stateToIn,
		list<double*>& trajectoryOut, list<float>& controlOut,
		bool check_obstacles, bool check_risk) {

	Level_OF_Risk temp_lor;
	temp_lor.metric_cost = DBL_MAX;
	temp_lor.risk = DBL_MAX;

	bool exactConnectionOut = false;
	list<double*> tmp_traj;
	list<float> tmp_control;
	bool tmp_exact_connection = false;
	vector<double> temp_risk;
	double *end_state = new double[5];

	double time = extend_line (
			stateFromIn.x, stateToIn.x,
			true, check_risk, temp_risk,
			tmp_exact_connection, end_state, &tmp_traj, tmp_control);

	if (time > 0){
		temp_lor.metric_cost = time;
		double max_risk = 0;
		for (int i = 0 ; i < temp_risk.size() ; i ++){
			if (temp_risk[i] > max_risk)
				max_risk = temp_risk[i];
		}
		temp_lor.risk = max_risk;

		trajectoryOut = tmp_traj;
		controlOut = tmp_control;
		exactConnectionOut = tmp_exact_connection;
	}

	delete [] end_state;

	if ((temp_lor.metric_cost <= 0.0) || (temp_lor.metric_cost > DBL_MAX/2)){
		return 0;
	}

	trajectoryOut.reverse();
	return 1;
}

double NormSystem::evaluateCostToGo (NormState& stateIn){
	double size_x = regionGoal.size[0];
	double size_y = regionGoal.size[1];
	double radius = sqrt(size_x*size_x + size_y*size_y);

	double dist_x = stateIn.x[0] - regionGoal.center[0];
	double dist_y = stateIn.x[1] - regionGoal.center[1];
	double dist = sqrt (dist_x*dist_x + dist_y*dist_y);

	return dist - radius;
}

int NormSystem::clear_tmp_trajectories(list<double*> &state_traj, list<float> &control_traj){
	for(list<double*>::iterator i=state_traj.begin(); i!= state_traj.end(); i++){
		delete[] (*i);
	}
	state_traj.clear();
	control_traj.clear();
	return 0;
}

int NormSystem::sampleState(NormState &randomStateOut){
	int r = (double)(rand()/(RAND_MAX+1.0))*free_cells.size();

	int index = free_cells[r];
	double local_xy[2] ={0, 0};
	getxy_from_index(local_xy[0], local_xy[1], index);

	for(int i=0; i<2; i++){
		randomStateOut.x[i] = local_xy[i] + (double)rand()/(RAND_MAX + 1.0)*regionCell.size[i] ;
	}

	// transform the sample from /base_link frame to /map frame
	double cyaw = cos(-map_origin[2]);
	double syaw = sin(-map_origin[2]);
	double state_copy[3];
	for(int i=0; i<2; i++)
		state_copy[i] = randomStateOut[i];

	randomStateOut.x[0] = map_origin[0] + state_copy[0]*cyaw + state_copy[1]*syaw;
	randomStateOut.x[1] = map_origin[1] + -state_copy[0]*syaw + state_copy[1]*cyaw;
	randomStateOut.x[2] =0;
	randomStateOut.x[3] =0;
	randomStateOut.x[4] =0;

	if (IsInCollisionLazy (randomStateOut.x))
		return 0;

	return 1;
}

int NormSystem::sampleGoalState (NormState &randomStateOut) {

	for (int i = 0; i < 2; i++)
		randomStateOut.x[i] = (double)rand()/(RAND_MAX + 1.0)*regionGoal.size[i]- regionGoal.size[i]/2.0 + regionGoal.center[i];

	for (int i = 2; i < 5 ; i++)
		randomStateOut.x[i] =0;

	if (IsInCollisionLazy (randomStateOut.x))
		return 0;
	return 1;
}

double NormSystem::norm_state(double s1[5], double s2[5]){
	double t = 0;
	for(int i=0; i<2; i++)
		t = t + (s1[i]-s2[i])*(s1[i]-s2[i]);
	return sqrt(t);
}
