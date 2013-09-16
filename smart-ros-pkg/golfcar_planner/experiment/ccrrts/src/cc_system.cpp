/*
 * System.cpp
 *
 *  Created on: Jul 24, 2013
 *      Author: liuwlz
 */

#include "cc_system.h"

bool first_debug = true;

CCState::CCState () {

}

CCState::~CCState() {

}

CCState::CCState (const CCState &stateIn) {

  for (int i = 0; i < 7; i++)
    x[i] = stateIn.x[i];
}

CCState& CCState::operator=(const CCState &stateIn){

  if (this == &stateIn)
    return *this;

  for (int i = 0; i < 7; i++)
    x[i] = stateIn.x[i];

  return *this;
}

CCTrajectory::CCTrajectory () {
	dubinsRadius = DBL_MAX;
	lor.metric_cost = DBL_MAX;
	lor.risk = DBL_MAX;
}

CCTrajectory::~CCTrajectory () {

}

CCTrajectory::CCTrajectory (const CCTrajectory &trajectoryIn) {
	dubinsRadius = trajectoryIn.dubinsRadius;
	endState = trajectoryIn.getEndState();
	lor.risk = trajectoryIn.lor.risk;
	lor.metric_cost = trajectoryIn.lor.metric_cost;
}

CCTrajectory& CCTrajectory::operator=(const CCTrajectory &trajectoryIn) {

	if (this == &trajectoryIn)
		return *this;

	dubinsRadius = trajectoryIn.dubinsRadius;

	endState = trajectoryIn.getEndState();

	lor = trajectoryIn.lor;

	return *this;
}

int CCTrajectory::getEndState (CCState &getEndStateOut) {

  getEndStateOut = endState;

  return 1;
}

double CCTrajectory::getDubinsRadius(){
	return dubinsRadius;
}

double CCTrajectory::evaluateCost () {

  return lor.metric_cost;
}

double CCTrajectory::evaluateRisk() {
	return lor.risk;
}

CCSystem::CCSystem (){
	turning_radii[0] = 3.0;
	turning_radii[1] = 4;
	turning_radii[2] = 6;

	distance_limit = 30.0;
	delta_distance = 0.1;

	risk_limit = 0.15;
	risk_evaluate = new RiskEvaluate();
	double factor_reduce_size = 1.0;
	car_width = 1.2/factor_reduce_size;
	car_height = 2.28/factor_reduce_size;
	safe_distance = 0.0/factor_reduce_size;    // car footprint is blown up by this distance for collision checking
	distance_rear_axis_rear = 0.45/factor_reduce_size; // dist between center of rear axis and rear end
}

CCSystem::~CCSystem () {

}

inline double ModTo2Pi(double theta){
	while (theta < 0)
		theta += 2.0 * M_PI;
	while (theta > 2.0*M_PI)
		theta -= 2.0*M_PI;
	return theta;
}

int CCSystem::setRiskLimit(double risk){
	risk_limit = risk;
	return 1;
}

int CCSystem::propagatePoseDubins(double stateIn[7], double (&stateOut)[7], double turning_radius, double increment, double direction){

	double linear_vel = 2.0;
	double rear_dist = distance_rear_axis_rear;

	double td;
	double gamma;
	if(turning_radius < DBL_MAX/2){
		td = turning_radius*increment/linear_vel;
		gamma= direction * atan(rear_dist/turning_radius);
	}
	else{
		td = increment / linear_vel;
		gamma = 0;
	}
	double theta = stateIn[2];

	stateOut[0] = stateIn[0] + linear_vel * td * cos(theta);
	stateOut[1] = stateIn[1] + linear_vel* td  * sin(theta);
	stateOut[2] = stateIn[2] + linear_vel*td*tan(gamma)/rear_dist;

	while (stateOut[2] < -M_PI)
		stateOut[2] += 2.0 * M_PI;
	while (stateOut[2] > M_PI)
		stateOut[2] -= 2.0*M_PI;

	//Update Pose Convariance matrix
	/*
	 * Linearise the error x^ between real state x and nominal x*, i.e. x^ = x - x*;
	 * x^_t+1 = A_t * x^_t + B_t * u^_t;
	 * P^_t+1 = A_t * P^_t * A_t.transpose() + V_t * M * V_t.transpose();
	 * If considering feedback control, then u^_t = L * x^_t, thus x^_t+1 = (A_t + B_t * L_t) x^_t;
	 */
	double x_conv = stateIn[3];
	double y_conv = stateIn[4];
	double t_conv = stateIn[5];

	double temp_P = stateIn[6];

	double temp_A_1 = -linear_vel*td*sin(theta);
	double temp_A_2 = linear_vel*td*cos(theta);

	double temp_V_1 = cos(theta);
	double temp_V_2 = sin(theta);
	double temp_V_3 = tan(gamma)/rear_dist;
	double temp_V_4 = linear_vel*td/(rear_dist*cos(gamma)*cos(gamma));

	Matrix3d A,  P_In, P_Out;
	Matrix2d M;
	MatrixXd V(3,2);

	//Eigen Matrix multiplication
	A << 1, 0, temp_A_1, 0 , 1, temp_A_2, 0 , 0, 1;
	P_In << x_conv, temp_P, 0, temp_P, y_conv, 0, 0, 0, t_conv;
	M << 0.09, 0.0, 0.0, (M_PI/180)*(M_PI/180);
	V << temp_V_1 ,0, temp_V_2, 0, temp_V_3, temp_V_4;

	P_Out = A * P_In * A.transpose() + V * M * V.transpose();

	stateOut [3] = P_Out(0,0);
	stateOut [4] = P_Out(1,1);
	stateOut [5] = P_Out(2,2);
	stateOut [6] = P_Out(0,1);

	if (false){
		cout << "Debug" <<endl;
		cout << "x_conv_in: "<< x_conv <<" y_conv_in: " << y_conv <<"t_conv_in: "<< t_conv <<endl;
		cout << "A_1: " <<temp_A_1 <<" A_2: " <<temp_A_2<< endl;
		cout << "V_1:" <<temp_V_1 << " V_2: " << temp_V_2 << "V_3:" <<temp_V_3 << "V_4:" <<temp_V_4<<endl;
		cout << "x_conv_out: "<< stateOut [3] <<" y_conv_out: " << stateOut [4] <<"t_conv_out: "<< stateOut [5] <<endl;

		first_debug = false;
	}
	return 1;
}

int CCSystem::evaluateRisk(double stateIn[7], vector<double>& risk){
	//double temp_x, temp_y, temp_t;
	//transform_map_to_local_map(stateIn, temp_x, temp_y, temp_t);
	risk_evaluate->sample_pose.x = stateIn[0];//temp_x + map.info.height*map.info.resolution/4;
	risk_evaluate->sample_pose.y = stateIn[1];//temp_y + map.info.width*map.info.resolution/2;
	risk_evaluate->sample_pose.r = stateIn[2];//temp_t;
	risk_evaluate->sample_pose.conv_x = stateIn[3];
	risk_evaluate->sample_pose.conv_y = stateIn[4];
	risk_evaluate->sample_pose.conv_xy = stateIn[6];
	risk_evaluate->CalcaulateRisk(risk_evaluate->constraints, risk_evaluate->sample_pose, risk);
	return 1;
}

int CCSystem::getStateKey (CCState &stateIn, double *stateKey) {

	double tmp[3] = {0};
	transform_map_to_local_map(stateIn.x, tmp[0], tmp[1], tmp[2]);

	tmp[0] = tmp[0] + map.info.height*map.info.resolution/4;
	tmp[1] = tmp[1] + map.info.width*map.info.resolution/2;

	for (int i = 0; i < 3; i++)
		stateKey[i] =  tmp[i] / regionOperating.size[i];

	//ROS_INFO("getStateKey: State: x: %f, y: %f t: %f. State Key: x: %f, y: %f, t: %f ", stateIn[0], stateIn[1], stateIn[2], stateKey[0], stateKey[1], stateKey[2]);

	return 1;
}

#define SQ(x) ((x)*(x))
float CCSystem::getGoalCost(const double x[7]){
	double tmp = x[2] - regionGoal.center[2];
	while(tmp < -M_PI)
		tmp += 2.0*M_PI;
	while(tmp > M_PI)
		tmp -= 2.0*M_PI;
	return (sqrt(SQ(x[0]-regionGoal.center[0]) + SQ(x[1]-regionGoal.center[1])) + 4.0*fabs(tmp));
}

bool CCSystem::isReachingTarget (CCState &stateIn) {
	for (int i = 0; i < 3; i++) {
		if (fabs(stateIn.x[i] - regionGoal.center[i]) > regionGoal.size[i]/3.0 )
			return false;
	}
	return true;
}

inline
int CCSystem::transform_map_to_local_map(const double stateIn[7], double &zlx, double &zly, double &yl){
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
	double ym = stateIn[2];

	double cos_map_yaw = cos(map_origin[2]);
	double sin_map_yaw = sin(map_origin[2]);

	// rotate zm by yaw, subtract map_origin to get zlocal
	zlx = (zm[0]-map_origin[0])*cos_map_yaw + (zm[1]-map_origin[1])*sin_map_yaw;
	zly = -(zm[0]-map_origin[0])*sin_map_yaw + (zm[1]-map_origin[1])*cos_map_yaw;
	yl = ym - map_origin[2];
	while(yl > M_PI)
		yl -= 2.0*M_PI;
	while(yl < -M_PI)
		yl += 2.0*M_PI;
 	return 0;
}

inline
int CCSystem::get_cell_index(double x, double y, int &map_index){
	// find cells corresponding to (x,y)
	// car is placed at (height/4, width/2) according to the local_map

	int cellx = x/map.info.resolution + map.info.height/4.0;
	int celly = map.info.width/2.0 - y/map.info.resolution;

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
int CCSystem::getxy_from_index(double &x, double &y, const int index){
	//Process the index in /base_link frame
	if( (index >= (int)map.info.height*map.info.width) || (index < 0))
		return 1;

	double cx = (index/map.info.width)*map.info.resolution;
	double cy = (index%map.info.width)*map.info.resolution;

	x = cx - map.info.height*map.info.resolution/4.0;
	y  = map.info.width*map.info.resolution/2.0 - cy;

	return 0;
}

bool CCSystem::IsInCollision (const double stateIn[7]){
	// (x,y) in local_map frame
	double zl[2] = {0};
	// yaw in local frame
	double yl = 0;
	transform_map_to_local_map(stateIn, zl[0], zl[1], yl);
	//cout<<"zl: "<< zl[0]<<" "<<zl[1]<<" "<<yl<<endl;

	double cos_yl = cos(yl);
	double sin_yl = sin(yl);

	bool is_obstructed = false;
	double cxmax = car_height - distance_rear_axis_rear + safe_distance + 0.001;
	double cymax = car_width/2.0 + safe_distance + 0.001;
	double cy = - car_width/2.0 - safe_distance;

	while(cy < cymax){
		double cx = - distance_rear_axis_rear - safe_distance;
		while(cx < cxmax){
			// x = stateInLocal + rel position (cx,cy) transformed into the (X_car,Y_car) frame
			double x = zl[0] + cx*cos_yl + cy*sin_yl;
			double y = zl[1] - cx*sin_yl + cy*cos_yl;
			int map_index = -1;
			if(get_cell_index(x, y, map_index) == 0){
				int to_check = map.data[map_index];
				if(to_check == 127){
					is_obstructed = true;
					return is_obstructed;
				}
			}
			else{
				is_obstructed = false;
			}
			cx = cx + map.info.resolution;
		}
		cy = cy + map.info.resolution;
	}
	return is_obstructed;
	return true;
}

bool CCSystem::IsInCollisionLazy (const double stateIn[7]){
	double zl[2] = {0};
	double yl = 0;
	transform_map_to_local_map(stateIn, zl[0], zl[1], yl);

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

double CCSystem::getLaneCost(const double zx, const double zy){
	int map_index = -1;
	if(get_cell_index(zx, zy, map_index) == 0){
		int val = map.data[map_index];
		if(val == 127){
			return 100;
		}
		else
			return val/127*10;
	}
	else
		return 10;
}

double CCSystem::getStateCost(const double stateIn[7]){
	// (x,y) in local_map frame
	double zlc[2] = {0};
	// yaw in local frame
	double ylc = 0;
	transform_map_to_local_map(stateIn, zlc[0], zlc[1], ylc);

	double cost = 0;

	double zll[2] = {0};
	double zlr[2] = {0};
	zlr[0] = zlc[0] + car_width/2.0*sin(ylc);
	zlr[1] = zlc[1] - car_width/2.0*cos(ylc);
	zll[0] = zlc[0] - car_width/2.0*sin(M_PI/2.0 - ylc);
	zll[1] = zlc[1] + car_width/2.0*cos(M_PI/2.0 - ylc);

	cost += getLaneCost(zlc[0], zlc[1]);
	cost += getLaneCost(zll[0], zll[1]);
	cost += getLaneCost(zlr[0], zlr[1]);
	return cost/3.0;
}

double CCSystem::extend_dubins_spheres (
		double x_s1, double y_s1, double t_s1,
		double x_s2, double y_s2, double t_s2,
		int combination_no,
		bool check_obstacles, bool return_control, bool check_risk, vector<double>& obst_risk,
		bool& fully_extends, double* &end_state, double* &init_state,
		list<double*>* trajectory, list<float> &control, double turning_radius) {

	double x_tr = x_s2 - x_s1;
	double y_tr = y_s2 - y_s1;
	double t_tr = atan2 (y_tr, x_tr);

	double distance = sqrt ( x_tr*x_tr + y_tr*y_tr);

	double x_start;
	double y_start;
	double t_start = 0;
	double x_end;
	double y_end;
	double t_end = 0;

	if (distance > 2 * turning_radius){
		// disks do not intersect
		double t_balls = acos (2 * turning_radius / distance);
		switch (combination_no){
		case 1:
			t_start = t_tr - t_balls;
			t_end = t_tr + M_PI - t_balls;
			break;
		case 2:
			t_start = t_tr + t_balls;
			t_end = t_tr - M_PI + t_balls;
			break;
		case 3:
			t_start = t_tr - M_PI_2;
			t_end = t_tr - M_PI_2;
			break;
		case 4:
			t_start = t_tr + M_PI_2;
			t_end = t_tr + M_PI_2;
			break;
		default:
			return -1.0;
		}
	}
	else {
    // disks are intersecting
		switch (combination_no) {
		case 1:
		case 2:
			//Case 1 and case 2 No solution
			return -1.0;
			break;
		case 3:
			t_start = t_tr - M_PI_2;
			t_end = t_tr - M_PI_2;
			break;
		case 4:
			t_start = t_tr + M_PI_2;
			t_end = t_tr + M_PI_2;
			break;
		}
	}

	x_start = x_s1 + turning_radius * cos (t_start);
	y_start = y_s1 + turning_radius * sin (t_start);
	x_end = x_s2 + turning_radius * cos (t_end);
	y_end = y_s2 + turning_radius * sin (t_end);

	int direction_s1 = 1;
	if ( (combination_no == 2) || (combination_no == 4) ) {
		direction_s1 = -1;
	}
	int direction_s2 = 1;
	if ( (combination_no == 1) || (combination_no == 4) ) {
		direction_s2 = -1;
	}

	double t_increment_s1 = direction_s1 * (t_start - t_s1);
	double t_increment_s2 = direction_s2 * (t_s2 - t_end);

	while (t_increment_s1 < 0)
		t_increment_s1 += 2.0 * M_PI;
	while (t_increment_s1 > 2.0 *M_PI)
		t_increment_s1 -= 2.0 * M_PI;

	while (t_increment_s2 < 0)
		t_increment_s2 += 2.0 * M_PI;
	while (t_increment_s2 > 2.0 *M_PI)
		t_increment_s2 -= 2.0 * M_PI;

	if  ( ( (t_increment_s1 > M_PI) && (t_increment_s2 > M_PI) )
			|| ( (t_increment_s1 > 3*M_PI_2) || (t_increment_s2 > 3*M_PI_2) )  ){
		return -1.0;
	}

	double line_distance;
	line_distance = sqrt ( (x_start-x_end)*(x_start-x_end) + (y_start-y_end)*(y_start-y_end) );

	double distanceTravel = ( t_increment_s1 + t_increment_s2) * turning_radius  + 	line_distance;

	fully_extends = false;

	if (check_obstacles) {
		// Generate states/inputs
		double del_d = delta_distance;
		double del_t = del_d/turning_radius;

		double state_curr[7] = {0.0};
		double state_in[7];
		double state_out[7];

		double extend_inc_1 = 0;
		bool should_break = false;
		bool over_limit = false;

		vector<double> temp_risk;

		for (int i =0; i < 7; i++)
			state_in[i] = init_state[i];

		double t_inc_curr_1 = 0.0;
		double t_inc_old_1 = 0.0;
		while ((t_inc_curr_1 < t_increment_s1) && (!over_limit)) {

			t_inc_curr_1 += del_t;

			if (t_inc_curr_1 > t_increment_s1) {
				t_inc_curr_1 = t_increment_s1;
				should_break = true;
			}

			if (t_inc_curr_1*turning_radius > distance_limit){
				t_inc_curr_1 = distance_limit/turning_radius;
				over_limit = true;
			}

			extend_inc_1 = t_inc_curr_1 - t_inc_old_1;

			state_curr[0] = x_s1 + turning_radius * cos (direction_s1 * t_inc_curr_1 + t_s1);
			state_curr[1] = y_s1 + turning_radius * sin (direction_s1 * t_inc_curr_1 + t_s1);
			state_curr[2] = direction_s1 * t_inc_curr_1 + t_s1 + ( (direction_s1 == 1) ? M_PI_2 : ( 3.0 * M_PI_2) );

			while (state_curr[2] < 0)
				state_curr[2] += 2 * M_PI;
			while (state_curr[2] > 2 * M_PI)
				state_curr[2] -= 2 * M_PI;

			if (IsInCollisionLazy (state_curr)){
				return -2.0;
			}

			if (check_risk){
				vector<double> risk;
				double angle_inc_s1 =0;

				propagatePoseDubins(state_in, state_out, turning_radius, extend_inc_1, direction_s1);
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
				for (int i = 0; i < 7; i++)
					state_in[i] = state_out[i];
			}

			if (trajectory || return_control){
				double *state_new = new double[7];
				for (int i = 0; i < 7; i++)
					state_new[i] = state_out[i];
				if (trajectory)
					trajectory->push_front(state_new);
				if (return_control)
					control.push_front (direction_s1*turning_radius);
			}

			t_inc_old_1 = t_inc_curr_1;
			if (should_break || over_limit){
				should_break = false;
				break;
			}
		}

		double extend_inc = 0;
		should_break = false;
		double d_inc_curr = 0.0;
		double d_inc_old = 0.0;

		while ((d_inc_curr < line_distance) && (!over_limit)){

			d_inc_curr += del_d;

			if (d_inc_curr > line_distance){
				d_inc_curr = line_distance;
				should_break = true;
			}

			double current_total_dist = turning_radius*t_increment_s1 + d_inc_curr;

			if (current_total_dist > distance_limit ){
				d_inc_curr = distance_limit - turning_radius*t_increment_s1;
				over_limit = true;
			}

			extend_inc = d_inc_curr - d_inc_old;

			state_curr[0] = (x_end - x_start) * d_inc_curr / line_distance + x_start;
			state_curr[1] = (y_end - y_start) * d_inc_curr / line_distance + y_start;
			state_curr[2] = atan2((y_end-y_start), (x_end-x_start));

			while (state_curr[2] < -M_PI)
				state_curr[2] += 2.0 * M_PI;
			while (state_curr[2] > M_PI)
				state_curr[2] -= 2.0*M_PI;

			if (IsInCollisionLazy (state_curr))
				return -2.0;

			if (check_risk){

				vector<double> risk;

				propagatePoseDubins(state_in, state_out, DBL_MAX, extend_inc, 0);
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
				for (int i = 0; i < 7; i++)
					state_in[i] = state_out[i];
			}

			if (trajectory || return_control){
				double *state_new = new double[7];
				for (int i = 0; i < 7; i++)
					state_new[i] = state_out[i];
				if (trajectory)
					trajectory->push_front(state_new);
				if (return_control)
					control.push_front (0.0);
			}

			d_inc_old = d_inc_curr;

			if (should_break || over_limit){
				should_break = false;
				break;
			}
		}

		should_break = false;
		double extend_inc_2 = 0;
		double t_inc_curr_2 = 0.0;
		double t_inc_old_2 = 0.0;

		while ((t_inc_curr_2 < t_increment_s2) && (!over_limit)) {

			t_inc_curr_2 += del_t;

			if (t_inc_curr_2 > t_increment_s2){
				t_inc_curr_2 = t_increment_s2;
				should_break = true;
			}

			double current_total_dist = turning_radius*(t_increment_s1+t_inc_curr_1) + line_distance;

			if (current_total_dist > distance_limit){
				t_inc_curr_2 = (distance_limit - turning_radius*t_increment_s1 - line_distance)/turning_radius;
				over_limit = true;
			}

			extend_inc_2 = t_inc_curr_2 - t_inc_old_2;

			state_curr[0] = x_s2 + turning_radius * cos (direction_s2 * (t_inc_curr_2 - t_increment_s2) + t_s2);
			state_curr[1] = y_s2 + turning_radius * sin (direction_s2 * (t_inc_curr_2 - t_increment_s2) + t_s2);
			state_curr[2] = direction_s2 * (t_inc_curr_2 - t_increment_s2) + t_s2 + ( (direction_s2 == 1) ?  M_PI_2 : 3.0*M_PI_2 );

			while (state_curr[2] < -M_PI)
				state_curr[2] += 2.0 * M_PI;
			while (state_curr[2] > M_PI)
				state_curr[2] -= 2.0*M_PI;

			if (IsInCollisionLazy (state_curr))
				return -2.0;

			if (check_risk){

				vector<double> risk;

				propagatePoseDubins(state_in, state_out, turning_radius, extend_inc_2, direction_s2);

				while (state_out[2] < 0)
					state_out[2] += 2 * M_PI;
				while (state_out[2] > 2 * M_PI)
					state_out[2] -= 2 * M_PI;

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
				for (int i = 0; i < 7; i++)
					state_in[i] = state_out[i];
			}

			if (trajectory || return_control){
				double *state_new = new double[7];
				for (int i = 0; i < 7; i++)
					state_new[i] = state_out[i];
				if (trajectory)
					trajectory->push_front(state_new);
				if (return_control)
					control.push_front (direction_s2*turning_radius);
			}

			t_inc_old_2 = t_inc_curr_2;

			if (should_break || over_limit){
				should_break = false;
				break;
			}
		}

		fully_extends = true;
		for (int i = 0; i < 7; i++)
			end_state[i] = state_out[i];

		//ROS_INFO("ExtendSphere End state: state_curr: x: %f, y: %f, t: %f. Turning Radius: %f, Traj_length: %f",
		//		end_state[0], end_state[1], end_state[2], turning_radius, distanceTravel);

		for (int i = 0; i < temp_risk.size(); i++){
			obst_risk.push_back(temp_risk[i]);
		}

		if(over_limit)
			return distance_limit;
	}
	return distanceTravel;
}

bool compareTrajTimePairs( pair< int , double> traj_i, pair< int , double> traj_j) {
	return ( traj_j.second < traj_j.second );
}

double CCSystem::extend_dubins_all (
		double state_ini[7], double state_fin[7],
		bool check_obstacles, bool return_trajectory,bool check_risk,vector<double>& obst_risk,
		bool& fully_extends, double*& end_state,
		list<double*>* trajectory, list<float> &control, double turning_radius){

	double ti = state_ini[2];
	double tf = state_fin[2];
	double sin_ti = sin (-ti);
	double cos_ti = cos (-ti);
	double sin_tf = sin (-tf);
	double cos_tf = cos (-tf);

	//Get the centre of the Dubins spheres
	double si_left[3] = {
			state_ini[0] + turning_radius * sin_ti,
			state_ini[1] + turning_radius * cos_ti,
			ti + 3 * M_PI_2
	};
	double si_right[3] = {
			state_ini[0] - turning_radius * sin_ti,
			state_ini[1] - turning_radius * cos_ti,
			ti + M_PI_2
	};
	double sf_left[3] = {
			state_fin[0] + turning_radius * sin_tf,
			state_fin[1] + turning_radius * cos_tf,
			tf + 3 * M_PI_2
	};
	double sf_right[3] = {
			state_fin[0] - turning_radius * sin_tf,
			state_fin[1] - turning_radius * cos_tf,
			tf + M_PI_2
	};

	// 2. extend all four spheres
	double times[4];

	bool exact_connection[4];

	vector<double> temp_risk;

	times[0] = extend_dubins_spheres (si_left[0], si_left[1], si_left[2],
			sf_right[0], sf_right[1], sf_right[2], 1,
			false, false, false, temp_risk,
			exact_connection[0], end_state, state_ini,
			NULL, control, turning_radius);
	times[1] = extend_dubins_spheres (si_right[0], si_right[1], si_right[2],
			sf_left[0], sf_left[1], sf_left[2], 2,
			false, false, false, temp_risk,
			exact_connection[1], end_state, state_ini,
			NULL, control, turning_radius);
	times[2] = extend_dubins_spheres (si_left[0], si_left[1], si_left[2],
			sf_left[0], sf_left[1], sf_left[2], 3,
			false, false, false, temp_risk,
			exact_connection[2], end_state, state_ini,
			NULL, control, turning_radius);
	times[3] = extend_dubins_spheres (si_right[0], si_right[1], si_right[2],
			sf_right[0], sf_right[1], sf_right[2], 4,
			false, false, false, temp_risk,
			exact_connection[3], end_state,state_ini,
			NULL, control, turning_radius);

	temp_risk.empty();

	double min_time = DBL_MAX;
	int comb_min = -1;
	for (int i = 0; i < 4; i++) {
		if  ((times[i] >= 0.0) && (times[i] < DBL_MAX/2) && (times[i] < min_time)) {
			comb_min = i+1;
			min_time = times[i];
		}
		//else
		//	ROS_INFO("No Solution due to %f", times[i]);
	}
	if (comb_min == -1)
		return -1.0;

	if (check_obstacles == false) {
		fully_extends = exact_connection[comb_min-1];
		return min_time;
	}

	double min_dist;
	switch (comb_min) {
	case 1:
		min_dist = extend_dubins_spheres (si_left[0], si_left[1], si_left[2],
				sf_right[0], sf_right[1], sf_right[2], 1,
				true, return_trajectory, true, obst_risk,
				fully_extends, end_state, state_ini,
				trajectory, control, turning_radius);
		if (min_dist > 0.0)
			return min_dist;
		else
			return -1.0;

	case 2:
		min_dist = extend_dubins_spheres (si_right[0], si_right[1], si_right[2],
				sf_left[0], sf_left[1], sf_left[2], 2,
				true, return_trajectory, true, obst_risk,
				fully_extends, end_state, state_ini,
				trajectory, control, turning_radius);
		if (min_dist > 0.0)
			return min_dist;
		else
			return -1.0;

    case 3:
    	min_dist = extend_dubins_spheres (si_left[0], si_left[1], si_left[2],
    			sf_left[0], sf_left[1], sf_left[2], 3,
    			true, return_trajectory, true, obst_risk,
    			fully_extends, end_state, state_ini,
    			trajectory, control, turning_radius);
		if (min_dist > 0.0)
			return min_dist;
		else
			return -1.0;

    case 4:
    	min_dist = extend_dubins_spheres (si_right[0], si_right[1], si_right[2],
    			sf_right[0], sf_right[1], sf_right[2], 4,
    			true, return_trajectory, true, obst_risk,
    			fully_extends, end_state, state_ini,
    			trajectory, control, turning_radius);
		if (min_dist > 0.0)
			return min_dist;
		else
			return -1.0;

    case -1:
    default:
    	return -1.0;
	}
}

int CCSystem::extendTo (
		CCState &stateFromIn, CCState &stateTowardsIn,
		bool check_obstacles, bool check_risk,
		CCTrajectory &trajectoryOut, bool &exactConnectionOut, list<float> &controlOut) {

	while (stateFromIn.x[2] <0)
		stateFromIn.x[2] += 2*M_PI;
	while (stateFromIn.x[2] > 2*M_PI)
		stateFromIn.x[2] -= 2*M_PI;

	//ROS_INFO("ExtendTO: startIn: x %f, y: %f. StateOut: x %f, y %f", stateFromIn.x[0], stateFromIn.x[1], stateTowardsIn[0], stateTowardsIn[1]);

	/*
	//Limit the orientation change into [0, Pi]
	if (fabs(ModTo2Pi(stateFromIn[2])-ModTo2Pi(stateTowardsIn[2]) >= M_PI/2)){
		ROS_INFO("Theta change exceed the limit");
		return -1;
	}
	*/

	double *end_state = new double [7];
	Level_OF_Risk min_lor;
	min_lor.metric_cost = DBL_MAX;
	min_lor.risk = DBL_MAX;

	double best_turning_radius = 100.0;
	for(int i=num_turning_radii -1; i >= 0; i--){
		double *tmp_end_state = new double [7];
		double turning_radius = turning_radii[i];
		bool tmp_exact_connection = false;
		list<float> tmp_control;
		vector<double> tmp_risk;

		Level_OF_Risk temp_lor;
		temp_lor.metric_cost = DBL_MAX;
		temp_lor.risk = DBL_MAX;

		double time_cost = extend_dubins_all (
				stateFromIn.x, stateTowardsIn.x,
				check_obstacles, false, check_risk, tmp_risk,
				tmp_exact_connection, tmp_end_state, NULL, tmp_control, turning_radius);

		if(time_cost > 0.0 && time_cost < DBL_MAX/2){

			double max_risk = 0;
			for (int i = 0 ; i < tmp_risk.size() ; i ++){
				if (tmp_risk[i] > max_risk)
					max_risk = tmp_risk[i];
			}
			temp_lor.metric_cost = time_cost;
			temp_lor.risk = max_risk;

			if(temp_lor < min_lor){
				for(int j=0; j<7; j++)
					end_state[j] = tmp_end_state[j];
				min_lor = temp_lor;
				best_turning_radius = turning_radius;
				exactConnectionOut = tmp_exact_connection;
				controlOut = tmp_control;
			}
		}
		delete[] tmp_end_state;
		tmp_risk.clear();
	}

	if((min_lor.metric_cost <= 0.0) || (min_lor.metric_cost > DBL_MAX/2)){
		delete[] end_state;
		return -1;
	}

	//ROS_INFO("Extendto: BestTurningRais: %f, dist: %f, risk: %f", best_turning_radius, min_lor.metric_cost, min_lor.risk);

	while (end_state[2] < -M_PI)
		end_state[2] += 2.0 * M_PI;
	while (end_state[2] > M_PI)
		end_state[2] -= 2.0 * M_PI;

	for (int i = 0; i < 7; i++) {
		trajectoryOut.endState.x[i] = end_state[i];
	}

	trajectoryOut.lor.metric_cost = min_lor.metric_cost;
	trajectoryOut.lor.risk = min_lor.risk;
	trajectoryOut.dubinsRadius = best_turning_radius;

	delete [] end_state;
	return 1;
}

Level_OF_Risk CCSystem::evaluateExtension (CCState &stateFromIn, CCState &stateTowardsIn, bool &exactConnectionOut, bool check_risk){

	double *end_state = new double[7];

	Level_OF_Risk min_lor;
	min_lor.metric_cost = DBL_MAX;
	min_lor.risk = DBL_MAX;

	for(int i=num_turning_radii -1; i >= 0; i--){
		double turning_radius = turning_radii[i];
		bool tmp_exact_connection = false;
		vector<double> temp_risk, risk;
		list<float> tmp_control;

		Level_OF_Risk temp_lor;
		temp_lor.metric_cost = DBL_MAX;
		temp_lor.risk = DBL_MAX;

		double time = extend_dubins_all (
				stateFromIn.x, stateTowardsIn.x,
				false, false, false, temp_risk,
				tmp_exact_connection, end_state, NULL, tmp_control, turning_radius);

		temp_risk.clear();

		if(time > 0.0 && time < DBL_MAX/2){
			if (check_risk){
				double risk_time = extend_dubins_all (
						stateFromIn.x, stateTowardsIn.x,
						true, false, true, risk,
						tmp_exact_connection, end_state, NULL, tmp_control, turning_radius);

				if (risk_time > 0.0 && risk_time < DBL_MAX/2){
					double max_risk = 0;
					for (int i = 0 ; i < risk.size() ; i ++){
						if (risk[i] > max_risk)
							max_risk = risk[i];
					}
					temp_lor.metric_cost = risk_time;
					temp_lor.risk = max_risk;

					if (temp_lor < min_lor){
						min_lor = temp_lor;
						exactConnectionOut = tmp_exact_connection;
					}
				}
			}
		}
	}
	delete[] end_state;
	return min_lor;
}

int CCSystem::getTrajectory (
		CCState& stateFromIn, CCState& stateToIn,
		list<double*>& trajectoryOut, list<float>& controlOut, double turning_radius) {

	//ROS_INFO("Get Trajectory: Turning Radius: %f", turning_radius);

	list<double*> tmp_traj;
	list<float> tmp_control;
	bool tmp_exact_connection = false;
	vector<double> temp_risk;
	double *end_state = new double[7];

	double time = extend_dubins_all (
			stateFromIn.x, stateToIn.x,
			true, true, true, temp_risk,
			tmp_exact_connection, end_state, &tmp_traj, tmp_control, turning_radius);
	if(time > 0.0 && time < DBL_MAX/2){
		trajectoryOut = tmp_traj;
		controlOut = tmp_control;
		delete [] end_state;
		trajectoryOut.reverse();
		return 1;
	}
	else{
		//ROS_INFO("Traj can not be found");
		return -1;
	}
}

double CCSystem::evaluateCostToGo (CCState& stateIn){
	double size_x = regionGoal.size[0];
	double size_y = regionGoal.size[1];
	double radius = sqrt(size_x*size_x + size_y*size_y);

	double dist_x = stateIn.x[0] - regionGoal.center[0];
	double dist_y = stateIn.x[1] - regionGoal.center[1];
	double dist = sqrt (dist_x*dist_x + dist_y*dist_y);

	return dist - radius;
}

int CCSystem::clear_tmp_trajectories(list<double*> &state_traj, list<float> &control_traj){
	for(list<double*>::iterator i=state_traj.begin(); i!= state_traj.end(); i++){
		delete[] (*i);
	}
	state_traj.clear();
	control_traj.clear();
	return 0;
}

int CCSystem::sampleState(CCState &randomStateOut){
	int r = (double)(rand()/(RAND_MAX+1.0))*free_cells.size();

	int index = free_cells[r];
	double local_xy[3] ={0, 0, 0};
	getxy_from_index(local_xy[0], local_xy[1], index);

	for(int i=0; i<3; i++){
		randomStateOut.x[i] = local_xy[i] + (double)rand()/(RAND_MAX + 1.0)*regionCell.size[i] ;
	}

	// transform the sample from /base_link frame to /map frame
	double cyaw = cos(-map_origin[2]);
	double syaw = sin(-map_origin[2]);
	double state_copy[3];
	for(int i=0; i<3; i++)
		state_copy[i] = randomStateOut[i];

	randomStateOut.x[0] = map_origin[0] + state_copy[0]*cyaw + state_copy[1]*syaw;
	randomStateOut.x[1] = map_origin[1] + -state_copy[0]*syaw + state_copy[1]*cyaw;
	randomStateOut.x[2] = map_origin[2] - M_PI/2 + state_copy[2];
	while(randomStateOut.x[2] > M_PI)
		randomStateOut.x[2] -= 2.0*M_PI;
	while(randomStateOut.x[2] < -M_PI)
		randomStateOut.x[2] += 2.0*M_PI;

	randomStateOut.x[3] =0;
	randomStateOut.x[4] =0;
	randomStateOut.x[5] =0;
	randomStateOut.x[6] =0;

	if (IsInCollisionLazy (randomStateOut.x))
		return 0;

	//ROS_INFO("Sampling: x: %f, y: %f, t: %f", randomStateOut[0], randomStateOut[1], randomStateOut[2]);

	return 1;
}

int CCSystem::sampleGoalState (CCState &randomStateOut) {

	for (int i = 0; i < 3; i++)
		randomStateOut.x[i] = (double)rand()/(RAND_MAX + 1.0)*regionGoal.size[i]- regionGoal.size[i]/2.0 + regionGoal.center[i];

	while(randomStateOut.x[2] > M_PI)
		randomStateOut.x[2] -= 2.0*M_PI;
	while(randomStateOut.x[2] < -M_PI)
		randomStateOut.x[2] += 2.0*M_PI;

	for (int i = 3; i < 7 ; i++)
		randomStateOut.x[i] =0;

	if (IsInCollisionLazy (randomStateOut.x))
		return 0;
	return 1;
}

double CCSystem::norm_state(double s1[7], double s2[7]){
	double t = 0;
	for(int i=0; i<3; i++)
		t = t + (s1[i]-s2[i])*(s1[i]-s2[i]);
	return sqrt(t);
}
