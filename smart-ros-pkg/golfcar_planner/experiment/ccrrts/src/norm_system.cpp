/*
 * norm_system.cpp
 *
 *  Created on: Aug 17, 2013
 *      Author: liuwlz
 *
 *      Modify it to 2D environment
 */
#include "norm_system.h"

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
	lor.metric_cost = DBL_MAX;
	lor.risk = DBL_MAX;
}

NormTrajectory::~NormTrajectory () {

}

NormTrajectory::NormTrajectory (const NormTrajectory &trajectoryIn){
	endState = trajectoryIn.getEndState();
	lor.risk = trajectoryIn.lor.risk;
	lor.metric_cost = trajectoryIn.lor.metric_cost;
}

NormTrajectory& NormTrajectory::operator=(const NormTrajectory &trajectoryIn) {

	if (this == &trajectoryIn)
		return *this;

	endState = trajectoryIn.getEndState();

	lor.risk = trajectoryIn.lor.risk;
	lor.metric_cost = trajectoryIn.lor.metric_cost;

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
	distance_limit = 1.00;
	delta_distance = 0.05;

	risk_limit = 0.05;
	risk_evaluate = new RiskEvaluate();
	max_free_dist = 0;

	for (int i = 0; i < dist_map.info.height; i++){
		for (int j = 0; j < dist_map.info.width; j++){
			double map_index = j + i*dist_map.info.width;
			if (max_free_dist < sqrt(dist_map.data[map_index]))
				max_free_dist = sqrt(dist_map.data[map_index]);
		}
	}
}

NormSystem::~NormSystem () {

}

int NormSystem::setRiskLimit(double risk){
	risk_limit = risk;
	return 1;
}

#define SQ(x)   ((x)*(x))

int NormSystem::propagatePose(double stateIn[5], double (&stateOut)[5]){

	double x_conv = stateIn[2];
	double y_conv = stateIn[3];
	double xy_conv = stateIn[4];

	double increment = sqrt((stateIn[0] - stateOut[0])*(stateIn[0] - stateOut[0]) + (stateIn[1] - stateOut[1])*(stateIn[1] - stateOut[1]));
	//ROS_INFO("Increment: %f", increment);
	double theta = atan2((stateOut[1]-stateIn[1]), (stateOut[0]-stateIn[0]));

	double temp_V_1 = cos(theta)*increment;
	double temp_V_2 = sin(theta)*increment;

	Matrix2d A,  P_In, P_Out;
	double M = 2;
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
		cout << "V_1:" <<temp_V_1 << " V_2: " << temp_V_2 <<endl;
		cout << "x_conv_out: "<< stateOut [3] <<" y_conv_out: " << stateOut [4] <<"t_conv_out: "<< stateOut [5] <<endl;
	}
	return 1;
}

int NormSystem::linearConvPropagate(double stateIn[5], double (&stateOut)[5]){

	double increment =sqrt(SQ(stateIn[0] - stateOut[0]) + SQ(stateIn[1] - stateOut[1]));
	double theta = atan2((stateOut[1]-stateIn[1]), (stateOut[0]-stateIn[0]));

	double temp_V_1 = cos(theta)*increment;
	double temp_V_2 = sin(theta)*increment;

	Matrix2d A, Q,  LB_In, LB_Out, LC_In, LC_Out;
	MatrixXd V(2,1);
	double M = 2;

	V << temp_V_1 ,temp_V_2;
	A << 1, 0, 0 ,1;

	/*
	 * Details refer to "The belief Roadmap: Efficient Planning in Belief Space by Factoring the Convariance"
	 *
	 * Factories the convariance as P = B/C for efficient belief update.
	 *
	 * The implement here set the C as I = [1, 0 , 0, 1], thus the result is the same as the function "propagatePose"
	 *
	 * Further work will change the state's data structure to store the value of B and C, and the update can be more efficient
	 */

	LB_In << stateIn[2], stateIn[4], stateIn[4], stateIn[3];
	LC_In << 1, 0, 0, 1;

	Q = V * M * V.transpose();

	LB_Out = A*LB_In + Q * (A.transpose()).inverse();
	LC_Out = (A.transpose()).inverse()*LC_In;

	return 1;
}

/*
int NormSystem::propagatePose(double stateIn[5], double (&stateOut)[5], double theta, double increment){

	double x_conv = stateIn[2];
	double y_conv = stateIn[3];
	double xy_conv = stateIn[4];

	double temp_A_1 = -increment * sin(theta);
	double temp_A_2 = increment * cos(theta);

	double temp_V_1 = cos(theta);
	double temp_V_2 = sin(theta);

	Matrix2d A,  P_In, P_Out;
	double M = 0.04;
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
	}
	return 1;
}
*/

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

	transform_map_to_local_map(stateIn.x, stateKey[0], stateKey[1]);
	//ROS_INFO("GetStateKey: orig_x: %f, orig_y: %f, x: %f, Y: %f", stateIn[0], stateIn[1], stateKey[0], stateKey[1]);
	return 1;
}

#define SQ(x)   ((x)*(x))
float NormSystem::getGoalCost(const double x[5]){
	return (sqrt(SQ(x[0]-regionGoal.center[0]) + SQ(x[1]-regionGoal.center[1])));
}

bool NormSystem::isReachingTarget (NormState &stateIn) {
	for (int i = 0; i < 2; i++) {
		if (fabs(stateIn.x[i] - regionGoal.center[i]) > regionGoal.size[i]/4.0 )
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

//TODO: Fix the extend_line function to be more general and well organized
double NormSystem::extend_line (
		double state_ini[5], double state_fin[5], vector<double>& obst_risk, double*& end_state) {

	double x_start = state_ini[0];
	double y_start = state_ini[1];
	double x_end = state_fin[0];
	double y_end = state_fin[1];

	//double theta = atan2((y_end-y_start), (x_end-x_start));

	double line_distance;
	line_distance = sqrt ((x_start-x_end)*(x_start-x_end) + (y_start-y_end)*(y_start-y_end) );

	//ROS_INFO("ExtendLine: line_dist: %f;  StateIn: x: %f, y : %f. StateOut: x: %f, y : %f", line_distance, x_start, y_start, x_end, y_end);

	// Generate states/inputs
	double state_curr[5] = {0.0};
	double state_in[5] = {0.0};
	double state_out[5] = {0.0};

	vector<double> temp_risk;

	for (int i =0; i < 5; i++)
		state_in[i] = state_ini[i];

	double d_inc_curr = 0.0;
	double d_inc_old = 0.0;
	double extend_inc = 0.0;
	bool should_stop = false;
	bool is_init_state = true;

	while (d_inc_curr < line_distance+0.01){
		d_inc_curr += delta_distance;

		if (d_inc_curr > line_distance){
			d_inc_curr = line_distance;
			should_stop = true;
		}

		if (d_inc_curr > distance_limit){
			d_inc_curr = distance_limit;
			should_stop = true;
		}

		//extend_inc = d_inc_curr- d_inc_old;

		state_curr[0] = (x_end - x_start) * d_inc_curr / line_distance + x_start;
		state_curr[1] = (y_end - y_start) * d_inc_curr / line_distance + y_start;

		if (is_init_state){
			double temp_state[5];
			convTruncation(state_in, temp_state);
			for (int i = 0 ; i < 5; i++)
				state_in[i] = temp_state[i];
			is_init_state = false;
		}
		/*
		else
			convTruncation(state_out, state_in);
		*/
		/*
		vector<double> debug_risk;
		evaluateRisk(state_in, debug_risk);
		if (debug_risk.size() >1)
			ROS_INFO("Debug convTrunct: Risk_1 : %f, Risk_2: %f", debug_risk[0], debug_risk[1]);
		*/

		state_out[0] = state_curr[0];
		state_out[1] = state_curr[1];

		propagatePose(state_in, state_out);

		if (IsInCollisionLazy (state_out)){
			return -2.0;
		}

		vector<double> risk;
		evaluateRisk(state_out, risk);

		if (temp_risk.size() == 0){
			for (int i = 0 ; i < risk.size(); i++){
				temp_risk.push_back(risk[i]);
			}
		}

		for (int i =0 ; i < risk.size(); i++){
			if (risk[i] > risk_limit){

				/*
				for (int i = 0; i < temp_risk.size(); i++){
					obst_risk.push_back(temp_risk[i]);
				}
				return d_inc_old;
				 */
				return -3.0;
			}
			if (temp_risk[i] < risk[i])
				temp_risk[i] = risk[i];
		}

		for (int i = 0; i < 5; i++){
			state_in[i] = state_out[i];
			end_state[i] = state_out[i];
		}
		//convTruncation(state_out, state_in);
		d_inc_old = d_inc_curr;

		if (should_stop){
			should_stop = false;
			break;
		}
	}

	for (int i = 0; i < temp_risk.size(); i++){
		obst_risk.push_back(temp_risk[i]);
	}

	double extend_dist = sqrt((end_state[0] - x_start)*(end_state[0] - x_start)+(end_state[1] - y_start)*(end_state[1] - y_start));

	//if (obst_risk.size() > 1)
		//ROS_INFO("ExtendLine: dist: %f; risk: %f, %f ; StateOut: x: %f, y: %f; extend_dist: %f", d_inc_curr, obst_risk[0], obst_risk[1], end_state[0], end_state[1], extend_dist);

	return extend_dist;
}

int NormSystem::getPiecewiseLine (double state_ini[5], double state_fin[5], list<double*>* trajectory, list<float> &control) {

	double x_start = state_ini[0];
	double y_start = state_ini[1];
	double x_end = state_fin[0];
	double y_end = state_fin[1];

	double theta = atan2((y_end-y_start), (x_end-x_start));

	double line_distance;
	line_distance = sqrt ((x_start-x_end)*(x_start-x_end) + (y_start-y_end)*(y_start-y_end) );

	//ROS_INFO("piecewise: line_dist: %f;  StateIn: x: %f, y : %f. StateOut: x: %f, y : %f", line_distance, x_start, y_start, x_end, y_end);

	double state_curr[5] = {0.0};

	double d_inc_curr = 0.0;

	while (d_inc_curr < line_distance){

		d_inc_curr += delta_distance;

		if (d_inc_curr > line_distance)
			d_inc_curr = line_distance;
		state_curr[0] = (x_end - x_start) * d_inc_curr / line_distance + x_start;
		state_curr[1] = (y_end - y_start) * d_inc_curr / line_distance + y_start;

		double *state_new = new double[5];
		for (int i = 0; i < 5; i++)
			state_new[i] = state_curr[i];
		if (trajectory)
			trajectory->push_front(state_new);
		control.push_front (0.0);
	}
	return 1;
}

int NormSystem::extendTo (
		NormState &stateFromIn, NormState &stateTowardsIn,
		bool check_obstacles, bool check_risk,
		NormTrajectory &trajectoryOut, bool &exactConnectionOut, list<float> &controlOut) {

	double *tmp_end_state = new double [5];
	bool tmp_exact_connection = true;
	list<float> tmp_control;
	vector<double> tmp_risk;

	Level_OF_Risk temp_lor;
	temp_lor.metric_cost = DBL_MAX;
	temp_lor.risk = DBL_MAX;

	//ROS_INFO("ExtendTO: startIn: x %f, y: %f", stateFromIn.x[0], stateFromIn.x[1]);

	double time_cost = extend_line (stateFromIn.x, stateTowardsIn.x, tmp_risk, tmp_end_state);

	if (time_cost > 0 && time_cost < DBL_MAX/2){
		double max_risk = 0;
		for (int i = 0 ; i < tmp_risk.size() ; i ++){
			if (tmp_risk[i] > max_risk)
				max_risk = tmp_risk[i];
		}
		temp_lor.metric_cost = time_cost;
		temp_lor.risk = max_risk;
	}

	exactConnectionOut = tmp_exact_connection;
	controlOut = tmp_control;

	for (int i = 0; i < 5; i++) {
		trajectoryOut.endState.x[i] = tmp_end_state[i];
	}
	trajectoryOut.lor.metric_cost = temp_lor.metric_cost;
	trajectoryOut.lor.risk = temp_lor.risk;

	//ROS_INFO("ExtendTO: length: %f, risk: %f, end_state: x: %f, y: %f", temp_lor.metric_cost, temp_lor.risk, tmp_end_state[0], tmp_end_state[1]);

	if((temp_lor.metric_cost < 0.0) || (temp_lor.metric_cost > DBL_MAX/2)){
		delete[] tmp_end_state;
		return -1;
	}

	delete [] tmp_end_state;
	return 1;
}

int NormSystem::getTrajectory (
		NormState& stateFromIn, NormState& stateToIn,
		list<double*>& trajectoryOut, list<float>& controlOut,
		bool check_obstacles, bool check_risk) {

	//ROS_INFO("getTrajectory");
	bool exactConnectionOut = false;
	list<double*> tmp_traj;
	list<float> tmp_control;
	bool tmp_exact_connection = false;
	vector<double> temp_risk;

	Level_OF_Risk temp_lor;
	temp_lor.metric_cost = DBL_MAX;
	temp_lor.risk = DBL_MAX;

	int length = getPiecewiseLine(stateFromIn.x, stateToIn.x, &tmp_traj, tmp_control);

	trajectoryOut = tmp_traj;
	controlOut = tmp_control;
	exactConnectionOut = tmp_exact_connection;

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
#if(0)
	boost::uniform_real<> real_dist_x(10.0, 20.0);
	boost::uniform_real<> real_dist_y(30.0, 50.0);
#else
	//boost::uniform_real<> real_dist_x(10.0, 25.0);
	//boost::uniform_real<> real_dist_y(13.0, 22.0);

	boost::uniform_real<> real_dist_x(10.0, 25.0);
	boost::uniform_real<> real_dist_y(60.0, 70.0);
#endif
	boost::variate_generator<boost::mt19937&, boost::uniform_real<> > real_uniform_x(gen, real_dist_x);
	boost::variate_generator<boost::mt19937&, boost::uniform_real<> > real_uniform_y(gen, real_dist_y);
	randomStateOut.x[0] = real_uniform_x();
	randomStateOut.x[1] = real_uniform_y();
	randomStateOut.x[2] =0;
	randomStateOut.x[3] =0;
	randomStateOut.x[4] =0;

	if (IsInCollisionLazy (randomStateOut.x))
		return 0;

	//ROS_INFO("Sampling: x: %f, y: %f", randomStateOut[0], randomStateOut[1]);

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

double NormSystem::dynamicRiskLimit(double stateIn[5]){
	// (x,y) in local_map frame
	double zl[2] = {0};
	double risk;
	transform_map_to_local_map(stateIn, zl[0], zl[1]);
	//cout<<"zl: "<< zl[0]<<" "<<zl[1]<<" "<<yl<<endl;
	int map_index = -1;
	if(get_cell_index(zl[0],zl[1], map_index) == 0){
		double log_dist = 1+0.05/log(sqrt(dist_map.data[map_index])/max_free_dist);
		risk = 0.5 + (1-risk_limit-0.5)*log_dist;
		if (risk < 0.5)
			risk = 0.5;
	}
	else
		risk = 1-risk_limit;
	return (1-risk);
}

/*
 * Refer to paper "Estimating probability of collision for safe motion planning under
 *  Gaussian motion and sensing uncertaimty" for more details
 */

int NormSystem::convTruncation(double stateIn[5], double (&stateOut)[5]){
	ccrrts::obsts_cst cst;
	bool need_update = false;
	double min_risk = 1.0;
	double state_inc[5];
	boost::math::normal_distribution<double> norm_dist;
	cst = risk_evaluate->constraints;
	if (cst.obsts_cst.size() != 0){
		for (vector<ccrrts::constraint>::iterator it = cst.obsts_cst.begin(); it != cst.obsts_cst.end(); it++){
			for (int i = 0; i < it->serial_no.size(); i++){
				double a_1 = it->a_1[i];
				double a_2 = it->a_2[i];
				double b = it->b[i];

				MatrixXd A(2,1), X(2,1), D_X(2,1), B(1,1), CONV(1,1), MEAN(1,1), MEAN_PRO(1,1), CONV_PRO(1,1);
				Matrix2d R, D_R;
				A << a_1, a_2;
				B << b;
				X << stateIn[0],stateIn[1];
				R << stateIn[2], stateIn[4], stateIn[4], stateIn[3];

				MEAN = A.transpose()*X;
				CONV = A.transpose()*R*A;

				double conv_orig = CONV(0,0);
				double mean_orig = MEAN(0,0);
				double risk_orig = (1 - boost::math::erf((mean_orig-b)/sqrt(2*conv_orig)))/2.0;
				//ROS_INFO("Risk_orig: %f", risk_orig);
				//if (risk_orig < min_risk && risk_orig < risk_limit && risk_orig > 0.002){
				//if (risk_orig < min_risk && risk_orig < 0){

				if (risk_orig < min_risk && risk_orig < 0.05 && risk_orig > 0.002){
					double alpha = (b-mean_orig) /sqrt(conv_orig);

					double pdf = boost::math::pdf(norm_dist, alpha);
					double cdf = boost::math::cdf(norm_dist, alpha);
					double lamda = pdf/(1-cdf);

					double mean_proc = mean_orig + lamda*sqrt(conv_orig);
					double conv_proc = conv_orig*(1-lamda*lamda + alpha*lamda);

					MEAN_PRO << mean_proc;
					CONV_PRO << conv_proc;

					D_X = (R*A)/CONV(0,0)*(MEAN(0,0)-MEAN_PRO(0,0));
					D_R = (R*A)/CONV(0,0)*(CONV(0,0)-CONV_PRO(0,0))*((A.transpose()*R)/CONV(0,0));

					state_inc[0] = D_X(0,0);
					state_inc[1] = D_X(1,0);
					state_inc[2] = D_R(0,0);
					state_inc[3] = D_R(1,1);
					state_inc[4] = D_R(0,1);

					min_risk = risk_orig;
					need_update = true;
					double risk_proc = (1 - boost::math::erf((mean_proc-b)/sqrt(2*conv_proc)))/2.0;

					if (false){
						ROS_INFO("StateIn x: %f, y: %f x_conv: %f, y_conv: %f, xy_conv: %f",
								stateIn[0], stateIn[1], stateIn[2], stateIn[3], stateIn[4]);
						ROS_INFO("Min_risk: %f", min_risk);
						ROS_INFO("Truncate Debug: alpha: %f, pdf: %f, cdf: %f, lamda: %f, conv factor: %f", alpha, pdf, cdf, lamda, (1-lamda*lamda + alpha*lamda));
						ROS_INFO("Mean_orig: %f, Conv_orig: %f, Risk_orig, %f", (mean_orig-b), conv_orig ,risk_orig);
						ROS_INFO("Mean_proc: %f, Conv_proc: %f, Risk_proc, %f", (mean_proc-b), conv_proc ,risk_proc);
					}
				}
			}
			if (need_update){
				min_risk = 1.0;
				for(int i = 0 ;i < 5; i++)
					stateOut[i] = stateIn[i] - state_inc[i];
				need_update = false;
			}
			else{
				for(int i = 0; i < 5; i++)
					stateOut[i] = stateIn[i];
			}
		}
		//ROS_INFO("Debug for ConvTruncation now: x: %f, y: %f, x_cov: %f, y_cov: %f, xy_cov: %f", state_inc[0], state_inc[1], state_inc[2], state_inc[3], state_inc[4]);
	}
	return 1;
}
