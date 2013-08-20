/*
 * norm_system.h
 *
 *  Created on: Aug 17, 2013
 *      Author: liuwlz
 */

#ifndef NORM_SYSTEM_H_
#define NORM_SYSTEM_H_

#include <system_exp.h>
#include "risk_evaluate.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

struct Level_OF_Risk{
	double risk;
	double metric_cost;

	Level_OF_Risk(){
		metric_cost = 0;
		risk = 0;
	}

	Level_OF_Risk( const Level_OF_Risk& original){
		metric_cost =  original.metric_cost;
		risk = original.risk;
	}

	Level_OF_Risk& operator+=( const Level_OF_Risk& lor) {
	    // Update largest risk
		if (risk < lor.risk)
			risk = lor.risk;
	    // Increase metric cost
	    metric_cost += lor.metric_cost;
	    return *this;
	}

	Level_OF_Risk& operator=( const Level_OF_Risk& lor) {

		if (this == &lor)
			return *this;

		risk = lor.risk;
	    metric_cost = lor.metric_cost;
	    return *this;
	}

	const Level_OF_Risk operator+(const Level_OF_Risk& other) const{
		Level_OF_Risk toret = *this;
		toret += other;
		return toret;
	}

	bool operator< ( const Level_OF_Risk& lor) const {
		if ( (this -> metric_cost) < lor.metric_cost )
			return true;
		else
			return false;
	}
};

class NormState{

public:
	double x[5];
    NormState ();
    ~NormState ();
    NormState (const NormState &stateIn);
    NormState& operator= (const NormState &stateIn);
    double& operator[] (const int i) {return x[i];}

    friend class NormSystem;
    friend class NormTrajectory;
};

class NormTrajectory{

	NormState endState;
	Level_OF_Risk lor;

public:

	NormTrajectory ();
	~NormTrajectory ();
	NormTrajectory (const NormTrajectory &trajectoryIn);
	NormTrajectory& operator= (const NormTrajectory &trajectoryIn);

	int getEndState (NormState &endStateOut);
	NormState& getEndState () {return (NormState&)endState;}
	NormState& getEndState () const {return (NormState&)endState;}
	double evaluateCost ();
	double evaluateRisk();
	friend class NormSystem;
};

class NormSystem {

	RiskEvaluate *risk_evaluate;
	double distance_limit;
	double delta_distance;
	NormState rootState;

public:

	RegionExp regionOperating;
	RegionExp regionGoal;
 	RegionExp regionCell;

#define num_turning_radii (1)
 	double turning_radii[1];
 	double car_width;
 	double car_height;
 	double safe_distance;
 	double distance_rear_axis_rear;

 	nav_msgs::OccupancyGrid map;
 	vector<int> free_cells;
 	double map_origin[3];

 	double risk_limit;

    NormSystem();
    ~NormSystem();

    double extend_line (
    		double state_ini[3], double state_fin[3],
    		bool return_trajectory,bool check_risk,vector<double>& obst_risk,
    		bool& fully_extends, double*& end_state,
    		list<double*>* trajectory, list<float> &control);

    int extendTo (
    		NormState &stateFromIn, NormState &stateTowardsIn,
    		bool check_obstacles, bool check_risk,
    		NormTrajectory &trajectoryOut, bool &exactConnectionOut, list<float> &controlOut);

    Level_OF_Risk evaluateExtension(
    		NormState &stateFromIn, NormState &stateTowardsIn,
    		bool &exactConnectionOut, bool check_risk);

    int getTrajectory (
    		NormState& stateFromIn, NormState& stateToIn,
    		list<double*>& trajectoryOut, list<float>& controlOut, bool check_obstacles, bool check_risk);

    int setRiskLimit(double risk);
    int getNumDimensions () {return 2;}
    NormState& getRootState () {return rootState;}
    int propagatePose(double stateIn[5], double (&stateOut)[5], double theta, double increment);
    int evaluateRisk(double stateIn[5], vector<double>& risk);
    int getStateKey (NormState &stateIn, double *stateKey);
    float getGoalCost(const double x[5]);
    bool isReachingTarget (NormState &stateIn);
    int transform_map_to_local_map(const double stateIn[5], double &zlx, double &zly);
    int get_cell_index(double x, double y, int &map_index);
    int getxy_from_index(double &x, double &y, const int index);
    bool IsInCollision (const double stateIn[5]);
    bool IsInCollisionLazy (const double stateIn[5]);
    int clear_tmp_trajectories(list<double*> &state_traj, list<float> &control_traj);
    double evaluateCostToGo (NormState& stateIn);
    int sampleState(NormState &randomStateOut);
    int sampleGoalState (NormState &randomStateOut);
    double norm_state(double s1[5], double s2[5]);
};

typedef struct{
  typedef NormState NormStateType;
  typedef NormTrajectory NormTrajectoryType;
  typedef NormSystem NormSystemType;
}NormSYS;

#endif /* NORM_SYSTEM_H_ */
