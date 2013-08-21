/*
 * System.h
 *
 *  Created on: Jul 24, 2013
 *      Author: liuwlz
 */

#ifndef CC_SYSTEM_H_
#define CC_SYSTEM_H_

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
		/*
		if ( (this -> risk) < lor.risk + 0.1)
			if ( (this -> metric_cost) < lor.metric_cost )
				return true;
			else
				return false;
				*/
		if ( (this -> metric_cost) < lor.metric_cost )
			return true;
		else
			return false;
	}
};

class CCState
{
public:
	double x[7];
    CCState ();
    ~CCState ();
    CCState (const CCState &stateIn);
    CCState& operator= (const CCState &stateIn);
    double& operator[] (const int i) {return x[i];}

    friend class CCSystem;
    friend class CCTrajectory;
};

class CCTrajectory{

	CCState endState;
	Level_OF_Risk lor;

public:

	CCTrajectory ();
	~CCTrajectory ();
	CCTrajectory (const CCTrajectory &trajectoryIn);
	CCTrajectory& operator= (const CCTrajectory &trajectoryIn);

	int getEndState (CCState &endStateOut);
	CCState& getEndState () {return (CCState&)endState;}
	CCState& getEndState () const {return (CCState&)endState;}
	double evaluateCost ();
	double evaluateRisk();
	friend class CCSystem;
};

class CCSystem {

	RiskEvaluate *risk_evaluate;
	double distance_limit;
	double delta_distance;
	CCState rootState;

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

    CCSystem();
    ~CCSystem();

    double extend_dubins_spheres(
    		double x_s1, double y_s1, double t_s1,
    		double x_s2, double y_s2, double t_s2,
    		int combination,
    		bool check_obstacles, bool return_control, bool check_risk,vector<double>& obst_risk,
    		bool& fully_extends, double* &end_state, double* &initConv,
    		list<double*>* trajectory, list<float>& control, double turning_radius);
    double extend_dubins_all (
    		double state_ini[3], double state_fin[3],
    		bool check_obstacles, bool return_trajectory,bool check_risk,vector<double>& obst_risk,
    		bool& fully_extends, double*& end_state,
    		list<double*>* trajectory, list<float> &control, double turning_radius);

    int extendTo (
    		CCState &stateFromIn, CCState &stateTowardsIn,
    		bool check_obstacles, bool check_risk,
    		CCTrajectory &trajectoryOut, bool &exactConnectionOut, list<float> &controlOut);

    Level_OF_Risk evaluateExtension(
    		CCState &stateFromIn, CCState &stateTowardsIn,
    		bool &exactConnectionOut, bool check_risk);

    int getTrajectory (
    		CCState& stateFromIn, CCState& stateToIn,
    		list<double*>& trajectoryOut, list<float>& controlOut, bool check_obstacles, bool check_risk);

    int setRiskLimit(double risk);
    int getNumDimensions () {return 3;}
    CCState& getRootState () {return rootState;}
    int propagatePoseDubins(double stateIn[7], double (&stateOut)[7], double turning_radius, double turning_angle, double direction);
    int evaluateRisk(double stateIn[7], vector<double>& risk);
    int getStateKey (CCState &stateIn, double *stateKey);
    float getGoalCost(const double x[7]);
    bool isReachingTarget (CCState &stateIn);
    int transform_map_to_local_map(const double stateIn[7], double &zlx, double &zly, double &yl);
    int get_cell_index(double x, double y, int &map_index);
    int getxy_from_index(double &x, double &y, const int index);
    bool IsInCollision (const double stateIn[7]);
    bool IsInCollisionLazy (const double stateIn[7]);
    double getLaneCost(const double zx, const double zy);
    double getStateCost(const double stateIn[7]);
    int clear_tmp_trajectories(list<double*> &state_traj, list<float> &control_traj);
    double evaluateCostToGo (CCState& stateIn);
    int sampleState(CCState &randomStateOut);
    int sampleGoalState (CCState &randomStateOut);
    double norm_state(double s1[7], double s2[7]);
};

typedef struct{
  typedef CCState CCStateType;
  typedef CCTrajectory CCTrajectoryType;
  typedef CCSystem CCSystemType;
}CCDUBINS;

#endif /* CC_SYSTEM_H_ */
