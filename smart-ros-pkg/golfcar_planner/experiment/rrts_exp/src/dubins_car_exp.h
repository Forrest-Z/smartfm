#ifndef __DUBINS_CAR_EXP_H_
#define __DUBINS_CAR_EXP_H_

#include <float.h>
#include <vector>
#include <list>
#include <stdlib.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pnc_msgs/local_map.h>

using namespace std;

class RegionExp
{
  public:
    double center[3];
    double size[3];

    RegionExp ();
    ~RegionExp ();
};

class StateExp
{
  public:
    double x[3];
    StateExp ();
    ~StateExp ();
    StateExp (const StateExp &stateIn);
    StateExp& operator= (const StateExp &stateIn);
    double& operator[] (const int i) {return x[i];}

    friend class SystemExp;
    friend class TrajectoryExp;
};

class TrajectoryExp {

  StateExp endState;
  double totalVariation;  

  public:    

  TrajectoryExp ();
  ~TrajectoryExp ();
  TrajectoryExp (const TrajectoryExp &trajectoryIn);
  TrajectoryExp& operator= (const TrajectoryExp &trajectoryIn);

  int getEndState (StateExp &endStateOut);
  StateExp& getEndState () {return (StateExp&)endState;}
  StateExp& getEndState () const {return (StateExp&)endState;}
  double evaluateCost ();

  friend class SystemExp;
};

class SystemExp {

  double distance_limit;

  double delta_distance;

  double extend_dubins_spheres (double x_s1, double y_s1, double t_s1, 
      double x_s2, double y_s2, double t_s2, int comb_no, 
      bool check_obstacles, bool return_trajectory,
      bool& fully_extends, double*& end_state, list<double*>* TrajectoryExp, list<float> &control, double turning_radius);

  double extend_dubins_all (double state_ini[3], double state_fin[3], 
      bool check_obstacles, bool return_trajectory,
      bool& fully_extends, double*& end_state, list<double*>* TrajectoryExp, list<float> &control, double turning_radius);

  StateExp rootState;

  public:    

  RegionExp regionOperating;
  RegionExp regionGoal;
  RegionExp regionCell;

#define num_turning_radii   (3)
  double turning_radii[3];
  double car_width;
  double car_height;
  double safe_distance;
  double distance_rear_axis_rear;

  nav_msgs::OccupancyGrid map;
  vector<int> free_cells;
  double map_origin[3];

  SystemExp ();
  ~SystemExp ();

  int getNumDimensions () {return 3;}    
  StateExp& getRootState () {return rootState;}
  int getStateKey (StateExp& stateIn, double* stateKey);

  int getxy_from_index(double &x, double &y, const int index);
  int get_cell_index(double x, double y, int &map_index);
  int transform_map_to_local_map(const double stateIn[3], double &zlx, double &zly, double &yl);
  bool IsInCollision (const double stateIn[3], bool debug_flag=false);
  double getLaneCost(const double zx, const double zy);
  double getStateCost(const double stateIn[3]);

  float getGoalCost(const double x[3]);
  bool isReachingTarget (StateExp& stateIn);

  int sampleState (StateExp& randomStateOut);
  int sampleGoalState (StateExp& randomStateOut);

  int extendTo (StateExp& stateFromIn, StateExp& stateTowardsIn,
      TrajectoryExp& trajectoryOut, bool& exactConnectionOut, list<float> &controlOut, bool check_obstacles);

  double evaluateExtensionCost (StateExp& stateFromIn, StateExp& stateTowardsIn, bool& exactConnectionOut);

  double evaluateCostToGo (StateExp& stateIn);

  int clear_tmp_trajectories(list<double*> &state_traj, list<float> &control_traj);
  int getTrajectory (StateExp& stateFromIn, StateExp& stateToIn, list<double*>& trajectoryOut, list<float> &controlOut, bool check_obstacles);

};

typedef struct _DubinsCarExp {

  typedef StateExp StateTypeExp;
  typedef TrajectoryExp TrajectoryTypeExp;
  typedef SystemExp SystemTypeExp;

} DubinsCarExp;

#endif
