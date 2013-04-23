#ifndef __RRTS_SYSTEM_H_
#define __RRTS_SYSTEM_H_

#include <float.h>
#include <vector>
#include <list>
#include <stdlib.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pnc_msgs/local_map.h>

using namespace std;


class region 
{
  public:    
    double center[3];
    double size[3];

    region()
    {
      size[0] = size[1] = 0;
      size[2] = 2.0*M_PI;
      center[0] = center[1] = center[2] = 0;
    }
    void set_size( const double sx, const double sy, const double sz=0) 
    {
      size[0] = sx;
      size[1] = sy;
      size[2] = sz;
    }
    void set_center( const double cx, const double cy, const double cz=0) 
    {
      center[0] = cx;     
      center[1] = cy;
      center[2] = cz;
    }
};



class State 
{
  public:
    double x[3];
    State (){};
    ~State (){};
    State (const State &stateIn);
    State& operator= (const State &stateIn);
    double& operator[] (const int i) {return x[i];}

    void set_to(const double x_s, const double y_s, const double t_s) 
    {
      x[0] = x_s; 
      x[1] = y_s; 
      x[2] = t_s;
    }
    void print()
    {
      for(int i=0; i<3; i++)
        std::cout<<x[i]<<" ";
      std::cout<<endl;
    }
    //friend class System;
    //friend class Trajectory;
};


class Trajectory {

  public:    
    State endState; 
    double totalVariation;  


    Trajectory ();
    ~Trajectory ();
    Trajectory (const Trajectory &trajectoryIn);
    Trajectory& operator= (const Trajectory &trajectoryIn);

    int getEndState (State &endStateOut);
    State& getEndState () {return (State&)endState;}
    State& getEndState () const {return (State&)endState;}
    double evaluateCost ();

    //friend class System;
};


class System {

  public:    
    double distance_limit;

    double delta_distance;

    double extend_dubins_spheres (double x_s1, double y_s1, double t_s1, 
        double x_s2, double y_s2, double t_s2, int comb_no, 
        bool check_obstacles, bool return_trajectory,
        bool& fully_extends, double*& end_state, list<double*>* trajectory, list<float> &control, double turning_radius);

    double extend_dubins_all (double state_ini[3], double state_fin[3], 
        bool check_obstacles, bool return_trajectory,
        bool& fully_extends, double*& end_state, list<double*>* trajectory, list<float> &control, double turning_radius);


    State rootState;


    region regionOperating;
    region regionGoal;
    region regionCell;

#define num_turning_radii   (3)
    double turning_radii[3];
    double car_width;
    double car_height;
    double safe_distance;
    double distance_rear_axis_rear;

    nav_msgs::OccupancyGrid map;
    vector<int> free_cells;
    double map_origin[3];

    System ();
    ~System ();

    int getNumDimensions () {return 3;}    
    State& getRootState () {return rootState;}
    int getStateKey (State& stateIn, double* stateKey);
    double norm_state(double s1[3], double s2[3]);

    int getxy_from_index(double &x, double &y, const int index);
    int get_cell_index(double x, double y, int &map_index);
    int transform_map_to_local_map(const double stateIn[3], double &zlx, double &zly, double &yl);
    bool IsInCollision (const double stateIn[3], bool debug_flag=false);
    double getLaneCost(const double zx, const double zy);
    double getStateCost(const double stateIn[3]);

    float getGoalCost(const double x[3]);
    bool isReachingTarget (State& stateIn);

    int sampleState (State& randomStateOut); 
    int sampleGoalState (State& randomStateOut);

    int extendTo (State& stateFromIn, State& stateTowardsIn, 
        Trajectory& trajectoryOut, bool& exactConnectionOut, list<float> &controlOut, bool check_obstacles); 

    double evaluateExtensionCost (State& stateFromIn, State& stateTowardsIn, bool& exactConnectionOut);

    double evaluateCostToGo (State& stateIn);

    int clear_tmp_trajectories(list<double*> &state_traj, list<float> &control_traj);
    int getTrajectory (State& stateFromIn, State& stateToIn, list<double*>& trajectoryOut, list<float> &controlOut, bool check_obstacles);

};



typedef struct _DubinsCar {

  typedef State StateType;
  typedef Trajectory TrajectoryType;
  typedef System SystemType;

} DubinsCar;

#endif
