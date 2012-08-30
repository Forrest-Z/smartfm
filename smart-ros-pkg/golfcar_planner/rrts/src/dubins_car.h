#ifndef __RRTS_SYSTEM_H_
#define __RRTS_SYSTEM_H_

#include <float.h>
#include <vector>
#include <list>
#include <stdlib.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;


class region 
{
    public:    
        double center[3];
        double size[3];

        region ();
        ~region ();
};



class State 
{
    public:
        double x[3];
        State ();
        ~State ();
        State (const State &stateIn);
        State& operator= (const State &stateIn);
        double& operator[] (const int i) {return x[i];}

    friend class System;
    friend class Trajectory;
};


class Trajectory {

    State endState; 
    double totalVariation;  

    public:    

    Trajectory ();
    ~Trajectory ();
    Trajectory (const Trajectory &trajectoryIn);
    Trajectory& operator= (const Trajectory &trajectoryIn);

    int getEndState (State &endStateOut);
    State& getEndState () {return (State&)endState;}
    State& getEndState () const {return (State&)endState;}
    double evaluateCost ();

    friend class System;
};


class System {

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

    public:    
    
    region regionOperating;
    region regionGoal;
    
#define num_turning_radii   (3)
    double turning_radii[3];
    double car_width;
    double car_height;
    double safe_distance;
    double distance_rear_axis_rear;

    nav_msgs::OccupancyGrid map;
    double map_origin[3];

    System ();
    ~System ();

    int getNumDimensions () {return 3;}    
    State& getRootState () {return rootState;}
    int getStateKey (State& stateIn, double* stateKey);

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
