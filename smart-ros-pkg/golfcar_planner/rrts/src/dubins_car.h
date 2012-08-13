#ifndef __RRTS_SYSTEM_H_
#define __RRTS_SYSTEM_H_

#include <float.h>
#include <vector>
#include <list>
#include <stdlib.h>
#include <geometry_msgs/Point.h>

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
    double x[3];

    public:
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
            bool& fully_extends, double*& end_state, list<double*>* trajectory, list<float> &control);

    double extend_dubins_all (double state_ini[3], double state_fin[3], 
            bool check_obstacles, bool return_trajectory,
            bool& fully_extends, double*& end_state, list<double*>* trajectory, list<float> &control);


    State rootState;

    public:    

    double turning_radius;
    region regionOperating;
    region regionGoal;

    // obstacles
    list<region*> obstacles;

    int has_found_path;
    geometry_msgs::Point origin;
    int xsize, ysize, xorigin, yorigin;
    float map_width, map_height, map_res;
    unsigned char *map_vals;

    System ();
    ~System ();

    int getNumDimensions () {return 3;}    
    State& getRootState () {return rootState;}
    int getStateKey (State& stateIn, double* stateKey);

    bool IsInCollision (double stateIn[3]);
    int getStateCost(double stateIn[3]);

    bool isReachingTarget (State& stateIn);

    int sampleState (State& randomStateOut); 
    int sampleGoalState (State& randomStateOut);

    int extendTo (State& stateFromIn, State& stateTowardsIn, 
            Trajectory& trajectoryOut, bool& exactConnectionOut, list<float> &controlOut ); 

    double evaluateExtensionCost (State& stateFromIn, State& stateTowardsIn, bool& exactConnectionOut);

    double evaluateCostToGo (State& stateIn);

    int getTrajectory (State& stateFromIn, State& stateToIn, list<double*>& trajectoryOut, list<float> &controlOut);

};



typedef struct _DubinsCar {

    typedef State StateType;
    typedef Trajectory TrajectoryType;
    typedef System SystemType;

} DubinsCar;

#endif
