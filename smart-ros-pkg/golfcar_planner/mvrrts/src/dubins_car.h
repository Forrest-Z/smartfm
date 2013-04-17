#ifndef __RRTS_SYSTEM_H_
#define __RRTS_SYSTEM_H_

#include <float.h>
#include <vector>
#include <list>
#include <stdlib.h>
#include <string>

#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pnc_msgs/local_map.h>

#include "automata.h"

using namespace std;

/** Rectangular region of the configuration space. */
struct Region {

  // name of region, used for drawing
  string name;
  /** Subset of atomic propositions associated with the region. */
  Subset_of_Sigma     label;
  /** True if region has an associated travel direction */
  bool                hasDir;
  /** Size of the region. */
  double              size[3];
  /** Center of the region. */
  double              center[3];
  /** Initializes size to zero, center to the origin, 
   * and label to the empty set. */
  Region() : 
    // Initialize subset of atomic props to the empty set
    label(), hasDir(false){
      name = "anon";
      size[0] = 0.0;      
      size[1] = 0.0;      
      size[2] = 2.0 * M_PI;
      center[0] = 0.0;    
      center[1] = 0.0;    
      center[2] = 0.0;
    }
  
  ~Region() {}
  /** Sets the size of the region to the given dimensions. 
   * @param sx    Size in the x-dimension. 
   * @param sy    Size in the y-dimension. */
  void set_size( const double sx, const double sy, const double sz=0) {
    // Copy size
    size[0] = sx;       
    size[1] = sy;
    //size[2] = sz;
  }
  
  /** Sets the center of the region to the given point. 
   * @param cx    Center in the x-dimension. 
   * @param cy    Center in the y-dimension. */
  void set_center( const double cx, const double cy, const double cz=0) {
    // Copy size
    center[0] = cx;     
    center[1] = cy;
    //center[2] = cz;
  }
  
  /** Sets the center and size of the region 
   * with respect to the path angle (theta). 
   * @param ct        Center angle. 
   * @param st        Angular width of region. */
  void set_direction( const double ct, const double st) {
    hasDir = true;
    center[2] = ct;     
    size[2] = st;
  }
};

/* State of the robot in the configuration space */
struct State {
  double x[3];
  State() { x[2] = x[1] = x[0] = 0.0; }
  /* Copy constructor */
  State( const State& stateIn) {
    x[0] = stateIn.x[0];
    x[1] = stateIn.x[1];
    x[2] = stateIn.x[2];
  }
  ~State() {}
  State& operator=( const State& stateIn) {
    if ( this != &stateIn ) 
    {
      x[0] = stateIn.x[0];
      x[1] = stateIn.x[1];
      x[2] = stateIn.x[2];
    }
    return *this;
  };
  /* Returns the i^th coordinate of the state */
  double& operator[] ( const UCH i) { return x[i]; }
  /* Sets coordinates to given ones */
  void set_to( const double x_s, const double y_s, const double t_s) 
  {
    x[0] = x_s; x[1] = y_s; x[2] = t_s;
  }
  void print()
  {
    for(int i=0; i<3; i++)
      std::cout<<x[i]<<" ";
    std::cout<<endl;
  }
  
  friend class Trajectory;
  friend class System;
};

/* Trajectory of the robot */
struct Trajectory {
  State               endState;
  double              totalVariation;
  
  Trajectory() : endState(), totalVariation(-1.0) {}
  Trajectory( const Trajectory &trajectoryIn) : 
    endState(trajectoryIn.endState), 
    totalVariation(trajectoryIn.totalVariation) {}
  
  ~Trajectory() {}
  Trajectory& operator= ( const Trajectory& trajectoryIn) 
  {
    // if pointer is same, return the same data
    if ( this == &trajectoryIn ) 
      return *this;
    // otherwise, deep copy
    endState        = trajectoryIn.getEndState();
    totalVariation  = trajectoryIn.totalVariation;
    return *this;
  }
  int getEndState( State &endStateOut) 
  {
    endStateOut = endState; 
    return 1;
  }
  State& getEndState() const 
  { 
    return ( (State&) endState ); 
  }
  double evaluateCost() 
  { 
    return totalVariation; 
  }
  friend class System;
};

/* Curvature-constrained constant-speed robot */
class System {
  public:

    State       rootState;
    // discretization for obstacle checking
    double      delta_distance;
    double      distance_limit;
    double extend_dubins_spheres( 
        double x_s1, double y_s1, double t_s1, 
        double x_s2, double y_s2, double t_s2, 
        int combination, bool check_obstacles, bool return_control, 
        bool& fully_extends, 
        double* &end_state, 
        list<double*>* trajectory, 
        list<float>& control);
    double extend_dubins_all( 
        double state_ini[3], double state_fin[3], 
        bool check_obstacles, bool return_control, 
        bool& fully_extends, 
        double*& end_state, 
        list<double*>* trajectory, 
        list<float>& control);

    double              turning_radius;

    Region              regionOperating;
    Region              regionGoal;
    Region              regionCell;
    list< Region* >     labeled_reg;
    list< Region* >     obstacles;
    vector<Region*> sampling_regions;

#define num_turning_radii   (3)
    double turning_radii[3];
    double car_width;
    double car_height;
    double safe_distance;
    double distance_rear_axis_rear;

    nav_msgs::OccupancyGrid map;
    vector<int> free_cells;
    double map_origin[3];

    System();
    ~System();

    int getNumDimensions() { return 3; }
    State& getRootState() { return rootState; }
    int getStateKey( State& stateIn, double* stateKey);

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

    /* Returns subset of atomic propositions 
     * corresponding to the given state */
    Subset_of_Sigma label_state( double stateIn[3]);
};

typedef struct {

  typedef State             StateType;
  typedef Trajectory        TrajectoryType;
  typedef System            SystemType;

} DubinsCar;

#endif
