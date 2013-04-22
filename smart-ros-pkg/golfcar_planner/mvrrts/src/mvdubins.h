#ifndef __MVDUBINS_H__
#define __MVDUBINS_H__

#include <dubins_car.h>
#include "automata.h"

class mvregion : public region
{
  public:
  string name;
  Subset_of_Sigma label;
  bool  has_dir;

  mvregion() : region(), name("anon"), label(), has_dir(false)
  {}
  void set_direction( const double ct, const double st) 
  {
    has_dir = true;
    center[2] = ct; 
    size[2] = st;
  }
};

class MVSystem : public System
{
  public:

    MVSystem() : System() {};
    ~MVSystem()
    {
      for(list<mvregion*>::iterator i=labeled_regions.begin(); i != labeled_regions.end(); i++)
        delete *i;
    }
    
    list<mvregion*> labeled_regions;
    Subset_of_Sigma label_state( double stateIn[3]);

    double extend_dubins_spheres( 
    double x_s1, double y_s1, double t_s1, 
    double x_s2, double y_s2, double t_s2, 
    int combination, bool check_obstacles, bool return_control, 
    bool& fully_extends, 
    double* &end_state, 
    list<double*>* trajectory, 
    list<float>& control, double turning_radius);
};

typedef struct{
  typedef State StateType;
  typedef Trajectory TrajectoryType;
  typedef MVSystem SystemType;
}MVDubinsCar;

#endif
