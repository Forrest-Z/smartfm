#include <iostream>

#include "mvdubins.h"
#include "mvrrts.h"
using namespace std;
using namespace MVRRTstar;

typedef MVDubinsCar::SystemType system_t;
typedef MVDubinsCar::StateType state_t;

typedef MVVertex<MVDubinsCar> vertex_t;
typedef Planner<MVDubinsCar> planner_t;

int main()
{
  
  system_t system;
  mvregion* region = new mvregion();
  region->set_size(1,1,1);
  system.labeled_regions.push_back(region);
  state_t s;
  s.set_to(0,0,0);
  Subset_of_Sigma state_label = system.label_state(s.x);
  
  planner_t mvrrts;

  cout<<"Hello"<<endl;
  return 0;
}
