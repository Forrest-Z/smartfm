#include "dubins_car_exp.h"
#include <cmath>
#include <cstdlib>
#include <inttypes.h>
#include <iostream>
#include <stdio.h>

using namespace std;

RegionExp::RegionExp () {

}

RegionExp::~RegionExp () {

}

StateExp::StateExp () {

}

StateExp::~StateExp() {

}

StateExp::StateExp (const StateExp &stateIn) {

  for (int i = 0; i < 3; i++) 
    x[i] = stateIn.x[i];
}

StateExp& StateExp::operator=(const StateExp &stateIn){

  if (this == &stateIn)
    return *this;

  for (int i = 0; i < 3; i++) 
    x[i] = stateIn.x[i];

  return *this;
}


TrajectoryExp::TrajectoryExp () {
  totalVariation = -1.0;
}


TrajectoryExp::~TrajectoryExp () {

}


TrajectoryExp::TrajectoryExp (const TrajectoryExp &trajectoryIn) : endState(trajectoryIn.getEndState()) {

}


TrajectoryExp& TrajectoryExp::operator=(const TrajectoryExp &trajectoryIn) {

  if (this == &trajectoryIn)
    return *this;

  endState = trajectoryIn.getEndState();

  totalVariation = trajectoryIn.totalVariation;

  return *this;
}


int TrajectoryExp::getEndState (StateExp &getEndStateOut) {

  getEndStateOut = endState;

  return 1;
}


double TrajectoryExp::evaluateCost () {

  return totalVariation;
}


SystemExp::SystemExp ()
{
  turning_radii[0] = 4.5;
  turning_radii[1] = 6;
  turning_radii[2] = 8;

  distance_limit = 100.0;
  delta_distance = 0.05;

  double factor_reduce_size = 1.0;
  car_width = 1.8/factor_reduce_size;
  car_height = 3.6/factor_reduce_size;
  safe_distance = 0.2/factor_reduce_size;    // car footprint is blown up by this distance for collision checking
  distance_rear_axis_rear = 0.50/factor_reduce_size; // dist between center of rear axis and rear end
}


SystemExp::~SystemExp (){

}


int SystemExp::getStateKey (StateExp &stateIn, double *stateKey){
	double tmp[3] = {0};
	for(int i=0; i<3; i++)
		tmp[i] = stateIn.x[i] - map_origin[i];
	while(tmp[2] < -M_PI)
		tmp[2] += 2.0*M_PI;
	while(tmp[2] > M_PI)
		tmp[2] -= 2.0*M_PI;
	for (int i = 0; i < 3; i++)
		stateKey[i] =  tmp[i] / regionOperating.size[i];
	return 1;
}


#define SQ(x)   ((x)*(x))
float SystemExp::getGoalCost(const double x[3])
{
  double tmp = x[2] - regionGoal.center[2];
  while(tmp < -M_PI)
    tmp += 2.0*M_PI;
  while(tmp > M_PI)
    tmp -= 2.0*M_PI;
  return (sqrt(SQ(x[0]-regionGoal.center[0]) + SQ(x[1]-regionGoal.center[1])) + 4.0*fabs(tmp));
}


bool SystemExp::isReachingTarget (StateExp &stateIn) {

  for (int i = 0; i < 3; i++) {

    if (fabs(stateIn.x[i] - regionGoal.center[i]) > regionGoal.size[i]/2.0 )
      return false;
  }
  return true;
}

inline
int SystemExp::transform_map_to_local_map(const double stateIn[3], double &zlx, double &zly, double &yl)
{
  /*
   * X_map
   * |       X_car|
   * |            |
   * | Y_car------|
   * |
   * |--------- Y_map
   */
  // map frame z, yaw
  double zm[2] = {stateIn[0], stateIn[1]};
  double ym = stateIn[2];
  //cout<<"zm: "<< zm[0]<<" "<<zm[1]<<" "<<ym<<endl;

  // base_link frame
  //cout<<"map_origin "<<map_origin[0]<<" "<<map_origin[1]<<" "<<map_origin[2]<<endl;
  double cos_map_yaw = cos(map_origin[2]);
  double sin_map_yaw = sin(map_origin[2]);

  // rotate zm by yaw, subtract map_origin to get zlocal
  zlx = (zm[0]-map_origin[0])*cos_map_yaw + (zm[1]-map_origin[1])*sin_map_yaw;
  zly = -(zm[0]-map_origin[0])*sin_map_yaw + (zm[1]-map_origin[1])*cos_map_yaw;
  yl = ym - map_origin[2];
  while(yl > M_PI)
    yl -= 2.0*M_PI;
  while(yl < -M_PI)
    yl += 2.0*M_PI;

  return 0;
}

  inline
int SystemExp::get_cell_index(double x, double y, int &map_index)
{
  // find cells corresponding to (x,y)
  // car is placed at (height/4, width/2) according to the local_map

  int cellx = x/map.info.resolution + map.info.height/4.0;
  int celly = map.info.width/2.0 - y/map.info.resolution;

  if( (cellx >=0) && (cellx < (int)map.info.height) && (celly >= 0) && (celly < (int)map.info.width))
  {
    map_index = cellx*map.info.width + celly;
    return 0;
  }
  else
  {
    map_index = -1;
    return 1;
  }
}

  inline
int SystemExp::getxy_from_index(double &x, double &y, const int index)
{
  if( (index >= (int)map.info.height*map.info.width) || (index < 0))
    return 1;

  double cx = (index/map.info.width)*map.info.resolution;
  double cy = (index%map.info.width)*map.info.resolution;

  x = cx - map.info.height*map.info.resolution/4.0;
  y  = map.info.width*map.info.resolution/2.0 - cy;

  return 0;
}

bool SystemExp::IsInCollision (const double stateIn[3], bool debug_flag)
{
  // (x,y) in local_map frame
  double zl[2] = {0};
  // yaw in local frame
  double yl = 0;
  transform_map_to_local_map(stateIn, zl[0], zl[1], yl);
  //cout<<"zl: "<< zl[0]<<" "<<zl[1]<<" "<<yl<<endl; 
  double cos_yl = cos(yl);
  double sin_yl = sin(yl);


#if 1
  bool is_obstructed = false;
  double cxmax = car_height - distance_rear_axis_rear + safe_distance + 0.001;
  double cymax = car_width/2.0 + safe_distance + 0.001;
  double cy = -car_width/2.0 - safe_distance;

  while(cy < cymax)
  {
    double cx = -distance_rear_axis_rear - safe_distance;
    while(cx < cxmax)
    {
      // x = stateInLocal + rel position (cx,cy) transformed into the (X_car,Y_car) frame
      double x = zl[0] + cx*cos_yl + cy*sin_yl;
      double y = zl[1] - cx*sin_yl + cy*cos_yl;
      if(debug_flag)
        cout<<"x: "<< x<<" y: "<<y<<endl;
      //getchar();
      int map_index = -1;
      if(get_cell_index(x, y, map_index) == 0)
      {
        int to_check = map.data[map_index];
        if(debug_flag)
          cout<<"to_check: "<<to_check<<endl;
        if(to_check == 127)
        {
          is_obstructed = true;
          //cout<<"is_obstructed 184: "<<is_obstructed<<endl;
          return is_obstructed;
        }
      }
      else
      {
        is_obstructed = false;
        //cout<<"is_obstructed 184: "<<is_obstructed<<endl;
        //return is_obstructed;
      }

      cx = cx + map.info.resolution;
    }
    cy = cy + map.info.resolution;
  }

  return is_obstructed;
#endif
  return true;
}

double SystemExp::getLaneCost(const double zx, const double zy)
{
  int map_index = -1;
  if(get_cell_index(zx, zy, map_index) == 0)
  {
    int val = map.data[map_index];
    if(val == 127)
    {
        return 100;
    }
    else
    	//TODO: Test the parameter for cost map ratio
      return val/127*5;
  }
  else
    return 10;
}

double SystemExp::getStateCost(const double stateIn[3])
{
  // (x,y) in local_map frame
  double zlc[2] = {0};
  // yaw in local frame
  double ylc = 0;
  transform_map_to_local_map(stateIn, zlc[0], zlc[1], ylc);

  double cost = 0;
  double zll[2] = {0};
  double zlr[2] = {0};
  zlr[0] = zlc[0] + car_width/2.0*sin(ylc);
  zlr[1] = zlc[1] - car_width/2.0*cos(ylc);
  zll[0] = zlc[0] - car_width/2.0*sin(M_PI/2.0 - ylc);
  zll[1] = zlc[1] + car_width/2.0*cos(M_PI/2.0 - ylc);

  cost += getLaneCost(zlc[0], zlc[1]);
  cost += getLaneCost(zll[0], zll[1]);
  cost += getLaneCost(zlr[0], zlr[1]);
  return cost/3.0;
}

#if 0
   int SystemExp::sampleState (StateExp &randomStateOut) {

   for (int i = 0; i < 3; i++) 
   {
   randomStateOut.x[i] = (double)rand()/(RAND_MAX + 1.0)*regionOperating.size[i] 
   - regionOperating.size[i]/2.0 + regionOperating.center[i];
   }
//cout<<"sample_local: "<<randomStateOut.x[0]<<" "<<randomStateOut.x[1]<<" "<<randomStateOut.x[2]<<endl;

// transform the sample from local_map frame to /map frame
double cyaw = cos(map_origin[2]);
double syaw = sin(map_origin[2]);

double local_map_state[3] = {0};
local_map_state[0] = randomStateOut.x[0] + map.info.resolution*map.info.height/4.0;
local_map_state[1] = randomStateOut.x[1];           // the origin of regionOperating is on the X axis of the local_map_frame 
local_map_state[2] = randomStateOut.x[2];

randomStateOut.x[0] = map_origin[0] + local_map_state[0]*cyaw + local_map_state[1]*syaw;
randomStateOut.x[1] = map_origin[1] + -local_map_state[0]*syaw + local_map_state[1]*cyaw;
randomStateOut.x[2] = map_origin[2] + local_map_state[2];
while(randomStateOut.x[2] > M_PI)
randomStateOut.x[2] -= 2.0*M_PI;
while(randomStateOut.x[2] < -M_PI)
randomStateOut.x[2] += 2.0*M_PI;

//cout<<"sample_transformed: "<<randomStateOut.x[0]<<" "<<randomStateOut.x[1]<<" "<<randomStateOut.x[2]<<endl;
if (IsInCollision (randomStateOut.x))
return 0;

return 1;
}

#else if

int SystemExp::sampleState(StateExp &z)
{
  int r = (double)(rand()/(RAND_MAX+1.0))*free_cells.size();

  int index = free_cells[r];
  //cout << "sample index"<< index<< endl;
  double local_xy[3] ={0, 0, 0};
  getxy_from_index(local_xy[0], local_xy[1], index);

  for(int i=0; i<3; i++)
  {
    z.x[i] = local_xy[i] + (double)rand()/(RAND_MAX + 1.0)*regionCell.size[i] ;
      //- regionCell.size[i]/2.0 + regionCell.center[i];
  }

  // transform the sample from local_map frame to /map frame
  double cyaw = cos(-map_origin[2]);
  double syaw = sin(-map_origin[2]);
  double state_copy[3];
  for(int i=0; i<3; i++)
    state_copy[i] = z[i];

  z.x[0] = map_origin[0] + state_copy[0]*cyaw + state_copy[1]*syaw;
  z.x[1] = map_origin[1] + -state_copy[0]*syaw + state_copy[1]*cyaw;
  z.x[2] = map_origin[2] + state_copy[2];
  while(z.x[2] > M_PI)
    z.x[2] -= 2.0*M_PI;
  while(z.x[2] < -M_PI)
    z.x[2] += 2.0*M_PI;

  //cout<<"sampled StateExp: "<<z.x[0]<<" "<<z.x[1]<<" "<<z.x[2]<<endl;
  if (IsInCollision (z.x))
    return 0;

  return 1;
}
#endif

int SystemExp::sampleGoalState (StateExp &randomStateOut) {

  for (int i = 0; i < 3; i++) {

    randomStateOut.x[i] = (double)rand()/(RAND_MAX + 1.0)*regionGoal.size[i] 
      - regionGoal.size[i]/2.0 + regionGoal.center[i];
  }

  while(randomStateOut.x[2] > M_PI)
    randomStateOut.x[2] -= 2.0*M_PI;
  while(randomStateOut.x[2] < -M_PI)
    randomStateOut.x[2] += 2.0*M_PI;

  //cout<<"sampleGoal"<<endl;
  if (IsInCollision (randomStateOut.x))
  {
    return 0;
  }
  //cout<<"sampled goal: "<< randomStateOut.x[0]<<" "<< randomStateOut.x[1]<< " "<<randomStateOut.x[2]<<endl;
  return 1;
}

double SystemExp::extend_dubins_spheres (double x_s1, double y_s1, double t_s1,
    double x_s2, double y_s2, double t_s2, int comb_no, 
    bool check_obstacles, bool return_trajectory,
    bool& fully_extends, double*& end_state, list<double*>* trajectory, list<float> &control, double turning_radius) {

  double x_tr = x_s2 - x_s1;
  double y_tr = y_s2 - y_s1;
  double t_tr = atan2 (y_tr, x_tr);

  double distance = sqrt ( x_tr*x_tr + y_tr*y_tr );

  double x_start;
  double y_start;
  double t_start = 0;
  double x_end;
  double y_end;
  double t_end = 0;

  if (distance > 2 * turning_radius) 
  {  
    // disks do not intersect
    double t_balls = acos (2 * turning_radius / distance);
    switch (comb_no) 
    {
      case 1:
        t_start = t_tr - t_balls;
        t_end = t_tr + M_PI - t_balls;
        break;
      case 2:
        t_start = t_tr + t_balls;
        t_end = t_tr - M_PI + t_balls;
        break;
      case 3:
        t_start = t_tr - M_PI_2;
        t_end = t_tr - M_PI_2;
        break;
      case 4:
        t_start = t_tr + M_PI_2;
        t_end = t_tr + M_PI_2;
        break;
      default:
        return -1.0;
    }
  }
  else 
  { 
    // disks are intersecting
    switch (comb_no) 
    {
      case 1:
      case 2:
        // No solution
        return -1.0;
        break;

      case 3:
        t_start = t_tr - M_PI_2;
        t_end = t_tr - M_PI_2;
        break;
      case 4:
        t_start = t_tr + M_PI_2;
        t_end = t_tr + M_PI_2;
        break;
    }
  }

  x_start = x_s1 + turning_radius * cos (t_start);
  y_start = y_s1 + turning_radius * sin (t_start);
  x_end = x_s2 + turning_radius * cos (t_end);
  y_end = y_s2 + turning_radius * sin (t_end);

  double line_distance = sqrt ((x_start-x_end)*(x_start-x_end) + (y_start-y_end)*(y_start-y_end) );

  int direction_s1 = 1;
  if ( (comb_no == 2) || (comb_no == 4) ) {
    direction_s1 = -1;
  }
  int direction_s2 = 1;
  if ( (comb_no == 1) || (comb_no == 4) ) {
    direction_s2 = -1;
  }

  double t_increment_s1 = direction_s1 * (t_start - t_s1);
  double t_increment_s2 = direction_s2 * (t_s2 - t_end);

  while (t_increment_s1 < 0) 
    t_increment_s1 += 2.0 * M_PI;
  while (t_increment_s1 > 2.0 *M_PI)
    t_increment_s1 -= 2.0 * M_PI;

  while (t_increment_s2 < 0) 
    t_increment_s2 += 2.0 * M_PI;
  while (t_increment_s2 > 2.0 *M_PI)
    t_increment_s2 -= 2.0 * M_PI;

  if  ( ( (t_increment_s1 > M_PI) && (t_increment_s2 > M_PI) ) 
      || ( (t_increment_s1 > 3*M_PI_2) || (t_increment_s2 > 3*M_PI_2) )  ){
    return -1.0;
  }

  // different costs
  double ts1c = t_increment_s1;
  double ts2c = t_increment_s2;
  while(ts1c > M_PI)
    ts1c -= 2.0*M_PI;
  while(ts1c < -M_PI)
    ts1c += 2.0*M_PI;
  while(ts2c > M_PI)
    ts2c -= 2.0*M_PI;
  while(ts2c < -M_PI)
    ts2c += 2.0*M_PI;

  double time_cost = (( fabs(ts1c) + fabs(ts2c)) * turning_radius  + line_distance);
  double local_map_cost = 0;
  double turning_cost = 2.5 * ( fabs(ts1c) + fabs(ts2c));
  time_cost += turning_cost;

  fully_extends = 0;

  if (check_obstacles) 
  {
    // Generate states/inputs
    double del_d = delta_distance;
    double del_t = del_d/turning_radius;
    int max_counter = map.info.resolution/del_d;
    //cout<<"max_counter: "<<max_counter<<endl;

    double t_inc_curr = 0.0;

    double state_curr[3] = {0};

    int obs_check_counter = 0;
    while (t_inc_curr < t_increment_s1) 
    {
      double t_inc_rel = del_t;
      t_inc_curr += del_t;
      if (t_inc_curr > t_increment_s1) 
      {
        t_inc_rel -= t_inc_curr - t_increment_s1;
        t_inc_curr = t_increment_s1;
      }

      state_curr[0] = x_s1 + turning_radius * cos (direction_s1 * t_inc_curr + t_s1);
      state_curr[1] = y_s1 + turning_radius * sin (direction_s1 * t_inc_curr + t_s1);
      state_curr[2] = direction_s1 * t_inc_curr + t_s1 + ( (direction_s1 == 1) ? M_PI_2 : 3.0*M_PI_2);
      obs_check_counter++;

      while (state_curr[2] < -M_PI)
        state_curr[2] += 2.0 * M_PI;
      while (state_curr[2] > M_PI)
        state_curr[2] -= 2.0*M_PI;

      // check for collision only if counter = map.info.resolution/delta_d
      if(obs_check_counter == max_counter)
      {
        obs_check_counter = 0;
        if (IsInCollision (state_curr))
          return -2.0;
      }
      local_map_cost += getStateCost(state_curr);
      if (trajectory) 
      {
        double *state_new = new double[3];
        for (int i = 0; i < 3; i++) 
          state_new[i] = state_curr[i];

        trajectory->push_front(state_new);
        control.push_front (direction_s1*turning_radius);
      }

      if (t_inc_curr * turning_radius > distance_limit)  
      {
        fully_extends = false;

        for (int i = 0; i < 3; i++)
          end_state[i] = state_curr[i];

        return time_cost + local_map_cost;
      }
    }

    obs_check_counter = 0;
    double d_inc_curr = 0.0;
    while (d_inc_curr < distance) 
    {
      double d_inc_rel = del_d;
      d_inc_curr += del_d;
      if (d_inc_curr > distance) {
        d_inc_rel -= d_inc_curr - distance;
        d_inc_curr = distance;
      }

      state_curr[0] = (x_end - x_start) * d_inc_curr / distance + x_start; 
      state_curr[1] = (y_end - y_start) * d_inc_curr / distance + y_start; 
      state_curr[2] = direction_s1 * t_inc_curr + t_s1 + ( (direction_s1 == 1) ? M_PI_2 : 3.0*M_PI_2);
      obs_check_counter++;

      while (state_curr[2] < -M_PI)
        state_curr[2] += 2.0 * M_PI;
      while (state_curr[2] > M_PI)
        state_curr[2] -= 2.0*M_PI;

      // input_curr[0] = d_inc_rel;
      // input_curr[1] = 0.0;

      if(obs_check_counter == max_counter)
      {
        obs_check_counter = 0;
        // check for collision
        if (IsInCollision (state_curr))
          return -2.0;
      }
      local_map_cost += getStateCost(state_curr);
      if (trajectory) {
        double *state_new = new double [3];
        for (int i = 0; i < 3; i++) 
          state_new[i] = state_curr[i];
        trajectory->push_front(state_new);
        control.push_front (0);
      }

      if (t_inc_curr * turning_radius + d_inc_curr > distance_limit) 
      {
        fully_extends = false;

        for (int i = 0; i < 3; i++)
          end_state[i] = state_curr[i];

        return time_cost + local_map_cost;
      }
    }

    double t_inc_curr_prev = t_inc_curr;
    t_inc_curr = 0.0;
    obs_check_counter = 0;
    while (t_inc_curr < t_increment_s2) 
    {
      double t_inc_rel = del_t;
      t_inc_curr += del_t;
      if (t_inc_curr > t_increment_s2)  {
        t_inc_rel -= t_inc_curr - t_increment_s2;
        t_inc_curr = t_increment_s2;
      }

      state_curr[0] = x_s2 + turning_radius * cos (direction_s2 * (t_inc_curr - t_increment_s2) + t_s2);
      state_curr[1] = y_s2 + turning_radius * sin (direction_s2 * (t_inc_curr - t_increment_s2) + t_s2);
      state_curr[2] = direction_s2 * (t_inc_curr - t_increment_s2) + t_s2 
        + ( (direction_s2 == 1) ?  M_PI_2 : 3.0*M_PI_2 );
      obs_check_counter++;

      while (state_curr[2] < -M_PI)
        state_curr[2] += 2.0 * M_PI;
      while (state_curr[2] > M_PI)
        state_curr[2] -= 2.0*M_PI;

      // input_curr[0] = t_inc_rel * turning_radius;
      // input_curr[1] = ( (comb_no == 2) || (comb_no == 3) ) ? -1 : 1;

      if(obs_check_counter == max_counter)
      {
        obs_check_counter = 0;
        // check for collision
        if (IsInCollision (state_curr))
          return -2.0;
      }
      local_map_cost += getStateCost(state_curr);
      if (trajectory) 
      {
        double *state_new = new double [3];
        for (int i = 0; i < 3; i++) 
          state_new[i] = state_curr[i];
        trajectory->push_front(state_new);
        control.push_front(turning_radius * direction_s2);
      }

      if ((t_inc_curr_prev + t_inc_curr) * turning_radius + d_inc_curr > distance_limit) 
      {
        fully_extends = false;
        for (int i = 0; i < 3; i++)
          end_state[i] = state_curr[i];

        return time_cost + local_map_cost;
      }
    }

    fully_extends = true;

    for (int i = 0; i < 3; i++)
      end_state[i] = state_curr[i];
  }
  //TODO: Decide the ratio
  return time_cost + 0.5 * local_map_cost;
}

double 
SystemExp::extend_dubins_all (double state_ini[3], double state_fin[3],
    bool check_obstacles, bool return_trajectory,
    bool &fully_extends, double*& end_state, list<double*>* trajectory, list<float> &control, double turning_radius) {

  double ti = state_ini[2];
  double tf = state_fin[2];
  double sin_ti = sin (-ti);
  double cos_ti = cos (-ti);
  double sin_tf = sin (-tf);
  double cos_tf = cos (-tf);

  double si_left[3] = {
    state_ini[0] + turning_radius * sin_ti,
    state_ini[1] + turning_radius * cos_ti,
    ti + 3 * M_PI_2
  };
  double si_right[3] = {
    state_ini[0] - turning_radius * sin_ti,
    state_ini[1] - turning_radius * cos_ti,
    ti + M_PI_2
  };
  double sf_left[3] = {
    state_fin[0] + turning_radius * sin_tf,
    state_fin[1] + turning_radius * cos_tf,
    tf + 3 * M_PI_2
  };
  double sf_right[3] = {
    state_fin[0] - turning_radius * sin_tf,
    state_fin[1] - turning_radius * cos_tf,
    tf + M_PI_2
  };

  // 2. extend all four spheres
  double times[4]; 

  bool exact_connection[4];

  times[0] = extend_dubins_spheres (si_left[0], si_left[1], si_left[2], 
      sf_right[0], sf_right[1], sf_right[2], 1,
      false, false,
      exact_connection[0], end_state, NULL, control, turning_radius);
  times[1] = extend_dubins_spheres (si_right[0], si_right[1], si_right[2], 
      sf_left[0], sf_left[1], sf_left[2], 2,
      false, false,
      exact_connection[1], end_state, NULL, control, turning_radius);
  times[2] = extend_dubins_spheres (si_left[0], si_left[1], si_left[2], 
      sf_left[0], sf_left[1], sf_left[2], 3, 
      false, false,
      exact_connection[2], end_state, NULL, control, turning_radius);
  times[3] = extend_dubins_spheres (si_right[0], si_right[1], si_right[2], 
      sf_right[0], sf_right[1], sf_right[2], 4, 
      false, false,
      exact_connection[3], end_state, NULL, control, turning_radius);

  double min_time = DBL_MAX;
  int comb_min = -1;
  for (int i = 0; i < 4; i++) {
    if  ( (times[i] >= 0.0) && (times[i] < min_time) ) {
      comb_min = i+1;
      min_time = times[i];
    }
  }

  if (comb_min == -1)
    return -1.0;

  if (check_obstacles == false) {
    fully_extends = exact_connection[comb_min-1];    
    return min_time;
  }


  double res;
  switch (comb_min) {
    case 1:
      res = extend_dubins_spheres (si_left[0], si_left[1], si_left[2], 
          sf_right[0], sf_right[1], sf_right[2], 1,
          true, return_trajectory,
          fully_extends, end_state, trajectory, control, turning_radius);
      return res;

    case 2:
      res = extend_dubins_spheres (si_right[0], si_right[1], si_right[2], 
          sf_left[0], sf_left[1], sf_left[2], 2, 
          true, return_trajectory,
          fully_extends, end_state, trajectory, control, turning_radius);
      return res;

    case 3:
      res = extend_dubins_spheres (si_left[0], si_left[1], si_left[2], 
          sf_left[0], sf_left[1], sf_left[2], 3, 
          true, return_trajectory,
          fully_extends, end_state, trajectory, control, turning_radius);
      return res;

    case 4:
      res = extend_dubins_spheres (si_right[0], si_right[1], si_right[2], 
          sf_right[0], sf_right[1], sf_right[2], 4, 
          true, return_trajectory,
          fully_extends, end_state, trajectory, control, turning_radius);
      return res;

    case -1:
    default:
      return -1.0;
  }
}

int SystemExp::extendTo (StateExp &stateFromIn, StateExp &stateTowardsIn,
    TrajectoryExp &trajectoryOut, bool &exactConnectionOut, list<float> &controlOut, bool check_obstacles) {

  double *end_state = new double [3];

  double min_cost = DBL_MAX;
  double best_turning_radius = 100.0; 
  for(int i=num_turning_radii -1; i >= 0; i--)
  {
    double *tmp_end_state = new double [3];
    double turning_radius = turning_radii[i];
    bool tmp_exact_connection = false;
    list<float> tmp_control;

    double cost = extend_dubins_all (stateFromIn.x, stateTowardsIn.x, 
        check_obstacles, false, 
        tmp_exact_connection, tmp_end_state, NULL, tmp_control, turning_radius);
    if(cost > 0.0)
    {
      if(cost < min_cost)
      {
        for(int j=0; j<3; j++)
          end_state[j] = tmp_end_state[j];
        min_cost = cost;
        best_turning_radius = turning_radius;
        exactConnectionOut = tmp_exact_connection;
        controlOut = tmp_control;
      }
    }
    delete[] tmp_end_state;
  }
  if((min_cost <= 0.0) || (min_cost > 1000.0))
  {
    delete[] end_state;
    return 0;
  }
  //cout<<"min_cost: "<< min_cost <<" tr: "<<best_turning_radius<<endl;
  //cout<<"tr: "<< best_turning_radius<<endl;
  while (end_state[2] < -M_PI)
    end_state[2] += 2.0 * M_PI;
  while (end_state[2] > M_PI)
    end_state[2] -= 2.0 * M_PI;

  for (int i = 0; i < 3; i++) {
    trajectoryOut.endState.x[i] = end_state[i];
  }

  trajectoryOut.totalVariation = min_cost;

  delete [] end_state;

  return 1;
}


double SystemExp::evaluateExtensionCost (StateExp &stateFromIn, StateExp &stateTowardsIn, bool &exactConnectionOut)
{

  double *end_state = new double[3];

  double min_cost = DBL_MAX;
  for(int i=num_turning_radii -1; i >= 0; i--)
  {
    double turning_radius = turning_radii[i];
    bool tmp_exact_connection = false;
    list<float> tmp_control;

    double cost = extend_dubins_all (stateFromIn.x, stateTowardsIn.x, 
        false, false, 
        tmp_exact_connection, end_state, NULL, tmp_control, turning_radius);
    if(cost > 0.0)
    {
      if(cost < min_cost)
      {
        min_cost = cost;
        exactConnectionOut = tmp_exact_connection;
      }
    }
  }
  delete[] end_state;
  return min_cost;
}


int SystemExp::clear_tmp_trajectories(list<double*> &state_traj, list<float> &control_traj)
{
  for(list<double*>::iterator i=state_traj.begin(); i!= state_traj.end(); i++)
  {
    delete[] (*i);
  }
  state_traj.clear();
  control_traj.clear();
  return 0;
}

int SystemExp::getTrajectory (StateExp& stateFromIn, StateExp& stateToIn, list<double*>& trajectoryOut, list<float>& controlOut, bool check_obstacles) {


  double min_cost = DBL_MAX;
  bool exactConnectionOut = false;
  for(int i=num_turning_radii -1; i >= 0; i--)
  {
    list<double*> tmp_traj;
    list<float> tmp_control;
    bool tmp_exact_connection = false;
    double *end_state = new double[3];
    double turning_radius = turning_radii[i];

    double cost = extend_dubins_all (stateFromIn.x, stateToIn.x, 
        check_obstacles, true, 
        tmp_exact_connection, end_state, &tmp_traj, tmp_control, turning_radius);
    if(cost > 0.0)
    {
      if(cost < min_cost)
      {
        min_cost = cost;
        trajectoryOut = tmp_traj;
        controlOut = tmp_control;
        exactConnectionOut = tmp_exact_connection;
      }
    }
    else
    {
      //clear tmp_traj
      clear_tmp_trajectories(tmp_traj, tmp_control);
    }
    delete [] end_state;
  }
  //cout<<"trajOut size: "<<trajectoryOut.size()<<" min_cost: "<<min_cost<<" exact: "<< exactConnectionOut<<endl;

  if ((min_cost <= 0.0) || (exactConnectionOut == false))
  {
    return 0;
  }

  trajectoryOut.reverse();

  return 1;
}

double SystemExp::evaluateCostToGo (StateExp& stateIn)
{
  double size_x = regionGoal.size[0];
  double size_y = regionGoal.size[1];
  double radius = sqrt(size_x*size_x + size_y*size_y);

  double dist_x = stateIn[0] - regionGoal.center[0];
  double dist_y = stateIn[1] - regionGoal.center[1];
  double dist = sqrt (dist_x*dist_x + dist_y*dist_y);

  return dist - radius;

}
