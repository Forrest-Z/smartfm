#include "mvdubins.h"

Subset_of_Sigma MVSystem::label_state( double stateIn[3]) 
{
  Subset_of_Sigma label_of_state;
  double zl[2] = {0};
  // yaw in local frame
  double yl = 0;
  transform_map_to_local_map(stateIn, zl[0], zl[1], yl);
  int map_index = -1;
  if(get_cell_index(zl[0], zl[1], map_index)==0)
  {
    int val = map.data[map_index];
    if(val == 87)
      label_of_state.insert_AP(GOOD_DRCT);
  }
  return label_of_state;
}

double MVSystem::extend_dubins_spheres( 
    double x_s1, double y_s1, double t_s1, 
    double x_s2, double y_s2, double t_s2, 
    int combination, bool check_obstacles, bool return_control, 
    bool& fully_extends, 
    double* &end_state, 
    list<double*>* trajectory, 
    list<float>& control, double turning_radius) 
{

  fully_extends = false;

  double x_tr = x_s2 - x_s1;
  double y_tr = y_s2 - y_s1;
  double t_tr = atan2( y_tr, x_tr);

  double distance = sqrt ( (x_tr * x_tr) + (y_tr * y_tr) );

  double x_start;
  double y_start;
  double t_start = 0;
  double x_end;
  double y_end;
  double t_end = 0;

  if (distance > 2 * turning_radius) {  // disks do not intersect 

    double t_balls = acos (2 * turning_radius / distance);

    switch (combination) {
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

  else { // disks do intersect
    switch (combination) {
      case 1:
        // See next case
      case 2:
        // There is no solution
        return -1.0; break;
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

  x_start     = x_s1 + turning_radius * cos (t_start);
  y_start     = y_s1 + turning_radius * sin (t_start);
  x_end       = x_s2 + turning_radius * cos (t_end);
  y_end       = y_s2 + turning_radius * sin (t_end);

  int direction_s1 = 1;
  if ( (combination == 2) || (combination == 4) ) {
    direction_s1 = -1;
  }
  int direction_s2 = 1;
  if ( (combination == 1) || (combination == 4) ) {
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

  double total_distance_travel = 
    (t_increment_s1 + t_increment_s2) * turning_radius  + distance;

  if (check_obstacles) {

    double del_d = delta_distance;
    double del_t = del_d / turning_radius;

    double t_inc_curr = 0.0;
    double state_curr[3] = { x_s1, y_s1, t_s1};

    // Declare and initialize label of initial state
    Subset_of_Sigma label_s1 = label_state(state_curr);
    Subset_of_Sigma label_init = label_s1;
    // Declare label of current state
    Subset_of_Sigma label_curr;
    // Indicates that there is a discontinuity in the 
    // state labeling function along this trajectory
    bool label_fun_discont = false;

    while (t_inc_curr < t_increment_s1) {

      t_inc_curr += del_t;
      if (t_inc_curr > t_increment_s1) {
        t_inc_curr = t_increment_s1;
      }

      state_curr[0] = 
        x_s1 + turning_radius * cos (direction_s1 * t_inc_curr + t_s1);
      state_curr[1] = 
        y_s1 + turning_radius * sin (direction_s1 * t_inc_curr + t_s1);
      state_curr[2] = direction_s1 * t_inc_curr + 
        t_s1 + ( (direction_s1 == 1) ? M_PI_2 : ( 3.0 * M_PI_2) );

      while (state_curr[2] < 0)
        state_curr[2] += 2 * M_PI;
      while (state_curr[2] > 2 * M_PI)
        state_curr[2] -= 2 * M_PI;

      // Check for collision
      if ( IsInCollision(state_curr) ) return -2.0;

      // Retrieve label of current state
      label_curr = label_state(state_curr);
      // If label is not equal to initial label
      if ( label_curr != label_init ) {
        // If discontinuity in the state labeling 
        // function had not been observed before
        if ( !label_fun_discont ) {
          // Redefine initial label to label of current state
          label_init = label_curr; 
          label_fun_discont = true;
        }
        // Otherwise, trajectory is not trace inclusive
        else return -3.0;
      }

      if ( trajectory || return_control ) {

        double * state_new = new double[3];

        for (int i = 0; i < 3; i++)
          state_new[i] = state_curr[i];

        while(state_new[2] > M_PI)
          state_new[2] -= 2.0 * M_PI;
        while(state_new[2] < -M_PI)
          state_new[2] += 2.0 * M_PI;

        // If trajectory is requested
        if (trajectory) {
          // Insert state into trajectory
          trajectory -> push_back(state_new);
        }
        // If input signal is requested
        if (return_control) {
          // Insert control into input signal
          control.push_back( direction_s1 * turning_radius );
        }

      }

      if ( t_inc_curr * turning_radius > distance_limit )  {
        fully_extends = false;
        for (int i = 0; i < 3; i++)
          end_state[i] = state_curr[i];
        return total_distance_travel;
      }

    }

    double d_inc_curr = 0.0;

    while ( d_inc_curr < distance ) {

      d_inc_curr += del_d;
      if (d_inc_curr > distance) {
        d_inc_curr = distance;
      }

      state_curr[0] = 
        (x_end - x_start) * d_inc_curr / distance + x_start; 
      state_curr[1] = 
        (y_end - y_start) * d_inc_curr / distance + y_start; 
      state_curr[2] = direction_s1 * t_inc_curr + 
        t_s1 + ( (direction_s1 == 1) ? M_PI_2 : (3.0 * M_PI_2) );

      while (state_curr[2] < 0)
        state_curr[2] += 2 * M_PI;
      while (state_curr[2] > 2 * M_PI)
        state_curr[2] -= 2 * M_PI;

      // Check for collision
      if ( IsInCollision(state_curr) ) 
        return -2.0;

      // Retrieve label of current state
      label_curr = label_state(state_curr);
      // If label is not equal to initial label
      if ( label_curr != label_init ) {
        // If discontinuity in the state labeling 
        // function had not been observed before
        if ( !label_fun_discont ) {
          // Redefine initial label to label of current state
          label_init = label_curr; 
          label_fun_discont = true;
        }
        // Otherwise, trajectory is not trace inclusive
        else 
          return -3.0;
      }

      if ( trajectory || return_control ) {

        double *state_new = new double [3];

        for (int i = 0; i < 3; i++) 
          state_new[i] = state_curr[i];

        while(state_new[2] > M_PI)
          state_new[2] -= 2.0 * M_PI;
        while(state_new[2] < -M_PI)
          state_new[2] += 2.0 * M_PI;

        // If trajectory is requested
        if (trajectory) {
          // Insert state into trajectory
          trajectory -> push_back(state_new);
        }
        // If input signal is requested
        if (return_control) {
          // Insert control into input signal
          control.push_back(0.0);
        }
      }

      if ( t_inc_curr * turning_radius + d_inc_curr > distance_limit ) 
      {
        fully_extends = false;
        for (int i = 0; i < 3; i++)
          end_state[i] = state_curr[i];
        return total_distance_travel;
      }

    }

    double t_inc_curr_prev = t_inc_curr;
    t_inc_curr = 0.0;

    while (t_inc_curr < t_increment_s2) {

      t_inc_curr += del_t;

      if (t_inc_curr > t_increment_s2)  {
        t_inc_curr = t_increment_s2;
      }

      state_curr[0] = x_s2 
        + turning_radius 
        * cos (direction_s2 * (t_inc_curr - t_increment_s2) + t_s2);
      state_curr[1] = y_s2 
        + turning_radius 
        * sin (direction_s2 * (t_inc_curr - t_increment_s2) + t_s2);
      state_curr[2] = 
        direction_s2 * (t_inc_curr - t_increment_s2) 
        + t_s2 + ( (direction_s2 == 1) ?  M_PI_2 : (3.0 * M_PI_2) );

      while (state_curr[2] < 0)
        state_curr[2] += 2 * M_PI;
      while (state_curr[2] > 2 * M_PI)
        state_curr[2] -= 2 * M_PI;

      // Check for collision
      if ( IsInCollision(state_curr) ) 
        return -2.0;

      // Retrieve label of current state
      label_curr = label_state(state_curr);
      // If label is not equal to initial label
      if ( label_curr != label_init ) {
        // If discontinuity in the state labeling 
        // function had not been observed before
        if ( !label_fun_discont ) {
          // Redefine initial label to label of current state
          label_init = label_curr; 
          label_fun_discont = true;
        }
        // Otherwise, trajectory is not trace inclusive
        else
        {
          //std::cout<<"trace inclusive second sphere"<< std::endl;
          return -3.0;
        }
      }

      if ( trajectory || return_control ) {

        double *state_new = new double [3];

        for (int i = 0; i < 3; i++) 
          state_new[i] = state_curr[i];

        while(state_new[2] >  M_PI) state_new[2] -= 2.0 * M_PI;
        while(state_new[2] < -M_PI) state_new[2] += 2.0 * M_PI;

        // If trajectory is requested
        if (trajectory) {
          // Insert state into trajectory
          trajectory -> push_back(state_new);
        }
        // If input signal is requested
        if (return_control) {
          // Insert control into input signal
          control.push_back( direction_s2 * turning_radius );
        }

      }

      if ( (t_inc_curr_prev + t_inc_curr) 
          * turning_radius + d_inc_curr > distance_limit ) {
        fully_extends = false;
        for (int i = 0; i < 3; i++)
          end_state[i] = state_curr[i];
        return total_distance_travel;
      }

    }

    fully_extends = true;

    for (int i = 0; i < 3; i++)
      end_state[i] = state_curr[i];

  }

  return total_distance_travel;
}
