#include "dubins_car.h"
#include <cmath>
#include <cstdlib>
#include <inttypes.h>
#include <iostream>
#include <stdio.h>

using namespace std;

region::region () {

}


region::~region () {

}


State::State () {

}


State::~State() {
}


State::State (const State &stateIn) {

    for (int i = 0; i < 3; i++) 
        x[i] = stateIn.x[i];
}


State& State::operator=(const State &stateIn){

    if (this == &stateIn)
        return *this;

    for (int i = 0; i < 3; i++) 
        x[i] = stateIn.x[i];

    return *this;
}


Trajectory::Trajectory () {

    totalVariation = -1.0;
}


Trajectory::~Trajectory () {

}


Trajectory::Trajectory (const Trajectory &trajectoryIn) : endState(trajectoryIn.getEndState()) {

}


Trajectory& Trajectory::operator=(const Trajectory &trajectoryIn) {

    if (this == &trajectoryIn)
        return *this;

    endState = trajectoryIn.getEndState();

    totalVariation = trajectoryIn.totalVariation;

    return *this;
}


int Trajectory::getEndState (State &getEndStateOut) {

    getEndStateOut = endState;

    return 1;
}


double Trajectory::evaluateCost () {

    return totalVariation;
}


System::System () 
{
    turning_radius = 4.0;
    distance_limit = 100.0;
    delta_distance = 0.15;

    car_width = 1.2;
    car_height = 2.2;
}


System::~System () {
}


int System::getStateKey (State &stateIn, double *stateKey) {

    for (int i = 0; i < 3; i++) 
        stateKey[i] =  stateIn.x[i] / regionOperating.size[i];

    return 1;
}


bool System::isReachingTarget (State &stateIn) {

    for (int i = 0; i < 3; i++) {

        if (fabs(stateIn.x[i] - regionGoal.center[i]) > regionGoal.size[i]/2.0 )
            return false;
    }
    return true;
}

bool System::IsInCollision (const double stateIn[3]) 
{
    //return false;

    //cout<<"attitude: "<<roll<<" "<<pitch<<" "<<yaw<<endl;

    //cout<<"134: stateIn: "<<stateIn[0]<<" "<<stateIn[1]<<" "<<stateIn[2]<<endl;
    // convert stateIn to local coords
    double stateInLocal[3] = {0};
    for(int i=0; i<3; i++)
        stateInLocal[i] = stateIn[i] - map_origin[i];

    while(stateInLocal[2] > M_PI)
        stateInLocal[2] -= 2.0*M_PI;
    while(stateInLocal[2] < -M_PI)
        stateInLocal[2] += 2.0*M_PI;
    //cout<<"stateCheck: "<<stateInLocal[0]<<" "<<stateInLocal[1]<<" "<<stateInLocal[2]<<endl;
    
    //cout<<"regionOperating_center: "<< regionOperating.center[0]<<" "<< regionOperating.center[1]<<" "<< regionOperating.center[2]<<endl;
    //cout<<"regionOperating_size: "<< regionOperating.size[0]<<" "<< regionOperating.size[1]<<" "<< regionOperating.size[2]<<endl;
    /*
    for(int i=0; i<3; i++)
    {
        if( (stateIn[i] > regionOperating.center[i] + regionOperating.size[i]/2.0) || (stateIn[i] < regionOperating.center[i] -regionOperating.size[i]/2.0) )
        {
            cout<<"returning line 152"<<endl;
            return true;
        }
    }
    */
    double cyaw = cos(stateInLocal[2]);
    double syaw = sin(stateInLocal[2]);

    bool is_obstructed = false;
    double cy = -car_width/2.0;
    double cx = -car_height/2.0;
    while(cy < car_width/2.0)
    {
        while(cx < car_height/2.0)
        {
            // 1. check the point (stateIn[0], stateIn[1]) + (cx,cy) -> R(-yaw)
            double x = stateInLocal[0] + cx*cyaw + cy*syaw;
            double y = stateInLocal[1] - cx*syaw + cy*cyaw;
            //cout<<"x: "<< x<<" y: "<<y<<endl;

            // 2. find cells corresponding to (x,y)
            // car is placed at (height/4, width/2) according to the local_map
            /*
             * X_map
             * |    X_car
             * |    |
             * | ---| Y_car
             * |
             * |--------- Y_map
             */
            int cellx = x/map.info.resolution + map.info.height/4.0;
            int celly = map.info.width/2.0 - y/map.info.resolution;
            //cout<<"cellx: "<<cellx<<" celly: "<<celly<<endl;
            if( (cellx >=0) && (cellx < (int)map.info.height) && (celly >= 0) && (celly < (int)map.info.width))
            {
                int to_check = cellx*map.info.width + celly;
                //cout<<"to_check: "<< to_check<<" map.data: "<<(int)map.data[to_check]<<endl;
                if(map.data[to_check] == 0)
                {
                    is_obstructed = true;
                    break;
                }
            }
            else
            {
                is_obstructed = true;
                break;
            }
            cx = cx + map.info.resolution;
        }
        cy = cy + map.info.resolution;
    }
    
    //cout<<"is_obstructed: "<<is_obstructed<<endl;
    return is_obstructed;
}

int System::getStateCost(const double stateIn[3])
{
    /*
    // get cell_num of stateIn
    double xtmp = stateIn[0] - origin.x;
    double ytmp = stateIn[1] - origin.y;

    int xnum, ynum;
    xnum = xtmp/map_res + xorigin;
    ynum = ytmp/map_res + yorigin;

    if( (xnum >= xsize) || (xnum < 0) || (ynum >= ysize) || (ynum < 0) )
    {
        return 0;
    }
    else
    {
        //cout<<"grid: " << xback <<" "<< xfront <<" "<< yleft <<" "<< yright << endl;
        if(map_vals[ynum + xnum*xsize] >= 250)
            return 250;
        else
            return (250 - map_vals[ynum + xnum*xsize]);
    }
    */
    return 0;
}

int System::sampleState (State &randomStateOut) {

    for (int i = 0; i < 3; i++) {

        randomStateOut.x[i] = (double)rand()/(RAND_MAX + 1.0)*regionOperating.size[i] 
            - regionOperating.size[i]/2.0 + regionOperating.center[i];
    }
    
    while(randomStateOut.x[2] > M_PI)
        randomStateOut.x[2] -= 2.0*M_PI;
    while(randomStateOut.x[2] < -M_PI)
        randomStateOut.x[2] += 2.0*M_PI;
    
    //cout<<"sampleState"<<endl;
    if (IsInCollision (randomStateOut.x))
        return 0;

    return 1;
}


int System::sampleGoalState (State &randomStateOut) {

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



double System::extend_dubins_spheres (double x_s1, double y_s1, double t_s1, 
        double x_s2, double y_s2, double t_s2, int comb_no, 
        bool check_obstacles, bool return_trajectory,
        bool& fully_extends, double*& end_state, list<double*>* trajectory, list<float> &control) {

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

    if (distance > 2 * turning_radius) {  // disks do not intersect 

        double t_balls = acos (2 * turning_radius / distance);


        switch (comb_no) {
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

    else { // disks are intersecting

        switch (comb_no) {
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

    double total_distance_travel = (t_increment_s1 + t_increment_s2) * turning_radius  + distance;
    //double cost_map_cost = 0;
    fully_extends = 0;

    if (check_obstacles) {

        // Generate states/inputs
        double del_d = delta_distance;
        double del_t = del_d/turning_radius;

        double t_inc_curr = 0.0;

        double state_curr[3] = {0};
        // double input_curr[2];

        while (t_inc_curr < t_increment_s1) {
            double t_inc_rel = del_t;
            t_inc_curr += del_t;
            if (t_inc_curr > t_increment_s1) {
                t_inc_rel -= t_inc_curr - t_increment_s1;
                t_inc_curr = t_increment_s1;
            }

            state_curr[0] = x_s1 + turning_radius * cos (direction_s1 * t_inc_curr + t_s1);
            state_curr[1] = y_s1 + turning_radius * sin (direction_s1 * t_inc_curr + t_s1);
            state_curr[2] = direction_s1 * t_inc_curr + t_s1 + ( (direction_s1 == 1) ? M_PI_2 : 3.0*M_PI_2);

            while (state_curr[2] < -M_PI)
                state_curr[2] += 2.0 * M_PI;
            while (state_curr[2] > M_PI)
                state_curr[2] -= 2.0*M_PI;

            // input_curr[0] = t_inc_rel * turning_radius;
            // input_curr[1] = ( (comb_no == 1) || (comb_no == 3) ) ? -1 : 1;

            // check for collision
            if (IsInCollision (state_curr))
                return -2.0;

            if (trajectory) {
                double *state_new = new double[3];
                for (int i = 0; i < 3; i++) 
                    state_new[i] = state_curr[i];
                
                trajectory->push_front(state_new);

                // populate controls here
                control.push_front (direction_s1*turning_radius);
            }

            if (t_inc_curr * turning_radius > distance_limit)  {

                fully_extends = false;

                for (int i = 0; i < 3; i++)
                    end_state[i] = state_curr[i];

                return total_distance_travel;
            }
        }

        double d_inc_curr = 0.0;
        while (d_inc_curr < distance) {
            double d_inc_rel = del_d;
            d_inc_curr += del_d;
            if (d_inc_curr > distance) {
                d_inc_rel -= d_inc_curr - distance;
                d_inc_curr = distance;
            }

            state_curr[0] = (x_end - x_start) * d_inc_curr / distance + x_start; 
            state_curr[1] = (y_end - y_start) * d_inc_curr / distance + y_start; 
            state_curr[2] = direction_s1 * t_inc_curr + t_s1 + ( (direction_s1 == 1) ? M_PI_2 : 3.0*M_PI_2);
            
            while (state_curr[2] < -M_PI)
                state_curr[2] += 2.0 * M_PI;
            while (state_curr[2] > M_PI)
                state_curr[2] -= 2.0*M_PI;

            // input_curr[0] = d_inc_rel;
            // input_curr[1] = 0.0;

            // check for collision
            if (IsInCollision (state_curr))
                return -2.0;

            if (trajectory) {
                double *state_new = new double [3];
                for (int i = 0; i < 3; i++) 
                    state_new[i] = state_curr[i];
                trajectory->push_front(state_new);

                // populate controls here
                control.push_front (0);
            }

            if (t_inc_curr * turning_radius + d_inc_curr > distance_limit) {

                fully_extends = false;

                for (int i = 0; i < 3; i++)
                    end_state[i] = state_curr[i];

                return total_distance_travel;
            }
        }

        double t_inc_curr_prev = t_inc_curr;
        t_inc_curr = 0.0;
        while (t_inc_curr < t_increment_s2) {
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
            
            while (state_curr[2] < -M_PI)
                state_curr[2] += 2.0 * M_PI;
            while (state_curr[2] > M_PI)
                state_curr[2] -= 2.0*M_PI;

            // input_curr[0] = t_inc_rel * turning_radius;
            // input_curr[1] = ( (comb_no == 2) || (comb_no == 3) ) ? -1 : 1;

            // check for collision
            if (IsInCollision (state_curr))
                return -2.0;

            if (trajectory) {
                double *state_new = new double [3];
                for (int i = 0; i < 3; i++) 
                    state_new[i] = state_curr[i];
                trajectory->push_front(state_new);
                control.push_front(turning_radius * direction_s2);
            }

            if ((t_inc_curr_prev + t_inc_curr) * turning_radius + d_inc_curr > distance_limit) {

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



double 
System::extend_dubins_all (double state_ini[3], double state_fin[3],
        bool check_obstacles, bool return_trajectory,
        bool &fully_extends, double*& end_state, list<double*>* trajectory, list<float> &control) {

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
            exact_connection[0], end_state, NULL, control);
    times[1] = extend_dubins_spheres (si_right[0], si_right[1], si_right[2], 
            sf_left[0], sf_left[1], sf_left[2], 2,
            false, false,
            exact_connection[1], end_state, NULL, control);
    times[2] = extend_dubins_spheres (si_left[0], si_left[1], si_left[2], 
            sf_left[0], sf_left[1], sf_left[2], 3, 
            false, false,
            exact_connection[2], end_state, NULL, control);
    times[3] = extend_dubins_spheres (si_right[0], si_right[1], si_right[2], 
            sf_right[0], sf_right[1], sf_right[2], 4, 
            false, false,
            exact_connection[3], end_state, NULL, control);

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
                    fully_extends, end_state, trajectory, control);
            return res;

        case 2:
            res = extend_dubins_spheres (si_right[0], si_right[1], si_right[2], 
                    sf_left[0], sf_left[1], sf_left[2], 2, 
                    true, return_trajectory,
                    fully_extends, end_state, trajectory, control);
            return res;

        case 3:
            res = extend_dubins_spheres (si_left[0], si_left[1], si_left[2], 
                    sf_left[0], sf_left[1], sf_left[2], 3, 
                    true, return_trajectory,
                    fully_extends, end_state, trajectory, control);
            return res;

        case 4:
            res = extend_dubins_spheres (si_right[0], si_right[1], si_right[2], 
                    sf_right[0], sf_right[1], sf_right[2], 4, 
                    true, return_trajectory,
                    fully_extends, end_state, trajectory, control);
            return res;

        case -1:
        default:
            return -1.0;
    }
}


int System::extendTo (State &stateFromIn, State &stateTowardsIn, 
        Trajectory &trajectoryOut, bool &exactConnectionOut, list<float> &controlOut) {

    double *end_state;
    end_state = new double [3];

    double time = extend_dubins_all (stateFromIn.x, stateTowardsIn.x, 
            true, false, 
            exactConnectionOut, end_state, NULL, controlOut);
    if (time < 0.0) {
        delete [] end_state;
        return 0;
    }

    while (end_state[2] < -M_PI)
        end_state[2] += 2.0 * M_PI;
    while (end_state[2] > M_PI)
        end_state[2] -= 2.0 * M_PI;

    for (int i = 0; i < 3; i++) {
        trajectoryOut.endState.x[i] = end_state[i];
    }

    trajectoryOut.totalVariation = time;

    delete [] end_state;

    return 1;
}


double System::evaluateExtensionCost (State &stateFromIn, State &stateTowardsIn, bool &exactConnectionOut) {

    double *end_state;

    list<float> tmp_control;
    double time = extend_dubins_all (stateFromIn.x, stateTowardsIn.x, 
            false, false, 
            exactConnectionOut, end_state, NULL, tmp_control);
    if (time < 0.0) 
        return DBL_MAX;

    return time;
}


int System::getTrajectory (State& stateFromIn, State& stateToIn, list<double*>& trajectoryOut, list<float>& controlOut) {

    double *end_state;
    end_state = new double[3];

    bool exactConnectionOut = false;

    double time = extend_dubins_all (stateFromIn.x, stateToIn.x, 
            true, true, 
            exactConnectionOut, end_state, &trajectoryOut, controlOut);

    delete [] end_state;

    if (time <= 0.0) {
        return 0;
    }

    if (exactConnectionOut == false) 
        return 0;

    trajectoryOut.reverse();

    return 1;
}



double System::evaluateCostToGo (State& stateIn) 
{
    double size_x = regionGoal.size[0];
    double size_y = regionGoal.size[1];
    double radius = sqrt(size_x*size_x + size_y*size_y);

    double dist_x = stateIn[0] - regionGoal.center[0];
    double dist_y = stateIn[1] - regionGoal.center[1];
    double dist = sqrt (dist_x*dist_x + dist_y*dist_y);

    return dist - radius;

}

