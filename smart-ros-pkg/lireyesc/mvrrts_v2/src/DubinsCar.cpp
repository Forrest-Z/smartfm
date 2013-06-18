// Header files: Standard C
#include <cmath>
// Header files: Standard C++
#include <limits>
// Header files: Custom
#include "DubinsCar.hpp"
#include "Defs_Utilities.hpp"

// ----------------------------------------------------------------------------------------
// CONSTRUCTOR AND DESTRUCTOR
// ----------------------------------------------------------------------------------------

DubinsCar::DubinsCar( const format_t        turn_radius_, 

                      const format_t        delta_x_, 
        
                      const Map_Global&     map_global_, 
        
                      const Map_Local&      map_local_, 
        
                      const Auto_AB&        auto_AB_, 
        
                      ptr_TLF               trans_lab_fun_) : 
                      
    turn_radius(turn_radius_), delta_x(delta_x_), 
        
    map_global(map_global_), 
        
    map_local(map_local_), 
        
    auto_AB(auto_AB_), 
        
    trans_lab_fun(trans_lab_fun_) {}

DubinsCar::~DubinsCar() {}

// ----------------------------------------------------------------------------------------
// METHODS : PUBLIC
// ----------------------------------------------------------------------------------------

void DubinsCar::clear_traj( trajectory_t& traj) const {
    
    for ( auto& state : traj ) { delete [] state; }
    
    traj.clear();
    
}

void DubinsCar::connect( const format_t * start, 
        
                         const format_t * endSt, 
        
                         DubPath& path) const {
    
    // Declares inner tangent paths
    
    DubPath path_rsl ( start, endSt, 'R', 'L', turn_radius);
    DubPath path_lsr ( start, endSt, 'L', 'R', turn_radius);
    
    // Declares outer tangent paths
    
    DubPath path_rsr ( start, endSt, 'R', 'R', turn_radius);
    DubPath path_lsl ( start, endSt, 'L', 'L', turn_radius);
    
    // Computes inner and outer tangent paths
    
    DubPath::tangents_inner( path_rsl, path_lsr);
    DubPath::tangents_outer( path_rsr, path_lsl);
    
    // Computes and copies optimal path
    
    path = 
            
    DubPath::arg_min( DubPath::arg_min( DubPath::arg_min( path_rsl, path_lsr), path_rsr), path_lsl);
    
}

void DubinsCar::discretize( const DubPath& path, 
        
                            trajectory_t& traj, 
        
                            Level_of_US& level_of_traj, 
        
                            const bool checking_for_collisions_and_TI) const {
    
    // If path does not exist then throw error
    
    if ( !path.exists ) {
        
        std::cerr << "In line " <<  __LINE__ << " of " << __FILE__ << ": ";
        std::cerr << "NON-EXISTENT path" << std::endl;
        exit(EXIT_FAILURE);
        
    }
    
    // Cleans and clears trajectory and resets level of unsafety to zero
    
    clear_traj(traj); level_of_traj = 0.0;
    
    // Declares and initializes current state (in global map coordinates); 
    // note that the fourth entry of the array is the corresponding control input
    
    format_t * state_curr   = new format_t [4];
    
    state_curr[0]           = path.arc1_EntryPt[0];
    state_curr[1]           = path.arc1_EntryPt[1];
    state_curr[2]           = path.arc1_EntryPt[2];
    
    state_curr[3]           = ( path.arc1_dir == 'R' ? -1.0 : +1.0 ) * path.turn_radius;
    
    // Inserts state and control input into trajectory object
    
    traj.push_back(state_curr);
    
    // Declares and current state (in local map coordinates)
    
    format_t state_curr_local [3];
    
    // Declares and initializes previous state (in global map coordinates)
    
    format_t * state_prev = state_curr;
    
    // Declares and initializes subsets of atomic propositions holding at current state
    
    Subset_of_Sigma lab_curr;
    
    map_global.is_in_collision( state_curr, &lab_curr);
    
    // Declares and initializes subsets of atomic propositions holding at previous state
    
    Subset_of_Sigma lab_prev = lab_curr;
    
    // Declares and initializes boolean flag for trace-inclusivity checking
    
    bool has_changed_label = false;
    
    // Declares angle and vector (temporary) variables
    
    format_t init_theta = 0.0, delta_theta = 0.0, step_theta = 0.0, control_input = 0.0;
    
    format_t v_perp [2] = { 0, 0};
    
    // ------------------------------------------------------------------------
    // FIRST ARC
    // ------------------------------------------------------------------------
    
    // Sets up initial angle and angle step size according to direction of motion 
    // as well as corresponding control input; note that the initial angle is that 
    // of a vector from the entry point into the first arc towards the center of the arc
    
    switch (path.arc1_dir) {
        
        case 'R':
            
            init_theta = path.arc1_EntryPt[2] - M_PI_2;
            
            delta_theta = -1.0 * delta_x / path.turn_radius;
            
            control_input = -path.turn_radius; break;
            
        case 'L':
            
            init_theta = path.arc1_EntryPt[2] + M_PI_2;
            
            delta_theta = +1.0 * delta_x / path.turn_radius;
            
            control_input = +path.turn_radius; break;
            
    }
    
    // Iterates through points along first (circular) arc
    
    for ( step_theta = delta_theta; fabs(step_theta) < path.arc1_len; step_theta += delta_theta) {
        
        // Computes unit vector from state to center
        
        v_perp[0] = (format_t) cos( init_theta + step_theta );
        
        v_perp[1] = (format_t) sin( init_theta + step_theta );
        
        // Allocates and computes coordinates of current state (in global map coordinates); 
        // note that the fourth entry of the array is the corresponding control input
        
        state_curr = new format_t [4];
        
        state_curr[0] = path.arc1_C_ABS[0] - path.turn_radius * v_perp[0];
        
        state_curr[1] = path.arc1_C_ABS[1] - path.turn_radius * v_perp[1];
        
        state_curr[2] = unwrap( path.arc1_EntryPt[2] + step_theta );
        
        state_curr[3] = control_input;
        
        // Inserts state and control input into trajectory object
        
        traj.push_back(state_curr);
        
        // Checks for collisions and for trace-inclusivity
        // (note that if the checks fail then this function takes care of properly de-allocating 
        // all states of the trajectory object)
        
        if ( checking_for_collisions_and_TI && 
                
             !is_collision_free_and_trace_inclusive( state_curr, 
                                                     state_curr_local, 
                                                     state_prev, 
                                                     lab_curr, 
                                                     lab_prev, 
                                                     has_changed_label, 
                                                     level_of_traj, 
                                                     traj) ) { return; }
        
    }
    
    // ------------------------------------------------------------------------
    // STRAIGHT LINE SEGMENT
    // ------------------------------------------------------------------------
    
    // Computes unit vector in direction of straight line segment
    
    format_t v_straight_line_dir [] = 
    
    {   ( path.arc2_EntryPt[0] - path.arc1_Exit_Pt[0] ) / path.straight_len, 
        ( path.arc2_EntryPt[1] - path.arc1_Exit_Pt[1] ) / path.straight_len };
    
    // Computes angle of straight line segment vector
    
    format_t straight_line_dir_theta = atan2( v_straight_line_dir[1], v_straight_line_dir[0]);
    
    // Iterates through points along the straight line segment
    
    for ( format_t step_x = 0; step_x < path.straight_len; step_x += delta_x) {
        
        // Allocates and computes coordinates of current state (in global map coordinates); 
        // note that the fourth entry of the array is the corresponding control input
        
        state_curr = new format_t [4];
        
        state_curr[0] = path.arc1_Exit_Pt[0] + v_straight_line_dir[0] * step_x;
        
        state_curr[1] = path.arc1_Exit_Pt[1] + v_straight_line_dir[1] * step_x;
        
        state_curr[2] = straight_line_dir_theta;
        
        state_curr[3] = 0.0;
        
        // Inserts state and control input into trajectory object
        
        traj.push_back(state_curr);
        
        // Checks for collisions and for trace-inclusivity
        // (note that if the checks fail then this function takes care of properly de-allocating 
        // all states of the trajectory object)
        
        if ( checking_for_collisions_and_TI && 
                
             !is_collision_free_and_trace_inclusive( state_curr, 
                                                     state_curr_local, 
                                                     state_prev, 
                                                     lab_curr, 
                                                     lab_prev, 
                                                     has_changed_label, 
                                                     level_of_traj, 
                                                     traj) ) { return; }
        
    }
    
    // ------------------------------------------------------------------------
    // SECOND ARC
    // ------------------------------------------------------------------------
    
    // Sets up initial angle and angle step size according to direction of motion 
    // as well as corresponding control input; note that the initial angle is that 
    // of a vector from the entry point into the first arc towards the center of the arc
    
    switch (path.arc2_dir) {
        
        case 'R':
            
            init_theta = path.arc2_Exit_Pt[2] - M_PI_2 + path.arc2_len;
            
            delta_theta = -1.0 * delta_x / path.turn_radius;
            
            control_input = -path.turn_radius; break;
            
        case 'L':
            
            init_theta = path.arc2_Exit_Pt[2] + M_PI_2 - path.arc2_len;
            
            delta_theta = +1.0 * delta_x / path.turn_radius;
            
            control_input = +path.turn_radius; break;
            
    }
    
    // Iterates through points along first (circular) arc
    
    for ( step_theta = 0; fabs(step_theta) < path.arc2_len; step_theta += delta_theta) {
        
        // Computes unit vector from state to center
        
        v_perp[0] = (format_t) cos( init_theta + step_theta );
        
        v_perp[1] = (format_t) sin( init_theta + step_theta );
        
        // Allocates and computes coordinates of current state (in global map coordinates); 
        // note that the fourth entry of the array is the corresponding control input
        
        state_curr = new format_t [4];
        
        state_curr[0] = path.arc2_C_ABS[0] - path.turn_radius * v_perp[0];
        
        state_curr[1] = path.arc2_C_ABS[1] - path.turn_radius * v_perp[1];
        
        state_curr[2] = unwrap( straight_line_dir_theta + step_theta );
        
        state_curr[3] = control_input;
        
        // Inserts state and control input into trajectory object
        
        traj.push_back(state_curr);
        
        // Checks for collisions and for trace-inclusivity
        // (note that if the checks fail then this function takes care of properly de-allocating 
        // all states of the trajectory object)
        
        if ( checking_for_collisions_and_TI && 
                
             !is_collision_free_and_trace_inclusive( state_curr, 
                                                     state_curr_local, 
                                                     state_prev, 
                                                     lab_curr, 
                                                     lab_prev, 
                                                     has_changed_label, 
                                                     level_of_traj, 
                                                     traj) ) { return; }
        
    }
    
    // Allocates and computes coordinates of final state (in global map coordinates); 
    // note that the fourth entry of the array is the corresponding control input
    
    state_curr      = new format_t [4];
    
    state_curr[0]   = path.arc2_Exit_Pt[0];
    state_curr[1]   = path.arc2_Exit_Pt[1];
    state_curr[2]   = path.arc2_Exit_Pt[2];
    
    state_curr[3]   = control_input;
    
    // Inserts state and control input into trajectory object
    
    traj.push_back(state_curr);
    
    // Checks for collisions and for trace-inclusivity
    // (note that if the checks fail then this function takes care of properly de-allocating 
    // all states of the trajectory object)
    
    if ( checking_for_collisions_and_TI ) {
        
        is_collision_free_and_trace_inclusive( state_curr, 
                                               state_curr_local, 
                                               state_prev, 
                                               lab_curr, 
                                               lab_prev, 
                                               has_changed_label, 
                                               level_of_traj, 
                                               traj);
        
    }
    
}

bool DubinsCar::is_in_collision( const format_t * state) const {
    
    // Declares and computes state in local map coordinates
    
    format_t state_local [3];
    
    map_local.transform_global_to_local( state, state_local);
    
    // Carries collision check in both local and global maps
    
    if ( map_local.is_in_collision(state_local) || map_global.is_in_collision(state) ) {
        
        return true;
    
    }
    
    else { return false; }
    
}

bool DubinsCar::is_feasible( trajectory_t& traj, Level_of_US& level_of_traj) const {
    
    // If trajectory has less than two states then trow error
    
    if ( traj.empty() || ( ++traj.begin() == traj.end() ) ) {
        
        std::cerr << "In line " <<  __LINE__ << " of " << __FILE__ << ": ";
        std::cerr << "TRAJECTORY has less than two states" << std::endl;
        exit(EXIT_FAILURE);
        
    }
    
    // Resets level of unsafety of trajectory to zero
    
    level_of_traj = 0.0;
    
    // Declares and initializes current state (in global and local map coordinates)
    
    format_t * state_curr = traj.front();
    
    format_t state_curr_local [3];
    
    // Declares and initializes previous state (in global map coordinates)
    
    format_t * state_prev = state_curr;
    
    // Declares and initializes subset of atomic propositions holding at current state
    
    Subset_of_Sigma lab_curr;
    
    map_global.is_in_collision( state_curr, &lab_curr);
    
    // Declares and initializes subset of atomic propositions holding at previous state
    
    Subset_of_Sigma lab_prev = lab_curr;
    
    // Declares and initializes boolean flag for trace-inclusivity checking
    
    bool has_changed_label = false;
    
    // Checks for collisions and for trace-inclusivity
    // (note that if the checks fail then this function takes care of properly de-allocating 
    // all states of the trajectory object)
    
    for ( trajectory_t::iterator traj_iter = ++traj.begin(); traj_iter != traj.end(); ++traj_iter) {
        
        state_curr = *traj_iter;
        
        if ( !is_collision_free_and_trace_inclusive( state_curr, 
                                                     state_curr_local, 
                                                     state_prev, 
                                                     lab_curr, 
                                                     lab_prev, 
                                                     has_changed_label, 
                                                     level_of_traj, 
                                                     traj) ) { return false; }
        
    }
    
    // If this line is reached then trajectory is collision free and trace inclusive
    
    return true;
    
}

// ----------------------------------------------------------------------------------------
// METHODS : PRIVATE
// ----------------------------------------------------------------------------------------

inline bool DubinsCar::is_collision_free_and_trace_inclusive( format_t * state_curr, 
        
                                                              format_t * state_curr_local, 
        
                                                              format_t * state_prev, 
        
                                                              Subset_of_Sigma& lab_curr, 
        
                                                              Subset_of_Sigma& lab_prev, 
        
                                                              bool& has_changed_label, 
        
                                                              Level_of_US& level_of_traj, 
        
                                                              trajectory_t& traj) const {
    
    // Transform state into local map coordinates and reset its label
            
    map_local.transform_global_to_local( state_curr, state_curr_local);

    lab_curr.clear_symbols();
    
    // Carries collision check on both maps and labels state according to global map

    if ( !map_local.is_in_collision(state_curr_local) && 
            
         !map_global.is_in_collision( state_curr, &lab_curr) ) {
        
        // Checks whether state is trace-inclusive with respect to the rest of the trajectory
        
        if ( (lab_curr == lab_prev) || !has_changed_label ) {
            
            // Computes level of unsafety of transition from previous state to current one

            auto_AB.add_LUS( trans_lab_fun( lab_prev, lab_curr), 

                             vec2d_norm( state_prev, state_curr), 
                    
                             level_of_traj);
            
            // Redefines previous state to current one
            
            state_prev = state_curr; lab_prev = lab_curr;
            
            // Checks whether there has been a label discontinuity along trajectory
            
            has_changed_label = has_changed_label || ( lab_curr != lab_prev );

            return true;

        }

    }

    // If this line is reached then either the current state is in collision or the trajectory 
    // is not trace-inclusive

    clear_traj(traj); level_of_traj = std::numeric_limits<format_t>::max();

    return false;
    
}
