/*! 
 * @file 
 * @brief Collision checking and model checking for the Dubin's Car. 
 */

#ifndef DUBINSCAR_HPP
#define	DUBINSCAR_HPP

// Header files: Custom [Luis]
#include "Maps.hpp"
#include "Paths_and_Regions.hpp"

/** Pointer to transition labeling function. 
 * 
 * @param label_start   Subset of atomic propositions that hold at starting state. 
 * 
 * @param label_endPt   Subset of atomic propositions that hold at ending state. 
 * 
 * @return Subset of atomic propositions that hold along a trace inclusive trajectory 
 * between the aforementioned states. */

typedef Subset_of_Sigma (* ptr_TLF) ( const Subset_of_Sigma label_start, 
        
                                      const Subset_of_Sigma label_endPt );

/** 
 * Generic trajectory of a dynamical system (not necessarily that of a Dubin's car). */

typedef std::list < format_t * > trajectory_t;

/** 
 * Dubin's Car. 
 * 
 * Handles all methods particular to the Dubin's Car dynamical system, such as optimal 
 * trajectory computation and discretization, collision checking, and trace-inclusivity checking. 
 */

class DubinsCar {
    
public:
    
    /** Turning radius of car. */
    
    const format_t          turn_radius;
    
    /** Discretization step size (for collision checking and visualization). */
    
    const format_t          delta_x;
    
    /** Current global map grid. */
    
    const Map_Global&       map_global;
    
    /** Current local map grid. */
    
    const Map_Local&        map_local;
    
private:
    
    /** See DubinsCar::DubinsCar. */
    
    const Auto_AB&          auto_AB;
    
    /** See DubinsCar::DubinsCar. */
    
    ptr_TLF                 trans_lab_fun;
    
public:
    
    /** Copies arguments. 
     * 
     * @param turn_radius_ See      DubinsCar::turn_radius. 
     * 
     * @param delta_x See           DubinsCar::delta_x. 
     * 
     * @param map_global_ See       DubinsCar::map_global. 
     * 
     * @param map_local_ See        DubinsCar::map_local. 
     * 
     * @param auto_AB_              Automaton A-bar. 
     * See Reyes, Chaudhari et al. CDC'13 for more information. 
     * 
     * @param trans_lab_fun_        Transition labeling function. 
     * This function maps the pair of atomic propositions holding at the endpoint states of a 
     * transition to the atomic proposition corresponding to the transition. */
    
    DubinsCar( const format_t       turn_radius_, 
            
               const format_t       delta_x_, 
            
               const Map_Global&    map_global_, 
            
               const Map_Local&     map_local_, 
            
               const Auto_AB&       auto_AB_, 
            
               ptr_TLF              trans_lab_fun_ = NULL);
    
    /** Does nothing. */
    
    ~DubinsCar();
    
public:
    
    /** Deallocates all states in trajectory. 
     * 
     * @param traj      Object to be destroyed. */
    
    void clear_traj( trajectory_t& traj) const;
    
    /** Computes the time-optimal path between the starting and ending states. 
     * 
     * @param start     Starting state (in R^2 cross S^1). 
     * 
     * @param endSt     Ending state (in R^2 cross S^1). 
     * 
     * @param path      Object where the time-optimal path will be written. */
    
    void connect( const format_t * start, 
    
                  const format_t * endSt, 
                  
                  DubPath& path) const;
    
    /** Discretizes Dubin's car path to form a trajectory. 
     * 
     * @param path              Object containing the Dubin's path to be discretized. 
     * 
     * @param traj              Object where the discretized trajectory will be written. 
     * 
     * @param level_of_traj     Object where the level of unsafety will be written. 
     * 
     * @param checking          True, if the trajectory is to be checked for collisions and 
     * for trace-inclusivity while it is being discretized; false, otherwise. */
    
    void discretize( const DubPath& path, 
    
                     trajectory_t& traj, 
                     
                     Level_of_US& level_of_traj, 
                     
                     const bool checking = true) const;
    
    /** Checks whether a state is in collision. 
     * 
     * @param state     State (in R^2 cross S^1). 
     * 
     * @return True, if the state is in collision; false, otherwise. */
    
    bool is_in_collision( const format_t * state) const;
    
    /** Checks whether trajectory is still collision-free and trace-inclusive. 
     * 
     * @param traj              Object containing the discretized trajectory to be checked. 
     * 
     * @param level_of_traj     Object where the level of unsafety will be written. 
     * 
     * @return True, if trajectory is collision-free and trace inclusive; false, otherwise. 
     * Note that if the trajectory is found to be infeasible, the method takes care of 
     * de-allocating all states in the trajectory so that it can be readily destroyed. */
    
    bool is_feasible( trajectory_t& traj, Level_of_US& level_of_traj) const;
    
private:
    
    /** Convenience method. 
     * 
     * @param state_curr            Current state. 
     * 
     * @param state_curr_local      Array where the current state in local coordinates can be written. 
     * 
     * @param state_prev            Previous state along the trajectory. 
     * 
     * @param lab_curr              Label (subset of atomic propositions) of current state. 
     * 
     * @param lab_prev              Label (subset of atomic propositions) of previous state. 
     * 
     * @param has_changed_label     True, if there has been a discontinuity of the label along the 
     * trajectory; false, otherwise. 
     * 
     * @param level_of_traj         Object containing the level of unsafety of the trajectory. 
     * 
     * @param traj                  Object containing the discretized trajectory. 
     * 
     * @return True, if the current state is collision-free, and if (a) its label is the same as that 
     * of the previous state along the trajectory or (b) its label is different from that of the 
     * previous state, but there had not been a discontinuity of the label along the trajectory 
     * (in this case, the boolean flag has_changed_label is changed to true); false, otherwise. 
     * Note that when true is returned, the level of unsafety of the trajectory is increased by the 
     * amount corresponding to the transition between the previous and current states. If false is 
     * returned, the trajectory object is cleared. */
    
    bool is_collision_free_and_trace_inclusive( format_t * state_curr, 
    
                                                format_t * state_curr_local, 
                                                
                                                format_t * state_prev, 
            
                                                Subset_of_Sigma& lab_curr, 
    
                                                Subset_of_Sigma& lab_prev, 
                                                
                                                bool& has_changed_label, 
                                                
                                                Level_of_US& level_of_traj, 
                                                
                                                trajectory_t& traj) const;
    
};

#endif	/* DUBINSCAR_HPP */
