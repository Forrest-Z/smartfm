/*! 
 * @file 
 * @brief Minimum-violation RRT* (MVRRT*) algorithm and supporting classes. 
 */

#ifndef DUBINSCAR_MVRRTS_HPP
#define	DUBINSCAR_MVRRTS_HPP

// Header files: ROS
#include <ros/ros.h>

// Header files: Custom [Luis]
#include "Defs_Nodes_and_Connections.hpp"

/** Nodes for both MP-Blocks K-D trees and Dubin's car MVRRT* trees. */

typedef DubinsTraits::Node                              Node_t;

/** MP-Blocks K-D tree. */

typedef mpblocks::kd_tree::Tree < DubinsTraits >        Tree_t;

/** MP-Blocks K-D tree Dubin's car scaled boxes search object. */

typedef mpblocks::kd_tree::DubinsBox < DubinsTraits >   DC_SB_Search_t;

/** 
 * Minimum-violation RRT* (MVRRT*) algorithm. 
 */

class DubinsCar_MVRRTs {
    
public:
    
    /** Floating-point number (either float or double). */
    
    typedef Node_t::Format_t                        Format_t;
    
    /** Vector in R^2. */
    
    typedef Node_t::Point_t                         Point_t;
    
    /** Set of candidate connections. */
    
    typedef std::vector < CandidateConnection * >   Set_of_Cand_Conn_t;
    
private:
    
    // ----------------------------------------------------------------------------------------------
    // MEMBERS RELATED TO THE MOTION PLANNING PROBLEM
    // ----------------------------------------------------------------------------------------------
    
    /** Dubin's car. */
    
    const DubinsCar&        car;
    
    /** MP-Blocks 2-D tree Dubin's car scaled boxes search object. */
    
    DC_SB_Search_t          dubcar_box;
    
    /** Root node. */
    
    const Node_t * const    root;
    
    /** Goal region. */
    
    const DubRegion&        goal_region;
    
    // ----------------------------------------------------------------------------------------------
    // MEMBERS RELATED TO THE TREES THEMSELVES
    // ----------------------------------------------------------------------------------------------
    
    /** MP-Blocks 2-D tree. */
    
    Tree_t                  jB_tree;
    
    /** Set of all nodes in the tree. */
    
    std::list < Node_t * >  node_store;
    
    /** Best node found so far. */
    
    const Node_t *          best_node;
    
    /** Best trajectory found so far. */
    
    trajectory_t            best_traj;
    
    /** See DubinsCar_MVRRTs::set_sampling_rates. */
    
    format_t                rate_from_global;
    
    /** See DubinsCar_MVRRTs::set_sampling_rates. */
    
    format_t                heuristic_rate_global;
    
    /** See DubinsCar_MVRRTs::set_sampling_rates. */
    
    format_t                heuristic_rate_local;
    
    // ----------------------------------------------------------------------------------------------
    // MEMBERS RELATED TO THE STATE SAMPLED AT EACH ITERATION
    // ----------------------------------------------------------------------------------------------
    
    /** Newly sampled node. */
    
    Node_t *                v_new;
    
    /** State of newly sampled node. */
    
    Format_t                z_new [3];
    
    /** Unit vector parallel to the velocity vector of the newly sampled node. */
    
    Point_t                 z_new_u_vec;
    
    /** Set of either candidate parents or candidate children. */
    
    Set_of_Cand_Conn_t      z_new_Cand_Conn;
    
public:
    
    // ----------------------------------------------------------------------------------------------
    // CONSTRUCTOR AND DESTRUCTOR
    // ----------------------------------------------------------------------------------------------
    
    /** Copies arguments, sets up Dubin's car scaled boxes search object and initializes 
     * all other members to trivial values. 
     * 
     * @param car_          Dubin's car object. 
     * 
     * @param root_state_   State (in R^2 cross S^1) of root node of MVRRT* tree. 
     * 
     * @param goal_region_  Goal region. */
    
    DubinsCar_MVRRTs( const DubinsCar& car_, 
            
                      const format_t * root_state_, 
            
                      const DubRegion& goal_region_);
    
    /** De-allocates set of candidate connections and set of all nodes in the trees. */
    
    ~DubinsCar_MVRRTs();
    
    // ----------------------------------------------------------------------------------------------
    // PUBLIC METHODS
    // ----------------------------------------------------------------------------------------------
    
    /** Retrieves best trajectory. 
     * 
     * @return Constant reference to the best trajectory from the root state to a goal state. Note 
     * that if there is no current best trajectory the returned reference points to an empty 
     * trajectory object. */
    
    const trajectory_t& get_best_traj();
    
    /** Retrieves number of nodes. 
     * 
     * @return Number of nodes in both of the trees. */
    
    unsigned int get_num_vertices() const;
    
    /** Checks whether a feasible solution has been found. 
     * 
     * @return True, if the algorithm has found a feasible trajectory to the goal region; 
     * false, otherwise. */
    
    bool has_found_feasible_solution() const;
    
    /** Samples a state from the obstacle-free space, and then attempts to connect to a 
     * parent node and to rewire (if a parent is found). 
     * 
     * More precisely, it executes the following actions: 
     * 
     * [+] Samples states until a collision-free state is found. 
     * 
     * [+] Attempts to connect the newly sampled state to a parent node. If the connection 
     * succeeds, a new node is allocated and inserted into both the K-D tree and the MVRRT* tree. 
     * 
     * [+] If a new node was allocated, then it attempts to rewire the potential children 
     * of the newly sampled node. 
     * 
     * @return True if the sample state was connected to the tree. */
    
    bool iteration();
    
    /** Prints level of unsafety of current best trajectory. */
    
    void print_LUS_of_best_trajectory() const;
    
    /** Publishes MVRRT* tree. 
     * 
     * @param advertiser_vertices   ROS publisher object tasked with advertising the ROS topic 
     * for the nodes of the tree. 
     * 
     * @param advertiser_edges      ROS publisher object tasked with advertising the ROS topic 
     * for the trajectories between nodes of the tree. */
    
    void publish_tree( const ros::Publisher& advertiser_vertices, 
    
                       const ros::Publisher& advertiser_edges) const;
    
    /** Sets tuning parameters of Dubin's car scaled boxes search object. 
     * 
     * @param semi_theta_scaling_factor     Factor by which the length of the semi-theta axis is 
     * to be scaled up. */
    
    void set_DCSB_search_object_params( const Format_t semi_theta_scaling_factor);
    
    /** Sets sampling rates. 
     * 
     * @param rate_from_global_             Probability that sample is drawn from global map. 
     * Note that it is equal to one minus the probability that a sample is drawn from the local map. 
     * 
     * @param heuristic_rate_global_        Probability that state is heuristically sampled, 
     * conditional on the event that it was sampled from the global map.
     * 
     * @param heuristic_rate_local_         Probability that state is heuristically sampled, 
     * conditional on the event that it was sampled from the local map. */
    
    void set_sampling_rates( const format_t rate_from_global_, 
    
                             const format_t heuristic_rate_global_, 
                             
                             const format_t heuristic_rate_local_);
    
    // ----------------------------------------------------------------------------------------------
    // PRIVATE METHODS
    // ----------------------------------------------------------------------------------------------
    
private:
    
    /** Convenience method.
     * 
     * De-allocates all objects in DubinsCar_MVRRTs::z_new_Cand_Conn before clearing the container. */
    
    void clear_z_new_Cand_Conn();
    
    /** Convenience method.
     * 
     * Filters candidate parents or candidate children by copying to the set of 
     * candidate connections only those nodes in the R^2 projection of the box whose 
     * angular coordinates differ from those of the newly-sampled state by no more 
     * than the semi-theta axis of the box. */
    
    void collect_candidates( const DC_SB_Search_t::List_t& box_of_Cand_Conn);
    
    /** Attempts to connect the newly sampled state to a parent. 
     * 
     * This method computes the level of unsafety of a trajectory at the same time 
     * when the trajectory is discretized and checked for collisions and for 
     * trace-inclusivity. */
    
    bool find_parent_exactly();
    
    /** Attempts to rewire the candidate children of the newly sampled state. 
     * 
     * This method computes the level of unsafety of a trajectory at the same time 
     * when the trajectory is discretized and checked for collisions and for 
     * trace-inclusivity. */
    
    void rewire_nodes_exactly();
    
    /** Samples a state from the collision-free space of either one of the local maps, according 
     * to the sampling rates. */
    
    void sample_state();
    
    /** Checks whether the given node has a smaller level of unsafety than that of 
     * the best node found so far, and updates the best node if necessary. 
     * 
     * @param node              Node already inserted into both of the trees. */
    
    void update_best_node( const Node_t * node);
    
    /** Recursively updates the levels of unsafety of the nodes in the subtree rooted at 
     * the given one (starting with said node). 
     * 
     * In addition, this method also compares the level of unsafety of each node with that 
     * of the current best node, and updates the best node if necessary. 
     * 
     * @param root_of_subtree   Root of subtree. */
    
    void update_LUS( Node_t * root_of_subtree);
    
};

#endif	/* DUBINSCAR_MVRRTS_HPP */
