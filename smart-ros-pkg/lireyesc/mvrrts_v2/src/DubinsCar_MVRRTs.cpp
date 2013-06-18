// Header files: Standard C++
#include <algorithm>

// Header files: Off-the-shelf libraries [Eigen]
#include <eigen3/Eigen/Core>

// Header files: ROS
#include <sensor_msgs/PointCloud.h>

// Header files: Custom [Luis]
#include "DubinsCar_MVRRTs.hpp"
#include "Defs_Utilities.hpp"

// ----------------------------------------------------------------------------------------
// CONSTRUCTOR AND DESTRUCTOR
// ----------------------------------------------------------------------------------------

DubinsCar_MVRRTs::DubinsCar_MVRRTs( const DubinsCar& car_, 
        
                                    const format_t * root_state_, 
        
                                    const DubRegion& goal_region_) : 
                                    
    // Sets up Dubin's car and Dubin's car scaled boxes search object

    car (car_), dubcar_box ( car_.turn_radius, car_.map_local.area()), 
        
    // Sets up root node
        
    root ( new Node_t ( root_state_[0], root_state_[1], root_state_[2], 
        
                        NULL, DubPath (), new trajectory_t (), Level_of_US (0.0) ) ), 
        
    // Sets up goal region
        
    goal_region(goal_region_), 
        
    // Sets up best node and best trajectory
        
    best_node (NULL), best_traj(), 
        
    // Sets up sampling rates
        
    rate_from_global (0.0), heuristic_rate_global (0.0), heuristic_rate_local (0.0) {
    
    // Inserts root node into storage list and into K-D tree
    
    node_store.push_back( const_cast < Node_t * > (root) );
    
    jB_tree.insert( const_cast < Node_t * > (root) );
    
}

DubinsCar_MVRRTs::~DubinsCar_MVRRTs() {
    
    clear_z_new_Cand_Conn();
    
    for ( auto& iter : node_store ) { delete iter; }
    
}

// ----------------------------------------------------------------------------------------
// METHODS : PUBLIC
// ----------------------------------------------------------------------------------------

const trajectory_t& DubinsCar_MVRRTs::get_best_traj() {
    
    // Clears current best trajectory and copies pointer to current best node
    
    best_traj.clear();
    
    const Node_t * current_node = best_node;
    
    // Iteratively parses the nodes of the best trajectory, starting from the best node and 
    // working its way backwards to the root node (note that if there is no current best node 
    // then this block will not execute)
    
    while ( current_node ) {
        
        best_traj.insert( best_traj.begin(), 
                
                          current_node -> traj_from_parent -> begin(), 
                
                          current_node -> traj_from_parent -> end() );
        
        current_node = current_node -> MVRRTs_parent;
        
    }
    
    // Returns the (possibly empty) current best trajectory of MVRRT* tree
    
    return best_traj;
    
}

unsigned int DubinsCar_MVRRTs::get_num_vertices() const { return (unsigned int) jB_tree.size(); }

bool DubinsCar_MVRRTs::has_found_feasible_solution() const { return (bool) best_node; }

bool DubinsCar_MVRRTs::iteration() {
    
    // Samples a state from the collision-free space
    
    sample_state();
    
    // Sets up the box for the candidate parents
    
    dubcar_box.reset( Point_t ( z_new[0], z_new[1]), z_new[2], jB_tree.size(), 'B');
    
    // Runs range search on the K-D tree for the box of parents
    
    jB_tree.findRange(dubcar_box);
    
    // Retrieves set of points inside the R^2 projection of the box of parents and then 
    // filters out those nodes with theta coordinates outside the angular range of the box
    
    const DC_SB_Search_t::List_t& inside_box_back = dubcar_box.get_result();
    
    collect_candidates(inside_box_back);
    
    // Attempts to assign a parent node to the newly sampled state
    
    if ( find_parent_exactly() ) {
        
        // Sets up the box for the candidate children
        
        dubcar_box.move_forward();
        
        // Runs range search on the K-D tree for the box of children
        
        jB_tree.findRange(dubcar_box);
        
        // Retrieves set of points inside the R^2 projection of the box of children and then 
        // filters out those nodes with theta coordinates outside the angular range of the box
        
        const DC_SB_Search_t::List_t& inside_box_front = dubcar_box.get_result();
        
        collect_candidates(inside_box_front);
        
        // Attempts to rewire the candidate children, if the rewiring improves their levels of 
        // unsafety
        
        rewire_nodes_exactly();
        
        // Regardless of whether re-wirings occurred, terminates procedure with flag of success
        
        return true;
        
    }
    
    // Otherwise terminates procedure with flag of failure
    
    else { return false; }
    
}

void DubinsCar_MVRRTs::print_LUS_of_best_trajectory() const {
    
    if (best_node) { (best_node -> level_from_root).print(); }
    
}

void DubinsCar_MVRRTs::publish_tree( const ros::Publisher& advertiser_nodes, 
        
                                     const ros::Publisher& advertiser_edges) const {
    
    // Declares and initializes array of node states
    
    sensor_msgs::PointCloud node_states;
    
    node_states.header.stamp = ros::Time::now();
    
    node_states.header.frame_id = "/map";
    
    // Declares and initializes array of trajectories from parent node
    
    sensor_msgs::PointCloud trajectories_from_parents;
    
    trajectories_from_parents.header.stamp = ros::Time::now();
    
    trajectories_from_parents.header.frame_id = "/map";
    
    // Iterates through set of nodes
    
    for ( auto& curr_node : node_store ) {
        
        // Declares and initializes planar coordinates of state of current node
        
        geometry_msgs::Point32 curr_state;
        
        curr_state.x = (geometry_msgs::Point32::_x_type) curr_node -> get_point_x();
        
        curr_state.y = (geometry_msgs::Point32::_y_type) curr_node -> get_point_y();
        
        curr_state.z = 0.0;
        
        // Pushes planar coordinates of state of current node into array
        
        node_states.points.push_back(curr_state);
        
        // If node has a parent (always the case except for the root node)
        
        if ( curr_node -> MVRRTs_parent ) {
            
            // Iterates through states along trajectory from parent
            
            for ( auto& state_along_traj : *(curr_node -> traj_from_parent) ) {
                
                // Declares and initializes planar coordinates of state
                
                geometry_msgs::Point32 state_along_traj_COPY;
        
                state_along_traj_COPY.x = (geometry_msgs::Point32::_x_type) state_along_traj[0];

                state_along_traj_COPY.y = (geometry_msgs::Point32::_y_type) state_along_traj[1];

                state_along_traj_COPY.z = 0.0;
                
                // Pushes planar coordinates of state of current node into array
                
                trajectories_from_parents.points.push_back(state_along_traj_COPY);
                
            }
            
        }
        
    }
    
    // Publishes both the array of node states and the array of trajectories from parent node
    
    advertiser_nodes.publish(node_states);
    
    advertiser_edges.publish(trajectories_from_parents);
    
}

void DubinsCar_MVRRTs::set_DCSB_search_object_params( const Format_t semi_theta_scaling_factor) {
    
    dubcar_box.set_semi_theta_scaling_factor(semi_theta_scaling_factor);
    
}

void DubinsCar_MVRRTs::set_sampling_rates( const format_t rate_from_global_, 
        
                                           const format_t heuristic_rate_global_, 
        
                                           const format_t heuristic_rate_local_) {
    
    rate_from_global        = rate_from_global_;
    
    heuristic_rate_global   = heuristic_rate_global_;
    
    heuristic_rate_local    = heuristic_rate_local_;
    
}

// ----------------------------------------------------------------------------------------
// METHODS : PRIVATE
// ----------------------------------------------------------------------------------------

void DubinsCar_MVRRTs::clear_z_new_Cand_Conn() {
    
    // Iterates through set of candidate connections and de-allocates each object
    for ( USI k = 0; k < z_new_Cand_Conn.size(); k++) { delete z_new_Cand_Conn[k]; }
    
    // Clears container
    z_new_Cand_Conn.clear();
    
}

void DubinsCar_MVRRTs::collect_candidates( const DC_SB_Search_t::List_t& box_of_Cand_Conn) {
    
    // Computes cosine lower bound
    Format_t z_new_cosine_LB = (Format_t) cos( dubcar_box.get_len_semi_theta_axis() );
    
    // Cleans and clears set of candidate connections
    clear_z_new_Cand_Conn();
    
    // Iterates through set of nodes inside the R^2 projection of the box
    for ( USI k = 0; k < box_of_Cand_Conn.size(); k++) {
        
        // If dot product of unit vectors meets or exceeds the cosine lower bound
        if ( z_new_u_vec.dot( box_of_Cand_Conn[k] -> u_vec ) >= z_new_cosine_LB ) {
            
            // Copy pointer to node into set of candidates
            z_new_Cand_Conn.push_back( new CandidateConnection( box_of_Cand_Conn[k] ) );
            
        }
        
    }
    
}

bool DubinsCar_MVRRTs::find_parent_exactly() {
    
    // Iterates through set of candidate parents
    
    for ( USI k = 0; k < z_new_Cand_Conn.size(); k++) {
        
        // Declares reference to candidate parent
        
        CandidateConnection& cand = *( z_new_Cand_Conn[k] );
        
        // Computes Dubin's path from candidate parent to newly sampled state
        
        car.connect( cand.state, z_new, cand.path);
        
        // Allocates trajectory object
        
        cand.traj = new trajectory_t ();
        
        // Computes discretization of trajectory from candidate parent to newly sampled state
        
        car.discretize( cand.path, *(cand.traj), cand.level_connection);
        
        // If trajectory is collision-free and trace-inclusive then compute level 
        // of unsafety of trajectory from root node to newly sampled state 
        // through this candidate parent
        
        if ( ! cand.traj -> empty() ) {
            
            cand.level_from_root += cand.level_connection;
            
            cand.level_from_root += cand.node -> level_from_root;
            
        }
        
        // Otherwise, set level of unsafety of trajectory from root to infinity
        
        else { cand.level_from_root = std::numeric_limits <Format_t> :: max(); }
        
    }
    
    // Sorts set of candidate parents according to level of unsafety
    
    std::sort( z_new_Cand_Conn.begin(), z_new_Cand_Conn.end(), compare_Cand_Conn);
    
    // Initializes pointer to newly sampled node
    
    v_new = NULL; 
    
    // Iterates through set of candidate parents (again)
    
    for ( USI k = 0; k < z_new_Cand_Conn.size(); k++) {
        
        // Declares reference to candidate parent
        
        CandidateConnection& cand = *( z_new_Cand_Conn[k] );
        
        // If no parent has been found so far, and if the trajectory from the root 
        // to the newly sampled state through the current candidate parent has a 
        // finite level of unsafety
        
        if ( !v_new && ( cand.level_from_root < std::numeric_limits <Format_t> :: max() ) ) {
            
            // Declares and allocates new node
            
            v_new = new Node_t ( z_new[0], z_new[1], z_new[2], 
                    
                                 cand.node, cand.path, cand.traj, cand.level_connection);
            
            // Inserts node into storage list and K-D tree
            
            node_store.push_back(v_new);
            
            jB_tree.insert(v_new);
            
            // Checks if new sample is best node and updates best node if necessary
            
            update_best_node(v_new);
            
        }
        
        // Otherwise, cleans, clears and de-allocates trajectory object
        
        else {
            
            car.clear_traj( *(cand.traj) );
            
            delete cand.traj;
        
        }
        
    }
    
    // Returns true if a parent node was found
    
    return (bool) v_new;
    
}

void DubinsCar_MVRRTs::rewire_nodes_exactly() {
    
    // Iterates through set of candidate children
    
    for ( USI k = 0; k < z_new_Cand_Conn.size(); k++) {
        
        // Declares reference to candidate child
        
        CandidateConnection& cand = *( z_new_Cand_Conn[k] );
        
        // Computes connection from newly sampled node to candidate child
        
        car.connect( z_new, cand.state, cand.path);
        
        // Allocates trajectory object
        
        cand.traj = new trajectory_t ();
        
        // Computes discretization of trajectory from new vertex to candidate child
        
        car.discretize( cand.path, *(cand.traj), cand.level_connection);
        
        // If trajectory is collision-free and trace-inclusive
        
        if ( ! cand.traj -> empty() ) {
        
            // Computes nominal level of unsafety of connection 
            // from root to candidate child through new sample
            
            cand.level_from_root += cand.level_connection;
            
            cand.level_from_root += v_new -> level_from_root;

            // If nominal level of unsafety of connection from root to candidate child
            // through new sample is better than the child's current level from root
            
            if ( ( cand.level_from_root ) < ( cand.node -> level_from_root ) ) {
                
                // Sets new sample as parent of candidate child
                
                cand.node -> change_parent_to( v_new, cand.path, cand.traj, cand.level_connection);
                
                // Updates level of unsafety of all nodes in subtree rooted at candidate child
                
                update_LUS(cand.node);
                
            }
            
            // Otherwise, cleans, clears and de-allocates trajectory object
            
            else {
                
                car.clear_traj( *(cand.traj) );
                
                delete cand.traj;
            
            }
            
        }
        
        // Otherwise, de-allocate trajectory object
        
        else { delete cand.traj; }
        
    }
    
}

void DubinsCar_MVRRTs::sample_state() {
    
    // Samples from the global map according to the user-defined rate
    
    if ( uniform() < rate_from_global ) {
        
        car.map_global.sample_free_space( z_new, heuristic_rate_global);
        
    }
    
    else {
        
        Format_t z_new_local [3];
        
        car.map_local.sample_free_space( z_new_local, heuristic_rate_local);
        
        car.map_local.transform_local_to_global( z_new_local, z_new);
        
    }
    
    // Computes unit vector in the direction of the velocity vector
    
    z_new_u_vec << (Format_t) cos(z_new[2]), (Format_t) sin(z_new[2]);
    
}

void DubinsCar_MVRRTs::update_best_node( const Node_t * node) {
    
    // If state associated with node is inside goal region
    
    if ( goal_region.intesects( node -> get_point_x(), node -> get_point_y(), node -> theta) ) {
        
        // If either there is no current best node or there is a current best node but 
        // given node has level of unsafety smaller than that of current best node
        
        if ( !best_node || ( (node -> level_from_root) < (best_node -> level_from_root) ) ) {
            
            best_node = node;
            
        }
        
    }
    
}

void DubinsCar_MVRRTs::update_LUS( Node_t * root_of_subtree) {
    
    // Re-computes level of unsafety from root
    
    root_of_subtree -> level_from_root = root_of_subtree -> MVRRTs_parent -> level_from_root;
    
    root_of_subtree -> level_from_root += root_of_subtree -> level_from_parent;
    
    // Compares root of subtree with current best node and updates if necessary
    
    update_best_node(root_of_subtree);
    
    // Calls this same function on all children of current root of subtree
    
    for ( auto& child : root_of_subtree -> MVRRTS_children ) { update_LUS(child); }
    
}
