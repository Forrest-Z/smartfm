/*! 
 * @file 
 * @brief Nodes for both the MP-Blocks K-D tree and the Dubin's Car MVRRT* tree, as well as 
 * connections between nodes in the MVRRT* tree. 
 */

#ifndef DEF_NODES_AND_CONNECTIONS_HPP
#define	DEF_NODES_AND_CONNECTIONS_HPP

// Header files: Standard C++
#include <set>

// Header files: Custom [MP-Blocks]
#include "mpblocks/kd_tree.h"

// Header files: Custom [Luis]
#include "DubinsBox.h"
#include "DubinsCar.hpp"

/** 
 * Parent class for nodes of MP-Blocks K-D Tree. 
 * 
 * Note that this is NOT the base class for the nodes of the MP-Blocks K-D tree, 
 * but merely an abstract class that allows the user to specify the floating point 
 * type to be used in the tree (either float or double) and the number of dimensions 
 * of the underlying state space. The reason for this is that the MP-Blocks K-D tree 
 * was written according to the Curiously Recursive Template Pattern (CRTP). 
 */

struct DubinsTraits {
    
    /** Floating-point number (either float or double). */
    
    typedef format_t Format_t;
    
    /** Number of dimensions of MP-Blocks K-D tree. */
    
    static const int NDim = 2;
    
    /** 
     * Nodes for both MP-Blocks K-D trees and Dubin's car MVRRT* trees. 
     * 
     * Handles construction (including attachment to a parent node in the MVRRT* tree) 
     * and destruction of the node. In addition, it provides a convenience method to 
     * change the parent node. 
     */
    
    class Node: public mpblocks::kd_tree::Node < DubinsTraits > {
    
    public:

        /** Node for both an MP-Blocks K-D tree and a Dubin's car MVRRT* tree. */

        typedef DubinsTraits::Node          Node_t;

        /** Set of nodes. */

        typedef std::set < Node_t * >       Set_of_Nodes_t;
        
        /** See Node::Node. */

        const Format_t                      theta;

        /** Unit vector parallel to velocity vector of state. */

        const Point_t                       u_vec;

        /** See Node::Node. */

        Node_t *                            MVRRTs_parent;
        
        /** See Node::Node. */
        
        DubPath                             path_from_parent;
        
        /** See Node::Node. */

        const trajectory_t *                traj_from_parent;

        /** See Node::Node. */

        Level_of_US                         level_from_parent;

        /** Level of unsafety of trajectory through parent starting at root and 
         * ending at node. */

        Level_of_US                         level_from_root;

        /** Children nodes. */

        Set_of_Nodes_t                      MVRRTS_children;
        
    public:

        /** Copies state, pointer to parent, pointer to trajectory from parent object, 
         * and level of unsafety of sub-trajectory from parent, and computes level of 
         * unsafety through parent from root to node. 
         * 
         * @param state_x               X-coordinate of state. 
         * 
         * @param state_y               Y-coordinate of state. 
         * 
         * @param state_th              Angular coordinate of state. 
         * 
         * @param MVRRTs_parent         Parent of node in the MVRRT* tree. 
         * 
         * @param path_from_parent      Skeleton of Dubin's path from parent node in 
         * the MVRRT* tree. Note that this member is not a pointer, but an actual object. 
         * 
         * @param traj_from_parent      Pointer to trajectory from parent node in 
         * the MVRRT* tree. Note that node takes ownership of trajectory object. 
         * 
         * @param level_from_parent     Level of unsafety of sub-trajectory starting at 
         * parent and ending at this node. */

        Node( const Format_t state_x, 

              const Format_t state_y, 

              const Format_t state_th, 

              Node_t *                  MVRRTs_parent, 
              
              const DubPath&            path_from_parent, 
              
              const trajectory_t *      traj_from_parent, 

              const Level_of_US&        level_from_parent) : 

              // Copies arguments and calls member constructors

              theta (state_th), u_vec ( cos(state_th), sin(state_th)), 

              MVRRTs_parent (MVRRTs_parent), 
              
              path_from_parent (path_from_parent), 

              traj_from_parent (traj_from_parent), 

              level_from_parent (level_from_parent), 

              level_from_root (level_from_parent) {

            // Sets the planar coordinates

            setPoint( Point_t ( state_x, state_y) );

            // If a parent node was provided (always the case, except for the root node)

            if ( MVRRTs_parent ) {

                // Inserts itself in the set of children of its parent

                MVRRTs_parent -> MVRRTS_children.insert(this);

                // Computes its level of unsafety from root

                level_from_root += MVRRTs_parent -> level_from_root;

            }

        }

        /** De-allocates its trajectory from parent. */

        ~Node() {

            for ( auto& state : *traj_from_parent ) { delete [] state; }

            delete traj_from_parent;

        }

        /** Changes parent node. 
         * 
         * This is a convenience method intended for use when rewiring nodes. 
         * 
         * @param MVRRTs_new_parent     New parent of node in MVRRT* tree. 
         * 
         * @param path_from_new_parent  Skeleteon of Dubin's path from new parent node in 
         * the MVRRT* tree. Note that this member is not a pointer, but an actual object. 
         * 
         * @param traj_from_new_parent  Pointer to trajectory from new parent node in 
         * the MVRRT* tree. Note that node takes ownership of trajectory object. 
         * 
         * @param level_from_new_parent Level of unsafety of sub-trajectory starting at 
         * new parent and ending at node. */

        void change_parent_to( Node_t *                 MVRRTs_new_parent, 
        
                               const DubPath&           path_from_new_parent, 

                               const trajectory_t *     traj_from_new_parent, 

                               const Level_of_US&       level_from_new_parent) {

            // Removes itself from the set of children of its current parent

            if ( MVRRTs_parent ) { MVRRTs_parent -> MVRRTS_children.erase(this); }

            // Iterates through states along current trajectory and 
            // de-allocates each object

            for ( auto& state : *traj_from_parent ) { delete [] state; }

            // De-allocates trajectory from current parent
            delete traj_from_parent;

            // Attaches to new parent

            MVRRTs_parent = MVRRTs_new_parent;

            // Inserts itself in the set of children of its parent

            MVRRTs_parent -> MVRRTS_children.insert(this);
            
            // Copies new path from parent
            
            path_from_parent = path_from_new_parent;

            // Copies reference to new trajectory from parent

            traj_from_parent = traj_from_new_parent;

            // Copies level of unsafety of connection from new parent

            level_from_parent = level_from_new_parent;

        }
        
        /** Retrieves x-coordinate of node. 
         * 
         * @return X-coordinate of state. */

        Format_t get_point_x() const { return m_point(0);}

        /** Retrieves y-coordinate of node. 
         * 
         * @return Y-coordinate of state. */

        Format_t get_point_y() const { return m_point(1);}
            
    };

};

/** 
 * Connection from a candidate parent to a new sample or from a new sample to a candidate child. 
 */

struct CandidateConnection {
    
    /** Either candidate parent or candidate child. */
    
    DubinsTraits::Node * const  node;
    
    /** State of candidate node. */
    
    DubinsTraits::Format_t      state[3];
    
    /** Skeleton of Dubin's path from candidate parent to new sample or 
     * from new sample to candidate child. */
    
    DubPath                     path;
    
    /** Trajectory from candidate parent to new sample or from new sample to candidate child. 
     * 
     * Note that the object does not take ownership of the trajectory object. */
    
    trajectory_t *              traj;
    
    /** Level of unsafety of CandidateConnection::path. */
    
    Level_of_US                 level_connection;
    
    /** Level of unsafety from root to endpoint of CandidateConnection::path. */
    
    Level_of_US                 level_from_root;
    
    /** Copies candidate node and its state, initializes path to non-existence, and 
     * initializes levels of unsafety to zero. 
     * 
     * @param node      Candidate parent or candiate child. */
    
    CandidateConnection( DubinsTraits::Node * const node) : 
    
        // Initializes members to trivial values
    
        node(node), state(), path(), traj(NULL), level_connection(0.0), level_from_root(0.0) {
        
        // Copies planar and angular coordinates
        
        state[0] = node -> get_point_x();
        
        state[1] = node -> get_point_y();
        
        state[2] = node -> theta;
        
    }
    
    /** Does nothing. Literally. */
    
    ~CandidateConnection() {}
    
};

/** Compares pointers to candidate connections. 
 * 
 * @param lhs           LHS pointer to candidate connection object. 
 * 
 * @param rhs           RHS pointer to candidate connection object. 
 * 
 * @return True if the LHS object has a smaller level of unsafety from the root node 
 * than the RHS object. */

inline bool compare_Cand_Conn( const CandidateConnection * lhs, 
        
                               const CandidateConnection * rhs) {
    
    return ( (lhs -> level_from_root) < (rhs -> level_from_root) );
    
}

#endif	/* DEF_NODES_AND_CONNECTIONS_HPP */
