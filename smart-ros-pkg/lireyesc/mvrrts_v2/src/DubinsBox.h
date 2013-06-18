/*! 
 * @file 
 * @brief K-D tree range search for the Dubin's car scaled boxes (declarations). 
 */

#ifndef DUBINSBOX_H
#define	DUBINSBOX_H

// Header files: Custom [MP-Blocks]
#include "mpblocks/kd_tree.h"

namespace mpblocks {
namespace kd_tree {

/** 
 * Range search object for the Dubin's car (non-holonomically) scaled boxes. 
 * 
 * Handles all methods required to use an MP-Blocks K-D tree to directly retrieve 
 * all points inside the configuration space's PROJECTION ONTO R^2 that intersect 
 * the Dubin's car non-holonomically scaled boxes sitting behind and in front of 
 * any given state in R^2 cross S^1. See Karaman & Frazzoli (ICRA'13) for more 
 * information. 
 */
    
template < class Traits > 
class DubinsBox : public RangeSearchIface < Traits > {
    
public:
    
    /** Floating-point number. */
    
    typedef typename Traits::Format_t           Format_t;
    
    /** Vector in R^2. */
    
    typedef Eigen::Matrix < Format_t , 2 , 1 >  Point_t;
    
    /** Linear Transformation from R^2 to R^2. */
    
    typedef Eigen::Matrix < Format_t , 2 , 2 >  Transformation_t;
    
    /** MP-Blocks K-D tree node. */
    
    typedef typename Traits::Node               Node_t;
    
    /** Set of MP-Blocks K-D tree nodes. */
    
    typedef std::vector < Node_t* >             List_t;
    
    /** Hyper-rectangle of MP-Blocks K-D tree. */
    
    typedef HyperRect < Traits >                HyperRect_t;
    
private:
    
    /** Minimum turning radius of the car. */
    
    Format_t            turn_radius;
    
    /** Gamma-RRG-star constant. 
     * 
     * This constant is defined in Karaman & Frazzoli, IJRR'11. */
    
    Format_t            gamma_RRG_star;
    
    /** Factor by which the length of the semi-theta axis is to be scaled up. */
    
    Format_t            semi_theta_scaling_factor;
    
    /** Length of the semi-major axis of the box. */
    
    Format_t            semi_maj;
    
    /** Length of the semi-minor axis of the box. */
    
    Format_t            semi_min;
    
    /** Unit vector in the direction of the semi-major axis. */
    
    Point_t             semi_maj_dir;
    
    /** Unit vector in the direction of the semi-minor axis. */
    
    Point_t             semi_min_dir;
    
    /** Distance from the queried state to either of the two boxes. */
    
    Format_t            offset;
    
    /** Geometric center of the current box. */
    
    Point_t             center;
    
    /** Maps every point inside the box to the unit ball (in the L-infinity sense). 
     * 
     * This matrix is computed only when the object is reset, so we keep track 
     * of it for convenience. It is obtained by inverting the matrix associated with 
     * the span of the semi-major and semi-major (scaled) vectors. */
    
    Transformation_t    A_inverse;
    
    /** Top right corner of the box. */
    
    Point_t             top_R;
    
    /** Bottom right corner of the box. */
    
    Point_t             bot_R;
    
    /** Top left corner of the box. */
    
    Point_t             top_L;
    
    /** Bottom left corner of the box. */
    
    Point_t             bot_L;
    
    /** Midpoint of the right edge of the box. */
    
    Point_t             edgeR_mid;
    
    /** Midpoint of the left edge of the box. */
    
    Point_t             edgeL_mid;
    
    /** Midpoint of the top edge of the box. */
    
    Point_t             edgeT_mid;
    
    /** Midpoint of the bottom edge of the box. */
    
    Point_t             edgeB_mid;
    
    /** Set of nodes found during the range search procedure. */
    
    List_t              nodes_found;
    
    // ------------------------------------------------------------------------
    // CONSTRUCTOR AND DESTRUCTOR
    // ------------------------------------------------------------------------
    
public:
    
    /** Computes the gamma-RRG-star constant. 
     * 
     * @param turn_radius       Minimum turning radius of the car.
     * 
     * @param measure_R2        Measure (area) of the R^2 projection of the environment. */
    
    DubinsBox ( const Format_t turn_radius, 
            
                const Format_t measure_R2);
    
    /** Does nothing. */
    
    ~DubinsBox();
    
    // ------------------------------------------------------------------------
    // METHODS: RANGE SEARCH INTERFACE
    // ------------------------------------------------------------------------
    
public:
    
    /** Inserts node into search result set if it is found to lie inside the box. 
     * 
     * @param q             Point associated with node (vector in R^2). 
     * 
     * @param n             Pointer to actual node. */
    
    void evaluate( const Point_t& q, Node_t* n);
    
    /** Checks for intersection between hyper-rectangle and box. 
     * 
     * @param h             Hyper-rectangle of MP-Blocks K-D tree. 
     * 
     * @return True if the two shapes intersect. */
    
    bool shouldRecurse( const HyperRect_t& h);
    
    // ------------------------------------------------------------------------
    // METHODS: OTHER PUBLIC
    // ------------------------------------------------------------------------
    
    /** Retrieves length semi-theta axis of box. 
     * 
     * @return Largest angle (in radians) by which any state lying inside the box 
     * is allowed to differ from that of the queried state. */
    
    Format_t get_len_semi_theta_axis() const;
    
    /** Retrieves result of range search. 
     * 
     * @return Constant Reference to the search result set. */
    
    const List_t& get_result() const;
    
    /** Moves the box from sitting behind the state to sitting in front of it. 
     * 
     * Note that in addition to recomputing the center, corners and midpoint of 
     * the box, this procedure also clears the search result set. */
    
    void move_forward();
    
    /** Prints lengths of major, minor and theta axes. */
    
    void print_geometric_information() const;
    
    /** Sets up box. 
     * 
     * @param state_xy          Planar coordinates of state (vector in R^2). 
     * 
     * @param state_th          Angular coordinate of state (scalar in S^1). 
     * 
     * @param iter_num          Number of vertices in the RRT* tree. 
     * 
     * @param location_of_box   Location of the box relative to the given state. 
     * Provide 'B' for the box sitting behind the state, and 'F' for the box 
     * sitting in front of the state. */
    
    void reset( const Point_t& state_xy, 
            
                const Format_t state_th, 
            
                const unsigned int iter_num, 
            
                const char location_of_box);
    
    /** Sets factor by which the length of the semi-theta axis is to be scaled up. 
     * 
     * @param factor            Scaling factor (larger than or equal to one). */
    
    void set_semi_theta_scaling_factor( const Format_t factor);
    
    // ------------------------------------------------------------------------
    // METHODS: OTHER PRIVATE
    // ------------------------------------------------------------------------
    
private:
    
    /** Calculates the coordinates of the corners of the box. */
    
    void calculate_corners();
    
    /** Calculates the coordinates of the midpoints of the edges of the box. */
    
    void calculate_edge_midpoints();
    
    /** Checks for intersection between point and box. 
     * 
     * @param q             Point (vector in R^2). 
     * 
     * @return True if the point lies inside the box. */
    
    bool is_inside( const Point_t& q) const;
    
};

}
}

#include "DubinsBox.hpp"

#endif	/* DUBINSBOX_H */
