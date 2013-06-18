/*! 
 * @file 
 * @brief Dubin's Paths and Regions of R^2 cross S^1. 
 */

#ifndef DEFS_PATHS_AND_REGIONS_HPP
#define	DEFS_PATHS_AND_REGIONS_HPP

// Header files: Standard C++
#include <list>

// Header files: Custom [Luis]
#include "Defs_Numbers.hpp"

/** 
 * Skeleton of a Dubin's path. 
 * 
 * Maintains all the geometric information required to describe and discretize a time-optimal 
 * trajectory of the Dubin's Car. Note that we only compute R-S-R, L-S-L, R-S-L and L-S-R paths. 
 */

struct DubPath {
    
    /** Point at which the car enters the first (circular) arc. 
     * This is also the starting state of the path. */
    
    format_t    arc1_EntryPt [3];
    
    /** Point at which the car exits the first (circular) arc. */
    
    format_t    arc1_Exit_Pt [2];
    
    /** Point at which the car enters the second (circular) arc. */
    
    format_t    arc2_EntryPt [2];
    
    /** Point at which the car exits the first (circular) arc. 
     * This is also the ending state of the path. */
    
    format_t    arc2_Exit_Pt [3];
    
    /** Direction of motion along the first (circular) arc. Takes either values 'R' or 'L'. */
    
    char        arc1_dir;
    
    /** Direction of motion along the second (circular) arc. Takes either values 'R' or 'L'. */
    
    char        arc2_dir;
    
    /** Minimum turning radius of the car for this specific path. */
    
    format_t    turn_radius;
    
    /** True if path exists. Note that this is independent of obstacles. */
    
    bool        exists;
    
    /** Point at which the car enters the first (circular) arc, relative to the center of the arc. */
    
    format_t    arc1_EP_REL [2];
    
    /** Point at which the car exits the second (circular) arc, relative to the center of the arc. */
    
    format_t    arc2_EP_REL [2];
    
    /** Absolute coordinates of the center of the first (circular) arc. */
    
    format_t    arc1_C_ABS [2];
    
    /** Absolute coordinates of the center of the second (circular) arc. */
    
    format_t    arc2_C_ABS [2];
    
    /** Arc length of the path along the first (circular) arc. In units of radians. */
    
    format_t    arc1_len;
    
    /** Arc length of the path along the second (circular) arc. In units of radians. */
    
    format_t    arc2_len;
    
    /** Length of the straight line segment connecting the two (circular) arcs. */
    
    format_t    straight_len;
    
    /** Initializes members to trivial values. */
    
    DubPath();
    
    /** Copies arguments and initializes path to non-existence. 
     * 
     * @param start         Entry point into the first arc (2-D vector). 
     * 
     * @param endPt         Exit point out of the second arc (2-D vector). 
     * 
     * @param turn_rad      Minimum turn radius. 
     * 
     * @param arc1_Dir      Motion direction along first arc. 
     * 
     * @param arc2_Dir      Motion direction along second arc. */
    
    DubPath( const format_t * start, 
    
             const format_t * endPt, 
             
             const char arc1_Dir, 
             
             const char arc2_Dir, 
             
             const format_t turn_Radius);
    
    /** Copies arguments. 
     * 
     * @param orig      Original object. */
    
    DubPath( const DubPath& orig);
    
    /** Does nothing. Literally. */
    
    ~DubPath();
    
    /** Computes length of path. 
     * 
     * @return Length of path, if path exists; otherwise, infinity. */
    
    format_t length() const;
    
    /** Prints arc length of each arc plus straight line segment length. */
    
    void print() const;
    
    /** Copies arguments. 
     * 
     * @param orig      Original object. */
    
    DubPath& operator=( const DubPath& orig);
    
    /** Computes path length minimizer. 
     * 
     * @param lhs       LHS Dubin's path object. 
     * 
     * @param rhs       RHS Dubin's path object. 
     * 
     * @return Dubin's path with the shortest path length. */
    
    static const DubPath& arg_min( const DubPath& lhs, const DubPath& rhs);
    
    /** Computes inner tangent paths. 
     * 
     * @param path_RSL  Object where the the R-S-L path will be written. 
     * 
     * @param path_LSR  Object where the the L-S-R path will be written. */
    
    static void tangents_inner( DubPath& path_RSL, DubPath& path_LSR);
    
    /** Computes outer tangent paths. 
     * 
     * @param path_RSR  Object where the the R-S-R path will be written. 
     * 
     * @param path_LSL  Object where the the L-S-L path will be written. */
    
    static void tangents_outer( DubPath& path_RSR, DubPath& path_LSL);
    
};

/** 
 * Region of R^2 cross S^1. 
 */

struct DubRegion {
    
    /** Geometric center of region. */
    
    format_t    center [3];
    
    /** Initializes the center of the region to the origin. */
    
    DubRegion();
    
    /** Does nothing. */
    
    virtual ~DubRegion();
    
    /** Checks whether state is inside region. 
     * 
     * @param state_x   X-coordinate of state. 
     * 
     * @param state_y   Y-coordinate of state. 
     * 
     * @param state_th  THETA-coordinate of state. 
     * 
     * @return True, if state lies inside the region; false, otherwise. */
    
    virtual bool intesects( const format_t state_x, 
    
                            const format_t state_y, 
                            
                            const format_t state_th) const = 0;
    
};

/** 
 * Cylindrical region of R^2 cross S^1. 
 * 
 * Consists of a ball in R^2 and an interval in S^1. 
 */

struct DubRegion_Cylindrical : public DubRegion {
    
    /** Radius of Ball in R^2. */
    
    format_t    radius_xy;
    
    /** Radius of interval in S^1.  */
    
    format_t    radius_th;
    
    /** Initializes the center to the origin and the radii to zero. */
    
    DubRegion_Cylindrical();
    
    /** Does nothing. */
    
    ~DubRegion_Cylindrical();
    
    /** See DubRegion::intersects. */
    
    bool intesects( const format_t state_x, const format_t state_y, const format_t state_th) const;
    
};

#endif	/* DEFS_PATHS_AND_REGIONS_HPP */
