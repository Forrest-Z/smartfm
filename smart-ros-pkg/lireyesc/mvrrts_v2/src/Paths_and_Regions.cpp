// Header files: Standard C++
#include <iostream>
#include <limits>

// Header files: Custom [Luis]
#include "Paths_and_Regions.hpp"
#include "Defs_Utilities.hpp"

// ----------------------------------------------------------------------------------------
// DUBPATH
// ----------------------------------------------------------------------------------------

DubPath::DubPath() : 

    arc1_EntryPt { 0 , 0 , 0 }, 
        
    arc1_Exit_Pt { 0 , 0 }, 
            
    arc2_EntryPt { 0 , 0 }, 
            
    arc2_Exit_Pt { 0 , 0 , 0 }, 
            
    arc1_dir('X'), 
            
    arc2_dir('X'), 
            
    turn_radius(0.0), 
            
    exists(false), 
            
    arc1_EP_REL { 0 , 0 }, 
            
    arc2_EP_REL { 0 , 0 }, 
            
    arc1_C_ABS { 0 , 0 }, 
            
    arc2_C_ABS { 0 , 0 }, 
            
    arc1_len(0), 
            
    arc2_len(0), 
            
    straight_len( std::numeric_limits<format_t>::max() ) {}

DubPath::DubPath( const format_t * start, 
                  const format_t * endPt, 
                  const char arc1_Dir, 
                  const char arc2_Dir, 
                  const format_t turn_Radius) : 
                  
    arc1_EntryPt {  start[0],   start[1],   start[2] }, 

    arc1_Exit_Pt {  0,          0           }, 

    arc2_EntryPt {  0,          0           }, 

    arc2_Exit_Pt {  endPt[0],   endPt[1],   endPt[2] }, 

    arc1_dir(arc1_Dir), arc2_dir(arc2_Dir), turn_radius(turn_Radius), exists(false) {

    // Computes entry point into first arc relative to center
                    
    switch (arc1_dir) {
        
        case 'R': 
            
            arc1_EP_REL[0] = -turn_radius * (format_t) cos( arc1_EntryPt[2] - M_PI_2 );
            arc1_EP_REL[1] = -turn_radius * (format_t) sin( arc1_EntryPt[2] - M_PI_2 );
            break;
            
        case 'L':
            
            arc1_EP_REL[0] = -turn_radius * (format_t) cos( arc1_EntryPt[2] + M_PI_2 );
            arc1_EP_REL[1] = -turn_radius * (format_t) sin( arc1_EntryPt[2] + M_PI_2 );
            break;
            
        default:
            
            std::cerr << "In line " <<  __LINE__ << " of " << __FILE__ << ": ";
            std::cerr << "DIRECTION is invalid" << std::endl;
            exit(EXIT_FAILURE);
            
    }

    // Computes absolute coordinates of center of first arc
    
    arc1_C_ABS[0] = arc1_EntryPt[0] - arc1_EP_REL[0];
    arc1_C_ABS[1] = arc1_EntryPt[1] - arc1_EP_REL[1];

    // Computes exit point out of second arc relative to center
    
    switch (arc2_dir) {
        
        case 'R': 
            
            arc2_EP_REL[0] = -turn_radius * (format_t) cos( arc2_Exit_Pt[2] - M_PI_2 );
            arc2_EP_REL[1] = -turn_radius * (format_t) sin( arc2_Exit_Pt[2] - M_PI_2 );
            break;
            
        case 'L':
            
            arc2_EP_REL[0] = -turn_radius * (format_t) cos( arc2_Exit_Pt[2] + M_PI_2 );
            arc2_EP_REL[1] = -turn_radius * (format_t) sin( arc2_Exit_Pt[2] + M_PI_2 );
            break;
            
        default:
            
            std::cerr << "In line " <<  __LINE__ << " of " << __FILE__ << ": ";
            std::cerr << "DIRECTION is invalid" << std::endl;
            exit(EXIT_FAILURE);
            
    }

    // Computes absolute coordinates of center of second arc
    
    arc2_C_ABS[0] = arc2_Exit_Pt[0] - arc2_EP_REL[0];
    
    arc2_C_ABS[1] = arc2_Exit_Pt[1] - arc2_EP_REL[1];

    // Initializes arc angles to zero
    
    arc1_len = 0; arc2_len = 0;
    
    // Initializes straight line segment length to largest floating point value
    
    straight_len = std::numeric_limits<format_t>::max();

}

DubPath::DubPath( const DubPath& orig) {
    
    *this = orig;

}

DubPath::~DubPath() {}

format_t DubPath::length() const {
    
    if ( exists ) { return turn_radius * (arc1_len + arc2_len) + straight_len; }
    
    else { return std::numeric_limits<format_t>::max(); }
    
}
void DubPath::print() const {
    
    if ( exists ) {
        
        std::cout << "Arc-1 angle: " << rad2Deg(arc1_len) << " deg\t";
        
        std::cout << "Arc-2 angle: " << rad2Deg(arc2_len) << " deg\t";
        
        std::cout << "Straight line length: " << straight_len << std::endl;
        
        std::cout << "Path length: " << length() << std::endl;
        
    }
    
    else { std::cout << "Path does not exist." << std::endl; }
    
}

DubPath& DubPath::operator=( const DubPath& orig) {
    
    arc1_EntryPt[0] = orig.arc1_EntryPt[0];
    arc1_EntryPt[1] = orig.arc1_EntryPt[1];
    arc1_EntryPt[2] = orig.arc1_EntryPt[2];

    arc1_Exit_Pt[0] = orig.arc1_Exit_Pt[0];
    arc1_Exit_Pt[1] = orig.arc1_Exit_Pt[1];

    arc2_EntryPt[0] = orig.arc2_EntryPt[0];
    arc2_EntryPt[1] = orig.arc2_EntryPt[1];

    arc2_Exit_Pt[0] = orig.arc2_Exit_Pt[0];
    arc2_Exit_Pt[1] = orig.arc2_Exit_Pt[1];
    arc2_Exit_Pt[2] = orig.arc2_Exit_Pt[2];

    arc1_dir        = orig.arc1_dir;
    arc2_dir        = orig.arc2_dir;
    
    turn_radius     = orig.turn_radius;

    exists          = orig.exists;

    arc1_EP_REL[0]  = orig.arc1_EP_REL[0];
    arc1_EP_REL[1]  = orig.arc1_EP_REL[1];

    arc2_EP_REL[0]  = orig.arc2_EP_REL[0];
    arc2_EP_REL[1]  = orig.arc2_EP_REL[1];

    arc1_C_ABS[0]   = orig.arc1_C_ABS[0];
    arc1_C_ABS[1]   = orig.arc1_C_ABS[1];

    arc2_C_ABS[0]   = orig.arc2_C_ABS[0];
    arc2_C_ABS[1]   = orig.arc2_C_ABS[1];

    arc1_len        = orig.arc1_len;
    arc2_len        = orig.arc2_len;
    
    straight_len    = orig.straight_len;

    return *this;

}

const DubPath& DubPath::arg_min( const DubPath& lhs, const DubPath& rhs) {
    
    return ( ( lhs.length() < rhs.length() ) ? lhs : rhs );
    
}

void DubPath::tangents_inner( DubPath& path_RSL, DubPath& path_LSR) {
    
    // ------------------------------------------------------------------------
    // PATH: R-S-L
    // ------------------------------------------------------------------------
    
    // Declares and computes vector between centers of arcs
    
    format_t v_12 [] = {    path_RSL.arc2_C_ABS[0] - path_RSL.arc1_C_ABS[0] , 
                            path_RSL.arc2_C_ABS[1] - path_RSL.arc1_C_ABS[1] };
    
    // Declares and computes Euclidean norm of vector before normalizing it
    
    format_t v_12_norm = vec2d_norm(v_12);
    
    v_12[0] = v_12[0] / v_12_norm;
    v_12[1] = v_12[1] / v_12_norm;
    
    // Declares and computes RHS of equation 
    // 
    // \hat{v_12} \cdot \hat{n} = \frac{ 2 * turn_radius }{ v_12_norm }, 
    // 
    // which, from linear algebra, is also the cosine of the angle 
    // between vectors \hat{v_12} and \hat{n}
    
    format_t eqRHS = ( 2 * path_RSL.turn_radius ) / v_12_norm;
    
    // If the centers of the arcs are separated by a distance 
    // of at least twice the minimum turning radius
    
    if ( eqRHS < 1 ) {
        
        // Declares and computes v_C2Tang, the vector from the center 
        // of the first arc to the exit point out of the arc (i.e. the 
        // tangent point); this vector is obtained by solving the equation 
        // above for \hat{n} and then scaling the vector by turn_radius
        
        format_t v_C2Tang [] = { 
            
        path_RSL.turn_radius 
        * ( (v_12[0] * eqRHS) - (v_12[1] * (format_t) sqrt( 1 - (eqRHS * eqRHS) ) ) ), 
        
        path_RSL.turn_radius 
        * ( (v_12[0] * (format_t) sqrt( 1 - (eqRHS * eqRHS) ) ) + (v_12[1] * eqRHS) ) };
        
        // Computes exit point out of first arc
        
        path_RSL.arc1_Exit_Pt[0] = path_RSL.arc1_C_ABS[0] + v_C2Tang[0];
        path_RSL.arc1_Exit_Pt[1] = path_RSL.arc1_C_ABS[1] + v_C2Tang[1];
        
        // Computes entry point into second arc
        
        path_RSL.arc2_EntryPt[0] = path_RSL.arc2_C_ABS[0] - v_C2Tang[0];
        path_RSL.arc2_EntryPt[1] = path_RSL.arc2_C_ABS[1] - v_C2Tang[1];
        
        // Computes the length of the straight line segment 
        // from the norm of the vector between tangent points
        
        path_RSL.straight_len = 
                
        vec2d_norm( path_RSL.arc1_Exit_Pt, path_RSL.arc2_EntryPt);
        
        // Computes angles traversed along first and second arcs
        
        path_RSL.arc1_len = angle_diff_directed( path_RSL.arc1_EP_REL, v_C2Tang, 'R');
        
        path_RSL.arc2_len = angle_diff_directed( vec2d_neg(v_C2Tang), path_RSL.arc2_EP_REL, 'L');
        
        // Marks path as existent
        
        path_RSL.exists = true;
        
    }
    
    // Otherwise, mark path as non-existent
    
    else { path_RSL.exists = false; }
    
    // ------------------------------------------------------------------------
    // PATH: L-S-R
    // ------------------------------------------------------------------------
    
    // Computes vector between centers of arcs
    
    v_12[0] = path_LSR.arc2_C_ABS[0] - path_LSR.arc1_C_ABS[0];
    v_12[1] = path_LSR.arc2_C_ABS[1] - path_LSR.arc1_C_ABS[1];
    
    // Computes Euclidean norm of vector before normalizing it
    v_12_norm = vec2d_norm(v_12);
    
    v_12[0] = v_12[0] / v_12_norm;
    v_12[1] = v_12[1] / v_12_norm;
    
    // Declares and computes RHS of equation
    // (same equation as for the R-S-L path)
    
    eqRHS = ( 2 * path_LSR.turn_radius ) / v_12_norm;
    
    // If the centers of the arcs are separated by a distance 
    // of at least twice the minimum turning radius
    
    if ( eqRHS < 1 ) {
        
        // Declares and computes v_C2Tang, the vector from the center 
        // of the first arc to the exit point out of the arc (note that 
        // for this path we rotate the v_12 vector in the CC direction)
        
        format_t v_C2Tang [] = { 
            
        path_LSR.turn_radius 
        * ( (v_12[0] * eqRHS) + (v_12[1] * (format_t) sqrt( 1 - (eqRHS * eqRHS) ) ) ), 
        
        path_LSR.turn_radius 
        * ( ( -v_12[0] * (format_t) sqrt( 1 - (eqRHS * eqRHS) ) ) + (v_12[1] * eqRHS) ) };
        
        // Computes exit point out of first arc
        
        path_LSR.arc1_Exit_Pt[0] = path_LSR.arc1_C_ABS[0] + v_C2Tang[0];
        path_LSR.arc1_Exit_Pt[1] = path_LSR.arc1_C_ABS[1] + v_C2Tang[1];
        
        // Computes entry point into second arc
        
        path_LSR.arc2_EntryPt[0] = path_LSR.arc2_C_ABS[0] - v_C2Tang[0];
        path_LSR.arc2_EntryPt[1] = path_LSR.arc2_C_ABS[1] - v_C2Tang[1];
        
        // Computes the length of the straight line segment 
        // from the norm of the vector between tangent points
        
        path_LSR.straight_len = 
                
        vec2d_norm( path_LSR.arc1_Exit_Pt, path_LSR.arc2_EntryPt);
        
        // Computes angles traversed along first and second arcs
        
        path_LSR.arc1_len = angle_diff_directed( path_LSR.arc1_EP_REL, v_C2Tang, 'L');
        
        path_LSR.arc2_len = angle_diff_directed( vec2d_neg(v_C2Tang), path_LSR.arc2_EP_REL, 'R');
        
        // Marks path as existent
        
        path_LSR.exists = true;
        
    }
    
    // Otherwise, mark path as non-existent
    
    else { path_LSR.exists = false; }
    
}

void DubPath::tangents_outer( DubPath& path_RSR, DubPath& path_LSL) {
    
    // ------------------------------------------------------------------------
    // PATH: R-S-R
    // ------------------------------------------------------------------------
    
    // Declares and computes vector perpendicular to the centers 
    // of the circles, obtained by taking the vector between 
    // centers u_12 and rotating it by POSITIVE pi/2 radians
    
    format_t v_12 [] = {    path_RSR.arc1_C_ABS[1] - path_RSR.arc2_C_ABS[1] , 
                            path_RSR.arc2_C_ABS[0] - path_RSR.arc1_C_ABS[0] };
    
    // Computes the length of the straight line segment
    
    path_RSR.straight_len = vec2d_norm(v_12);
    
    // Simultaneously normalizes vector v_12 and scales it by 
    // the minimum turning radius; the resulting vector joins 
    // the centers of the circles with the desired tangent points
    
    v_12[0] = ( v_12[0] / path_RSR.straight_len ) * path_RSR.turn_radius;
    v_12[1] = ( v_12[1] / path_RSR.straight_len ) * path_RSR.turn_radius;
    
    // Computes exit point out of first arc
    
    path_RSR.arc1_Exit_Pt[0] = path_RSR.arc1_C_ABS[0] + v_12[0];
    path_RSR.arc1_Exit_Pt[1] = path_RSR.arc1_C_ABS[1] + v_12[1];
    
    // Computes entry point into second arc
    
    path_RSR.arc2_EntryPt[0] = path_RSR.arc2_C_ABS[0] + v_12[0];
    path_RSR.arc2_EntryPt[1] = path_RSR.arc2_C_ABS[1] + v_12[1];
    
    // Computes angles traversed along first and second arcs
    
    path_RSR.arc1_len = angle_diff_directed( path_RSR.arc1_EP_REL, v_12, 'R');
    
    path_RSR.arc2_len = angle_diff_directed( v_12, path_RSR.arc2_EP_REL, 'R');
    
    // Marks path as existent
    
    path_RSR.exists = true;
    
    // ------------------------------------------------------------------------
    // PATH: L-S-L
    // ------------------------------------------------------------------------
    
    // Declares and computes vector perpendicular to the centers 
    // of the circles, obtained by taking the vector between 
    // centers u_12 and rotating it by NEGATIVE pi/2 radians
    
    v_12[0] = path_LSL.arc2_C_ABS[1] - path_LSL.arc1_C_ABS[1];
    v_12[1] = path_LSL.arc1_C_ABS[0] - path_LSL.arc2_C_ABS[0];
    
    // Computes the length of the straight line segment
    
    path_LSL.straight_len = vec2d_norm(v_12);
    
    // Simultaneously normalizes vector v_12 and scales it by 
    // the minimum turning radius; the resulting vector joins 
    // the centers of the circles with the desired tangent points
    
    v_12[0] = ( v_12[0] / path_LSL.straight_len ) * path_LSL.turn_radius;
    v_12[1] = ( v_12[1] / path_LSL.straight_len ) * path_LSL.turn_radius;
    
    // Computes exit point out of first arc
    
    path_LSL.arc1_Exit_Pt[0] = path_LSL.arc1_C_ABS[0] + v_12[0];
    path_LSL.arc1_Exit_Pt[1] = path_LSL.arc1_C_ABS[1] + v_12[1];
    
    // Computes entry point into second arc
    
    path_LSL.arc2_EntryPt[0] = path_LSL.arc2_C_ABS[0] + v_12[0];
    path_LSL.arc2_EntryPt[1] = path_LSL.arc2_C_ABS[1] + v_12[1];
    
    // Computes angles traversed along first and second arcs
    
    path_LSL.arc1_len = angle_diff_directed( path_LSL.arc1_EP_REL, v_12, 'L');
    
    path_LSL.arc2_len = angle_diff_directed( v_12, path_LSL.arc2_EP_REL, 'L');
    
    // Marks path as existent
    
    path_LSL.exists = true;
    
}

// ----------------------------------------------------------------------------------------
// DUBREGION
// ----------------------------------------------------------------------------------------

DubRegion::DubRegion() : center { 0, 0, 0} {}
    
DubRegion::~DubRegion() {}

// ----------------------------------------------------------------------------------------
// DUBREGION-CYLINDRICAL
// ----------------------------------------------------------------------------------------

DubRegion_Cylindrical::DubRegion_Cylindrical() : DubRegion(), radius_xy(0.0), radius_th(0.0) {}

DubRegion_Cylindrical::~DubRegion_Cylindrical() {}

bool DubRegion_Cylindrical::intesects( const format_t state_x, 
        
                                       const format_t state_y, 
        
                                       const format_t state_th) const {
    
    // Returns true, if state is within region; otherwise, returns false
        
    if ( (state_x - center[0]) * (state_x - center[0]) 
            
       + (state_y - center[1]) * (state_y - center[1]) < (radius_xy * radius_xy) ) {

        if ( angle_diff_smallest( state_th, center[2]) < radius_th ) { return true; }

    }

    return false;
    
}
