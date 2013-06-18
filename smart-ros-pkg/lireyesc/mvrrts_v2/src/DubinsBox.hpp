/*! 
 * @file 
 * @brief K-D tree range search for the Dubin's car scaled boxes (definitions). 
 */

#ifndef DUBINSBOX_HPP
#define	DUBINSBOX_HPP

// Header files: Standard C
#include <cmath>

// Header files: Off-the-shelf libraries [Eigen]
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

// Header files: Custom [Luis]
#include "Defs_Utilities.hpp"

namespace mpblocks {
namespace kd_tree {

// ----------------------------------------------------------------------------------------
// CONSTRUCTOR AND DESTRUCTOR
// ----------------------------------------------------------------------------------------

template < class Traits > 
DubinsBox < Traits > :: DubinsBox( const Format_t turn_radius, 
        
                                   const Format_t measure_R2) : 
                                   
    // Copies argument
                                   
    turn_radius(turn_radius) {
    
    // Throws error if there is a dimension mismatch
    
    if ( Traits::NDim != 2 ) {
        
        std::cerr << "In line " <<  __LINE__ << " of " << __FILE__ << ": ";
        std::cerr << "DIMENSION of Traits is not two" << std::endl;
        exit(EXIT_FAILURE);
        
    }
    
    // Declares and computes Lebesgue measure of the environment
    
    Format_t mu_X = measure_R2 * (2 * M_PI);
    
    // Computes and sets gamma-RRG-star constant
    
    gamma_RRG_star = 2.0 * pow( mu_X / M_PI , 1.0 / 3.0 ) / M_LN2;
    
    // Sets factor by which the length of the semi-theta axis is to be scaled up
    
    semi_theta_scaling_factor = 1.0;
    
}

template < class Traits > 
DubinsBox < Traits > :: ~DubinsBox() {}

// ----------------------------------------------------------------------------------------
// METHODS: RANGE SEARCH INTERFACE
// ----------------------------------------------------------------------------------------

template < class Traits > 
void DubinsBox < Traits > :: evaluate( const Point_t& q, Node_t* n) {
    
    // If point is inside box then insert corresponding node into set
    
    if ( is_inside(q) ) { nodes_found.push_back(n); }
    
}

template < class Traits > 
bool DubinsBox < Traits > :: shouldRecurse( const HyperRect_t& h) {
    
    // If R-edge of the K-D tree rectangle is separating plane, shapes do not intersect
    
    if ( ( top_R(0) > h.maxExt(0) ) && 
         ( bot_R(0) > h.maxExt(0) ) && 
         ( top_L(0) > h.maxExt(0) ) && 
         ( bot_L(0) > h.maxExt(0) ) ) { return false; }
    
    // If T-edge of the K-D tree rectangle is separating plane, shapes do not intersect
    
    if ( ( top_R(1) > h.maxExt(1) ) && 
         ( bot_R(1) > h.maxExt(1) ) && 
         ( top_L(1) > h.maxExt(1) ) && 
         ( bot_L(1) > h.maxExt(1) ) ) { return false; }
    
    // If L-edge of the K-D tree rectangle is separating plane, shapes do not intersect
    
    if ( ( top_R(0) < h.minExt(0) ) && 
         ( bot_R(0) < h.minExt(0) ) && 
         ( top_L(0) < h.minExt(0) ) && 
         ( bot_L(0) < h.minExt(0) ) ) { return false; }
    
    // If B-edge of the K-D tree rectangle is separating plane, shapes do not intersect
    
    if ( ( top_R(1) < h.minExt(1) ) && 
         ( bot_R(1) < h.minExt(1) ) && 
         ( top_L(1) < h.minExt(1) ) && 
         ( bot_L(1) < h.minExt(1) ) ) { return false; }
    
    // Declares and computes L-T corner of the K-D tree rectangle
    
    Point_t hyper_LT ( h.minExt(0) , h.maxExt(1) );
    
    // Declares and computes R-B corner of the K-D tree rectangle
    
    Point_t hyper_RB ( h.maxExt(0) , h.minExt(1) );
    
    // If R-edge of the box is separating plane, shapes do not intersect
    
    if ( ( semi_maj_dir.dot( h.maxExt - edgeR_mid) > 0 ) && 
         ( semi_maj_dir.dot( h.minExt - edgeR_mid) > 0 ) && 
         ( semi_maj_dir.dot( hyper_LT - edgeR_mid) > 0 ) && 
         ( semi_maj_dir.dot( hyper_RB - edgeR_mid) > 0 ) ) { return false; }
    
    // If T-edge of the box is separating plane, shapes do not intersect
    
    if ( ( semi_min_dir.dot( h.maxExt - edgeT_mid) > 0 ) && 
         ( semi_min_dir.dot( h.minExt - edgeT_mid) > 0 ) && 
         ( semi_min_dir.dot( hyper_LT - edgeT_mid) > 0 ) && 
         ( semi_min_dir.dot( hyper_RB - edgeT_mid) > 0 ) ) { return false; }
    
    // If L-edge of the box is separating plane, shapes do not intersect
    
    if ( ( ( -semi_maj_dir ).dot( h.maxExt - edgeL_mid) > 0 ) && 
         ( ( -semi_maj_dir ).dot( h.minExt - edgeL_mid) > 0 ) && 
         ( ( -semi_maj_dir ).dot( hyper_LT - edgeL_mid) > 0 ) && 
         ( ( -semi_maj_dir ).dot( hyper_RB - edgeL_mid) > 0 ) ) { return false; }
    
    // If B-edge of the box is separating plane, shapes do not intersect
    
    if ( ( ( -semi_min_dir ).dot( h.maxExt - edgeB_mid) > 0 ) && 
         ( ( -semi_min_dir ).dot( h.minExt - edgeB_mid) > 0 ) && 
         ( ( -semi_min_dir ).dot( hyper_LT - edgeB_mid) > 0 ) && 
         ( ( -semi_min_dir ).dot( hyper_RB - edgeB_mid) > 0 ) ) { return false; }
    
    // If this line is reached, shaped do intersect
    
    return true;
    
}

// ----------------------------------------------------------------------------------------
// METHODS: OTHER PUBLIC
// ----------------------------------------------------------------------------------------

template < class Traits > 
typename Traits::Format_t 
DubinsBox < Traits > :: get_len_semi_theta_axis() const {
    
    Format_t semi_theta = semi_theta_scaling_factor * (semi_maj / turn_radius);
    
    return ( semi_theta <= M_PI ? semi_theta : M_PI );
    
}

template < class Traits > 
const typename DubinsBox < Traits > :: List_t& 
DubinsBox < Traits > :: get_result() const { return nodes_found; }

template < class Traits > 
void DubinsBox < Traits > :: move_forward() {
    
    // Shifts the center forward
    
    center = center + 2 * (offset + semi_maj) * semi_maj_dir;
    
    // Updates the locations of the corners
    
    calculate_corners();
    
    // Updates the locations of the midpoints of the edges
    
    calculate_edge_midpoints();
    
    // Clears the set of nodes
    
    nodes_found.clear();
    
}

template < class Traits > 
void DubinsBox < Traits > :: print_geometric_information() const {
    
    std::cout << "DUBIN'S CAR SCALED BOXES (DC-SB) RANGE SEARCH OBJECT: " << std::endl;
    
    std::cout << "\tMajor axis = " << 2 * semi_maj << std::endl;
    
    std::cout << "\tMinor axis = " << 2 * semi_min << std::endl;
    
    std::cout << "\tTheta axis = " << 2 * rad2Deg( get_len_semi_theta_axis() ) << std::endl;
    
    std::cout << "\tOffset distance = " << 2 * get_len_semi_theta_axis() << std::endl;
    
}

template < class Traits > 
void DubinsBox < Traits > :: reset( const Point_t& state_xy, 
        
                                            const Format_t state_th, 
        
                                            const unsigned int iter_num, 
        
                                            const char location_of_box) {
    
    // Computes length of semi-major axis
    semi_maj = 0.5 * pow( turn_radius * gamma_RRG_star * ( log(iter_num + 1) / iter_num ) , 0.25 );
    
    // Computes length of semi-minor axis
    semi_min = 2.0 * semi_maj * semi_maj;
    
    // If length of semi-major axis is less than length of semi-minor axis
    if ( semi_maj < semi_min ) {
        
        // Sets length of both axes to square root of area of R^2 projection of box
        semi_maj = sqrt( semi_maj * semi_min ); semi_min = semi_maj;
        
    }
    
    // Computes unit vector along the semi-major axis
    semi_maj_dir(0) = (Format_t) cos(state_th);
    semi_maj_dir(1) = (Format_t) sin(state_th);
    
    // Computes unit vector along the semi-minor axis
    semi_min_dir(0) = -semi_maj_dir(1);
    semi_min_dir(1) =  semi_maj_dir(0);
    
    // Declares and computes cosine of angle alpha
    Format_t cos_alpha = 0.5 * ( 1 + cos( get_len_semi_theta_axis() ) 
                                   - (semi_min / turn_radius) );
    
    // If cosine of alpha is smaller than negative one then set it to that number
    if ( cos_alpha < -1.0 ) { cos_alpha = -1.0; }
    
    // Compute actual offset
    offset = turn_radius * ( sin( get_len_semi_theta_axis() ) 
                             + 2 * sqrt( 1 - (cos_alpha * cos_alpha) ) );
    
    // Depending on the location of the box
    switch (location_of_box) {
        
        // Sets up the box sitting behind the state
        case ('B'): 
            center = state_xy - ( (offset + semi_maj) * semi_maj_dir); break;
            
        // Sets up the box sitting in front of the state
        case ('F'):
            center = state_xy + ( (offset + semi_maj) * semi_maj_dir); break;
            
        // Throws error
        default:
            std::cerr << "DubinsDynamicsBox::reset" << std::endl;
            exit(EXIT_FAILURE);
            
    }
    
    // Computes matrix of semi-major axes
    Transformation_t A; A << (semi_maj * semi_maj_dir), (semi_min * semi_min_dir);
    
    // Computes inverse of matrix
    A_inverse = ( A.inverse() ).eval();
    
    // Computes corners of box
    calculate_corners();
    
    // Computes midpoints of edges of box
    calculate_edge_midpoints();
    
    // Clears the set of nodes
    nodes_found.clear();
    
}

template < class Traits > 
void DubinsBox < Traits > :: set_semi_theta_scaling_factor( const Format_t factor) {
    
    // If the scaling factor is less than one then throw error
    
    if ( factor < 1.0 ) {
        
        std::cerr << "In line " <<  __LINE__ << " of " << __FILE__ << ": ";
        std::cerr << "SCALING FACTOR is less than one" << std::endl;
        exit(EXIT_FAILURE);
        
    }
    
    // Otherwise copies scaling factor
    
    else { semi_theta_scaling_factor = factor; }
    
}

// ----------------------------------------------------------------------------------------
// METHODS: OTHER PRIVATE
// ----------------------------------------------------------------------------------------

template < class Traits > 
void DubinsBox < Traits > :: calculate_corners() {
    
    // Carries self-evident vector operations
    
    top_R = center + ( semi_maj * semi_maj_dir) + (semi_min * semi_min_dir);
    
    bot_R = center + ( semi_maj * semi_maj_dir) - (semi_min * semi_min_dir);
    
    top_L = center + (-semi_maj * semi_maj_dir) + (semi_min * semi_min_dir);
    
    bot_L = center + (-semi_maj * semi_maj_dir) - (semi_min * semi_min_dir);
    
}

template < class Traits > 
void DubinsBox < Traits > :: calculate_edge_midpoints() {
    
    // Carries self-evident vector operations
    
    edgeR_mid = center + (semi_maj * semi_maj_dir);
    
    edgeT_mid = center + (semi_min * semi_min_dir);
    
    edgeL_mid = center - (semi_maj * semi_maj_dir);
    
    edgeB_mid = center - (semi_min * semi_min_dir);
    
}

template < class Traits > 
bool DubinsBox < Traits > :: is_inside( const Point_t& q) const {
    
    // Computes solution to the equation A * x = (q - center), where A is the 
    // matrix formed by the semi-major and semi-minor axes, and center is the 
    // geometric center of the box
    
    Point_t solution ( A_inverse * (q - center) );
    
    // If infinity-norm of solution is less than or equal to one, then point is inside the box
    
    if ( ( fabs( solution(0) ) <= 1.0 ) && ( fabs( solution(1) ) <= 1.0 ) ) { return true; }
    
    // Otherwise, point is outside the box
    
    else { return false; }
    
}

}
}

#endif	/* DUBINSBOX_HPP */
