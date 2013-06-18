/*! 
 * @file 
 * @brief Various convenience functions. 
 */

#ifndef UTILS_ANGLE_VEC_HPP
#define	UTILS_ANGLE_VEC_HPP

// Header files: Standard C
#include <cmath>
#include <cstdlib>

// Header files: Custom
#include "Defs_Numbers.hpp"

// ------------------------------------------------------------------------
// FUNCTIONS FOR ANGLES
// ------------------------------------------------------------------------

/** Directed angle difference from by given vectors. 
 * 
 * @param vector_0      First vector. 
 * 
 * @param vector_1      Second vector. 
 * 
 * @param direction     Direction of the arc. 
 * 
 * @return              Directed angle difference from first vector to second vector. 
 * Note that this quantity is always non-negative. */

inline format_t angle_diff_directed( const format_t * vector_0, 
        
                                     const format_t * vector_1, 
        
                                     const char direction) {
    
    format_t delta_theta = atan2( vector_1[1], vector_1[0]) - atan2( vector_0[1], vector_0[0]);
    
    // If angle is negative and circle flows left then add two PI
    
    if ( (delta_theta < 0) && direction == 'L' ) { delta_theta += 2 * M_PI; }
    
    // If angle is positive and circle flows right then subtract two PI
    
    else if ( (delta_theta > 0) && direction == 'R' ) { delta_theta -= 2 * M_PI; }
    
    // In any case return absolute value
    
    return fabs(delta_theta);
    
}

/** Smallest angle difference between given angles. 
 * 
 * @param angle_0   First angle (in the open-closed interval from -PI to PI). 
 * 
 * @param angle_1   Second angle (in the open-closed interval from -PI to PI). 
 * 
 * @return          Smallest angle difference between first and second angles. 
 * Note that this quantity is always non-negative. */

inline format_t angle_diff_smallest( const format_t angle_0, const format_t angle_1) {
    
    return fmin( fabs(angle_0 - angle_1) , 2 * M_PI - fabs(angle_0 - angle_1) );
    
}

/** Degrees to radians conversion. 
 * 
 * @param angle     Angle in degrees. 
 * 
 * @return          Angle in radians. */

inline format_t deg2rad( const format_t angle) { return ( M_PI / 180.0 ) * angle; }

/** Radians to degrees conversion. 
 * 
 * @param angle     Angle in radians. 
 * 
 * @return          Angle in degrees. */

inline format_t rad2Deg( const format_t angle) { return ( 180.0 / M_PI ) * angle; }

/** Projects angle into the manifold S^1 (the open-closed interval between -PI and +PI). 
 * 
 * @param theta     Angle. 
 * 
 * @return          Projection of angle into S^1. */

inline format_t unwrap( format_t  theta) {
    
    while ( theta <= -M_PI ) { theta += 2 * M_PI; }
    
    while ( theta > M_PI ) { theta -= 2 * M_PI; }
    
    return theta;
    
}

// ------------------------------------------------------------------------
// FUNCTIONS FOR SAMPLING
// ------------------------------------------------------------------------

/** Samples from the uniform distribution supported on the interval [0,1]. 
 * 
 * @return          Real number drawn from the desired distribution. */

inline format_t uniform() { return ( ( (format_t) rand() ) /  RAND_MAX ); }

/** Samples from the univariate Gaussian distribution. 
 * 
 * @param mean      Desired mean. 
 * 
 * @param std_dev   Desired standard deviation. 
 * 
 * @return          Real number drawn from the desired distribution. */

inline format_t gaussian( const format_t mean, const format_t std_dev) {
    
    format_t u = uniform(), v = uniform();
    
    return mean + std_dev * sqrt( -2.0 * log(u) / u ) * cos( 4.0 * M_PI * (v - 0.5) );
    
}

// ------------------------------------------------------------------------
// FUNCTIONS FOR TIME MANAGEMENT
// ------------------------------------------------------------------------

/** Indicates that the desired runtime has been exceeded. 
 * 
 * @param start_time    Starting time. 
 * 
 * @param runtime       Desired runtime, in seconds. 
 * 
 * @return True, if the runtime has been exceeded, false, otherwise. */

inline bool is_time_to_stop( const clock_t start_time, const float runtime) {
    
    clock_t current_time = clock();
    
    return ( ( (float) (current_time - start_time) / CLOCKS_PER_SEC ) > runtime );
    
}

// ------------------------------------------------------------------------
// FUNCTIONS FOR VECTORS (2D only)
// ------------------------------------------------------------------------

/** Flips the sign of the given vector. 
 * 
 * @param vector    Vector. 
 * 
 * @return          Negative of given vector (written on the given vector). */

inline format_t * vec2d_neg( format_t * vector) {
    
    vector[0] = - vector[0];
    vector[1] = - vector[1];
    
    return vector;
    
}

/** Computes Euclidean norm. 
 * 
 * @param vector    Vector. 
 * 
 * @return          Euclidean norm of vector. */

inline format_t vec2d_norm( const format_t * vector) {
    
    return sqrt( (vector[0] * vector[0]) + (vector[1] * vector[1]) );
    
}

/** Computes Euclidean norm of difference of vectors. 
 * 
 * @param vec_0     First vector. 
 * 
 * @param vec_1     Second vector. 
 * 
 * @return          Euclidean norm of difference of vectors. */

inline format_t vec2d_norm( const format_t * vec_0, const format_t * vec_1) {
    
    format_t difference [] = { vec_0[0] - vec_1[0] , vec_0[1] - vec_1[1] };
    
    return vec2d_norm(difference);
    
}

#endif	/* UTILS_ANGLE_VEC_HPP */
