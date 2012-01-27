#ifndef __FMUTIL_MATH_H__
#define __FMUTIL_MATH_H__

#include <math.h>
#include <string>


#define BOUND(m,x,M) ((x)<(m) ? (m) : (x)>(M) ? (M) : (x))
#define SYMBOUND(x,m) BOUND(-(m),x,m)

#define SIGN(a) ( (a)>0 ? 1 : (a)<0 ? -1 : 0)


namespace fmutil {

/// Computes the corresponding angle on ]-180,180].
float angMod180(float ang);

/// Computes the corresponding angle on [0,360[.
float angMod360(float ang);

/// Computes the corresponding angle on ]-pi,pi].
float angModPI(float ang);

/// Computes the corresponding angle on ]-pi/2,pi/2].
float angModPI2(float ang);

/// Computes the corresponding angle on ]0,2*pi].
float angMod2PI(float ang);

/**
 * Computes the angular distance between 2 angles: fabs(angModPI(a-b))
 * @param a in radians
 * @param b in radians
 * @return fabs(angModPI(a-b))
 */
float angDist(float a, float b);

///converts degrees to radians
float d2r(float ang);

///converts radians to degrees
float r2d(float ang);

/// returns the euclidian norm (magnitude) of vector [a b (c)]
/// as sqrt(a^2+b^2+c^2)
float mag(float a, float b, float c=0);

/// Saturation function: returns -1 if x < -s, 1 if x > s, x/s otherwise.
float sat(float x, float s);

/// returns the euclidian distance between points (x1,y1) and (x2,y2)
float distance(float x1, float y1, float x2, float y2);

/// returns the angle of the vector from (x1,y1) to (x2,y2)
/// i.e. atan2f(y1-y2,x1-x2)
float angle(float x1, float y1, float x2, float y2);

/// returns the minimum of the 3 numbers. Can also be used with only 2 numbers.
float min(float a, float b, float c=INFINITY);

/// returns the maximum of the 3 numbers. Can also be used with only 2 numbers.
float max(float a, float b, float c=-INFINITY);

/// checks whether x is within min(a,b) and max(a,b)
bool isWithin(float x, float a, float b);

/// works like printf but returns a string with the formatted output instead of
/// printing to stdout
std::string stringPrintf(const char *fmt, ...);


} //namespace fmutil

#endif //__FMUTIL_MATH_H__
