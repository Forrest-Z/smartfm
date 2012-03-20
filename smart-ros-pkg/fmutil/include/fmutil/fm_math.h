#ifndef __FMUTIL_MATH_H__
#define __FMUTIL_MATH_H__

#include <math.h>
#include <string>

#include <geometry_msgs/Point.h>


#define BOUND(m,x,M) ((x)<(m) ? (m) : (x)>(M) ? (M) : (x))
#define SYMBOUND(x,m) BOUND(-(m),x,m)

#define SIGN(a) ( (a)>0 ? 1 : (a)<0 ? -1 : 0)


namespace fmutil {

/// Computes the corresponding angle on ]-180,180].
double angMod180(double ang);

/// Computes the corresponding angle on [0,360[.
double angMod360(double ang);

/// Computes the corresponding angle on ]-pi,pi].
double angModPI(double ang);

/// Computes the corresponding angle on ]-pi/2,pi/2].
double angModPI2(double ang);

/// Computes the corresponding angle on ]0,2*pi].
double angMod2PI(double ang);

/**
 * Computes the angular distance between 2 angles: fabs(angModPI(a-b))
 * @param a in radians
 * @param b in radians
 * @return fabs(angModPI(a-b))
 */
double angDist(double a, double b);

///converts degrees to radians
double d2r(double ang);

///converts radians to degrees
double r2d(double ang);

/// returns the euclidian norm (magnitude) of vector [a b (c)]
/// as sqrt(a^2+b^2+c^2)
double mag(double a, double b, double c=0);

/// Saturation function: returns -1 if x < -s, 1 if x > s, x/s otherwise.
double sat(double x, double s);

/// returns the euclidian distance between points (x1,y1) and (x2,y2)
double distance(double x1, double y1, double x2, double y2);

/// returns the euclidian distance between points
double distance(const geometry_msgs::Point & p1,
		const geometry_msgs::Point & p2);

/// returns the angle of the vector from (x1,y1) to (x2,y2)
/// i.e. atan2f(y1-y2,x1-x2)
double angle(double x1, double y1, double x2, double y2);

/// returns the minimum of the 3 numbers. Can also be used with only 2 numbers.
double min(double a, double b, double c=INFINITY);

/// returns the maximum of the 3 numbers. Can also be used with only 2 numbers.
double max(double a, double b, double c=-INFINITY);

/// checks whether x is within min(a,b) and max(a,b)
bool isWithin(double x, double a, double b);

/// works like printf but returns a string with the formatted output instead of
/// printing to stdout
std::string stringPrintf(const char *fmt, ...);


} //namespace fmutil

#endif //__FMUTIL_MATH_H__
