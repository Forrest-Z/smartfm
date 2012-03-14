#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <stdarg.h> //For variable number of arguments
#include <string.h>
#include <math.h>

#include <string>
#include <iostream>

#include <fmutil/fm_math.h>

namespace fmutil {

double angMod180(double ang) {
  double a = ang;
  while( a <= -180 ) a+=360.0;
  while( a > 180 )   a-=360.0;
  return a;
}

double angMod360(double ang) {
  double a = ang;
  while( a < 0 ) a+=360.0;
  while( a >= 360 ) a-=360.0;
  return a;
}

double angModPI(double ang) {
  double a = ang;
  while( a <= -M_PI ) a+=2.0*M_PI;
  while( a > M_PI )   a-=2.0*M_PI;
  return a;
}

double angModPI2(double ang) {
  double a = ang;
  while( a <= -M_PI_2 ) a+=M_PI;
  while( a > M_PI_2 )   a-=M_PI;
  return a;
}

double angMod2PI(double ang) {
  double a = ang;
  while( a <= 0 ) a+=2.0*M_PI;
  while( a > 2*M_PI ) a-=2.0*M_PI;
  return a;
}

double angDist(double a, double b) {
  return fabs(angModPI(a-b));
}

double d2r(double ang) {
  return ang/180.0*M_PI;
}

double r2d(double ang) {
  return ang/M_PI*180.0;
}

double mag(double a, double b, double c) {
  return sqrt(a*a + b*b + c*c);
}

double distance(double x1, double y1, double x2, double y2) {
  return mag(x1-x2, y1-y2);
}

double distance(const geometry_msgs::Point & p1,
		const geometry_msgs::Point & p2)
{
	return mag(p1.x-p2.x, p1.y-p2.y);
}

double angle(double x1, double y1, double x2, double y2) {
  return atan2f(y1-y2, x1-x2);
}

double min(double a, double b, double c) {
  return a<b ? (a<c ? a : c) : (b<c ? b : c);
}

double max(double a, double b, double c) {
  return a>b ? (a>c ? a : c) : (b>c ? b : c);
}

bool isWithin(double x, double a, double b)  {
  return min(a,b)<x && x<max(a,b);
}

std::string stringPrintf(const char *fmt, ...) {
  va_list ap;
  int n, size = 100;
  char *p, *np;

  p = (char *) malloc(size);
  assert( p!=NULL );

  va_start(ap,fmt); //point to first element after fmt
  while( true ) {
    /* Try to print in the allocated space. */
    n = vsnprintf(p, size, fmt, ap);
    /* If that worked, return the string. */
    if (n > -1 && n < size)
      break;
    /* Else try again with more space. */
    if (n > -1)    /* glibc 2.1 */
      size = n+1; /* precisely what is needed */
    else           /* glibc 2.0 */
      size *= 2;  /* twice the old size */
    if ((np = (char *)realloc (p, size)) == NULL) {
      free(p);
      std::cerr << "%s\n" << strerror(errno);
    } else {
      p = np;
    }
  }
  va_end(ap);
  assert( p!=NULL );
  std::string msg(p);
  free(p);
  return msg;
}

} //namespace fmutil
