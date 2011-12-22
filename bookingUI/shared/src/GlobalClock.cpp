#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <stdio.h>

#include "GlobalClock.h"

int GlobalClock::sleep(float t) {
  if( t<=0 ) return 0;

  struct timespec delay;
  delay.tv_sec = (time_t) floor(t);
  delay.tv_nsec = (long) ( (t - delay.tv_sec) * 1e9 );

  return nanosleep(&delay,NULL);
}


double GlobalClock::time() {
  static struct timeval TimeZero;
  static bool firstCall = true;

  if( firstCall ) {
    gettimeofday(&TimeZero,NULL);
    firstCall = false;
  }

  struct timeval now, diff;
  gettimeofday(&now,NULL);
  timeval_subtract(&diff,&now,&TimeZero);
  return (double)diff.tv_sec + (double)(diff.tv_usec)*1e-6;
}

std::string GlobalClock::timeStr() {
  return timeStr(time());
}

std::string GlobalClock::timeStr(double t) {
  double h = floor(t / 3600);
  double m = floor((t - h*3600) / 60);
  double s = floor(t - h*3600 - m*60);
  double ms = floor((t - h*3600 - m*60 - s)*1e3);
  double us = floor((t - h*3600 - m*60 - s - ms/1e3)*1e6);
  char timebuf[20];
  snprintf(timebuf,20,"%02dh%02dm%02ds:%03d%03d", (int)h, (int)m, (int)s,
           (int)ms, (int)us);
  return std::string(timebuf);
}


void GlobalClock::timeval_subtract (struct timeval *result,
                                   const struct timeval *x,
                                   const struct timeval *y)
{
   struct timeval yy;
   yy.tv_sec = y->tv_sec;
   yy.tv_usec= y->tv_usec;
   /* Perform the carry for the later subtraction by updating y. */
   if (x->tv_usec < yy.tv_usec) {
      int nsec = (yy.tv_usec - x->tv_usec) / 1000000 + 1;
      yy.tv_usec -= 1000000 * nsec;
      yy.tv_sec += nsec;
   }
   if (x->tv_usec - yy.tv_usec > 1000000) {
      int nsec = (x->tv_usec - yy.tv_usec) / 1000000;
      yy.tv_usec += 1000000 * nsec;
      yy.tv_sec -= nsec;
   }

   /* Compute the time remaining to wait.
   tv_usec is certainly positive. */
   result->tv_sec = x->tv_sec - yy.tv_sec;
   result->tv_usec = x->tv_usec - yy.tv_usec;
}
