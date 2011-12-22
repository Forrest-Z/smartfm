#ifndef __GLOBAL_CLOCK_H__
#define __GLOBAL_CLOCK_H__

#include <string>

/// Provides the time elapsed since initialization time
class GlobalClock
{
public:
    /// Returns the time in seconds since program started.
    static double time();

    /// Returns the time since the program started as a string.
    static std::string timeStr();

    /// Returns the time passed as argument as a string.
    static std::string timeStr(double);

    /** Sleep for t seconds. Uses nanosleep.
    * @param t the amount of time to sleep in seconds.
    * @returns On successfully sleeping for the requested interval, sleep()
    * returns 0.  If the call is interrupted by a signal handler or encounters
    * an error,  then  it  returns -1, with errno set to indicate the error.
    */
    static int sleep(float t);

    /** Sleep for t milliseconds. Uses nanosleep.
    * @param t the amount of time to sleep in milliseconds.
    * @returns On successfully sleeping for the requested interval, sleep()
    * returns 0.  If the call is interrupted by a signal handler or encounters
    * an error,  then  it  returns -1, with errno set to indicate the error.
    */
    static int sleep_ms(float t) { return GlobalClock::sleep(0.001*t); }

private:
    static void timeval_subtract (struct timeval *result,
                                const struct timeval *x,
                                const struct timeval *y);
    GlobalClock() { }
    GlobalClock(const GlobalClock&) { }
    GlobalClock& operator=(const GlobalClock&) { return *this; }
};

#endif
