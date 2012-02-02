#ifndef __FM_FILTER__H__
#define __FM_FILTER__H__

/// Author: Brice Rebsamen, Feb 2012

#include <cassert>

namespace fmutil {

/// Implement a low pass filter
///
/// Jeong Hwan's implementation, improved.
class LowPassFilter
{
    double y; ///< The filtered value
    double t; ///< The time of the last input
    double tau; ///< The constant of the filter
    bool initialized; ///< Whether the filter has been initialized

public:

    /// Construct the filter
    /// @arg tau: the time constant of the filter
    LowPassFilter(double tau)
    {
        assert(tau>0);
        this->tau = tau;
        reset();
    }

    /// Reset the filter
    void reset()
    {
        this->initialized = false;
    }

    /// Add a value to the filter
    /// @arg t: the time of the value in seconds
    /// @arg x: the value to filter
    /// @return the filtered value
    double filter(double t, double x)
    {
        if( ! this->initialized ) {
            this->initialized = true;
            this->y = x;
            this->t = t;
            return x;
        }

        double dt = t - this->t;
        this->t = t;

        if( dt > this->tau ) {
            // filter has not been updated for a long time
            // need to reset it or value can jump very far away
            this->y = x;
            return x;
        }

        this->y = (x - this->y) * dt / this->tau;
        return this->y;
    }

    /// Add a value to the filter
    /// @arg dt: the time since the last value in seconds
    /// @arg x: the value to filter
    /// @return the filtered value
    double filter_dt(double dt, double x)
    {
        if( ! this->initialized ) {
            this->initialized = true;
            this->t = 0;
            this->y = x;
            return x;
        }

        return filter(this->t+dt, x);
    }

};

} //namespace fmutil


#endif //__FM_FILTER__H__