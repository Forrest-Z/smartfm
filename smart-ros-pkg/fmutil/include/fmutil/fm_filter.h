#ifndef __FM_FILTER__H__
#define __FM_FILTER__H__

/// Author: Brice Rebsamen, Feb 2012

#include <stdexcept>

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

    LowPassFilter()
    {
        this->tau = 0;
        reset();
    }

    /// Construct the filter
    /// @arg tau: the time constant of the filter
    LowPassFilter(double tau)
    {
        if( tau<=0 )
            throw std::invalid_argument("Filter's time constant (tau) must be > 0");
        this->tau = tau;
        reset();
    }

    /// Returns the current value of the filter. Throws a runtime_error if
    /// the filter has not been initialized.
    double value() const
    {
        if( ! this->initialized )
            throw std::runtime_error("Trying to get the value from an uninitialized filter");
        return this->y;
    }

    /// Sets the current value of the filter (and time to 0)
    double value(double x)
    {
        this->initialized = true;
        this->t = 0;
        this->y = x;
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
        if( this->tau==0 )
            throw std::runtime_error("Filter's time constant (tau) was not set");

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

        this->y += (x - this->y) * dt / this->tau;
        return this->y;
    }

    /// Add a value to the filter
    /// @arg dt: the time since the last value in seconds
    /// @arg x: the value to filter
    /// @return the filtered value
    double filter_dt(double dt, double x)
    {
        if( this->tau==0 )
            throw std::runtime_error("Filter's time constant (tau) was not set");

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
