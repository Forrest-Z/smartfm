#ifndef __PASSENGER_COMM__H__
#define __PASSENGER_COMM__H__

#include <stdlib.h>
#include <iostream>
#include <limits>

/// A base class to communicate with the passenger
class PassengerComm
{
public:
    /// Called when the vehicle is at the pickup location and waiting for the
    /// passenger. Blocks until the passenger has boarded the vehicle.
    virtual void waitForPassengerInAtPickup() = 0;

    /// Called when the vehicle is at the dropoff location and waiting for the
    /// passenger to alight. Blocks until the passenger has exited the vehicle.
    virtual void waitForPassengerOutAtDropoff() = 0;
};


/// Do not interface with the passenger, simply wait for a few seconds.
class DummyPassengerComm : public PassengerComm
{
public:
    void waitForPassengerInAtPickup();
    void waitForPassengerOutAtDropoff();
};


/// Interface with the passenger using a text interface: prompts for passenger
/// to press a key when ready.
class SimplePassengerComm : public PassengerComm
{
public:
    void waitForPassengerInAtPickup();
    void waitForPassengerOutAtDropoff();
};

#endif

