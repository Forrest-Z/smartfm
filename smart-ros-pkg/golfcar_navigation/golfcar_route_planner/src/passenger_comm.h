#ifndef __PASSENGER_COMM__H__
#define __PASSENGER_COMM__H__

#include "threaded.h"

/// A base class to communicate with the passenger
class PassengerComm : public Threaded
{
public:
    virtual void waitForStart() = 0;
};

#endif

