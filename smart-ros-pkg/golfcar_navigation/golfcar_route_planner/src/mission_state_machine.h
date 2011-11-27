#ifndef __MISSION_STATE_MACHINE__HH__
#define __MISSION_STATE_MACHINE__HH__

#include "threaded.h"
#include "route_planner.h"

#include <dbserver_comm/Mission.h>

class DBMissionComm;



/// A base class for mission state machines
class MissionStateMachine : public Threaded
{
    friend class DBMissionComm;

public:
    MissionStateMachine(RoutePlanner & rp)
        : rp_(rp), stationList_(rp.stationList_), state_(sWaitingMission)
    {

    }

    void receivedNewMission(Mission m)
    {
        if( state_ != sWaitingMission ) {
            ROS_DEBUG("Wrong transition");
            return;
        }
        rp_.pickup_ = StationList(m.pickUpLocation);
        rp_.dropoff_ = StationList(m.dropOffLocation);
        if( rp_.currentStation_ !=  rp_.pickup_ ) {
            state_ = sGoingToPickup;
            rp_.state_ = RoutePlanner::sGoingToPickupLocation;
        }
        else {
            state_ = sAtPickup;
            rp_.state_ = RoutePlanner::sAtPickupLocation;
        }
    }

    void reachedStation()
    {
        if( state_ == sGoingToPickup ) {
            state_ = sAtPickup;
            rp_.state_ = RoutePlanner::sAtPickupLocation;
        }
        else if( state_ == sGoingToDropoff ) {
            state_ = sWaitingMission;
            rp_state_ = RoutePlanner::sWaitingForAMission;
        }
        else
            ROS_DEBUG("Wrong transition");
    }

    void passengerBoarded()
    {
        if( state_ == sAtPickup ) {
            state_ = sGoingToDropoff;
            rp_.state_ = RoutePlanner::sGoingToDropOffLocation;
        }
        else
            ROS_DEBUG("Wrong transition");
    }


protected:
    enum State {
        sWaitingMission,
        sGoingToPickup,
        sAtPickup,
        sGoingToDropoff
    };

    RoutePlanner & rp_;
    const StationList & stationList_;
    State state_;
};


#endif
