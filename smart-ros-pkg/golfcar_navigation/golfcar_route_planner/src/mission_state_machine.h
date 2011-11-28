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
    MissionStateMachine(RoutePlanner & rp);

    void receivedNewMission(dbserver_comm::Mission m);
    void reachedStation();
    void passengerBoarded();


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

    Station & current_, pickup_, dropoff_;
};


#endif
