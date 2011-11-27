#ifndef __ROUTE_PLANNER__H__
#define __ROUTE_PLANNER__H__


#include "threaded.h"
#include "station_path.h"


class MissionStateMachine;
class DBMissionComm;

/// A base class for route planners
class RoutePlanner : public Threaded
{
    friend class MissionStateMachine;
    friend class DBMissionComm;

public:
    const StationList & stationList_;

    RoutePlanner(StationPaths & sp) : sp_(sp), stationList_(sp.knowStations()) { }

protected:
    enum State {
        sNotAvailable,
        sGoingToPickupLocation,
        sGoingToDropOffLocation,
        sAtPickupLocation,
        sWaitingForAMission
    };

    StationPaths & sp_;
    Station currentStation_, pickup_, dropoff_;
    State state_;
};

#endif
