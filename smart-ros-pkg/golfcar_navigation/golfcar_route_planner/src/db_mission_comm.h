#ifndef __DB_MISSION_COMM__H__
#define __DB_MISSION_COMM__H__

#include "threaded.h"
#include "mission_state_machine.h"
#include "station_path.h"

/// A base class to get and receive missions.
class DBMissionComm : public Threaded
{
public:
    DBMissionComm( MissionStateMachine & stateMachine )
        : stateMachine_(stateMachine), stationList_(stateMachine_.stationList_)
    {

    }

protected:
    const StationList & stationList_;
    MissionStateMachine & stateMachine_;
};

#endif __DB_MISSION_COMM__H__
