#ifndef __DB_MISSION_COMM__H__
#define __DB_MISSION_COMM__H__

#include <golfcar_route_planner/station_path.h>
#include <dbserver_comm/Mission.h>

#include "threaded.h"
#include "mission_state_machine.h"

/// A base class to get and receive missions.
class DBMissionComm : public Threaded
{
public:
    DBMissionComm( MissionStateMachine & sm )
        : stateMachine_(sm), stationList_(sm.stationList_),
          current_(sm.current_), pickup_(sm.pickup_), dropoff_(sm.dropoff_)
    {

    }

protected:
    MissionStateMachine & stateMachine_;
    const StationList & stationList_;
    Station & current_, pickup_, dropoff_;
};


class PromptMissionComm : public DBMissionComm
{
public:
    dbserver_comm::Mission waitForNewMission()
    {
        dbserver_comm::Mission m;
        return m;
    }

    void run();
};


class DBServerMissionComm : public DBMissionComm
{
public:

    DBServerMissionComm(MissionStateMachine & sm);

    void missionCB( const dbserver_comm::Mission & m );

    dbserver_comm::Mission waitForNewMission();


private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
};

#endif //__DB_MISSION_COMM__H__
