#ifndef __DB_MISSION_COMM__H__
#define __DB_MISSION_COMM__H__

#include <vector>
#include <string>

#include <golfcar_route_planner/station_path.h>
#include <dbserver_comm/Mission.h>

#include "threaded.h"
#include "route_planner.h"

/// A base class to get and receive missions.
class DBMissionComm : protected Threaded
{
public:
    DBMissionComm( RoutePlanner & rp );

protected:
    enum State { sWaitingMission, sGoingToPickup, sGoingToDropoff, sAtPickup };

    ros::NodeHandle n;
    RoutePlanner & routePlanner_;
    const StationList & stationList_;
    const Station & currentStation_;

    std::vector<std::string> stateStr_;
    Station pickup_, dropoff_;
    State state_;

    void run();
    virtual void updateStatus() = 0;
    virtual void waitForMission() = 0;
};


class PromptMissionComm : public DBMissionComm
{
public:
    PromptMissionComm(RoutePlanner & rp) : DBMissionComm(rp) { }

private:
    void updateStatus();
    void waitForMission();
};


class DBServerMissionComm : public DBMissionComm
{
public:
    DBServerMissionComm(RoutePlanner &);

private:
    ros::Publisher pub;
    ros::Subscriber sub;

    void missionCB( const dbserver_comm::Mission & m );
    void updateStatus();
    void waitForMission();
};

#endif //__DB_MISSION_COMM__H__
