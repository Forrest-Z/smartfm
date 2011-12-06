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

    /// Updates the DB with the current status of the vehicle and mission
    virtual void updateStatus() = 0;

    /// Checks the DB for any pending mission.
    virtual void waitForMission() = 0;
};


/// Interface with the user via a text interface (simulates a DB access).
class PromptMissionComm : public DBMissionComm
{
public:
    PromptMissionComm(RoutePlanner & rp) : DBMissionComm(rp) { }

private:
    void updateStatus();
    void waitForMission();
};


/// Implements communication with the database via the Python ROS node.
class DBServerMissionComm : public DBMissionComm
{
public:
    DBServerMissionComm(RoutePlanner &);

private:
    ros::Publisher pub;
    ros::ServiceClient client;

    dbserver_comm::Mission *currentMission_;

    void updateStatus();
    void waitForMission();
    void updateMission(const std::string &, const std::string &);
};

#endif //__DB_MISSION_COMM__H__
