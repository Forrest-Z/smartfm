#ifndef __MISSION_COMM_H__
#define __MISSION_COMM_H__

#include <vector>
#include <string>

#include "RoutePlanner.h"

/// A base class to get and receive missions.
class MissionComm : public Threaded
{
public:
    MissionComm( RoutePlanner & rp );

protected:
    enum State { sWaitingMission, sGoingToPickup, sGoingToDropoff, sAtPickup };

    RoutePlanner & routePlanner_;
    const StationList & stationList_;
    const Station & currentStation_;

    State state_;
    Station pickup_, dropoff_;
    std::vector<std::string> stateStr_;

    void run();

    /// Updates the DB with the current status of the vehicle and mission
    virtual void updateStatus() = 0;

    /// Checks the DB for any pending mission.
    virtual void waitForMission() = 0;
};


/// Interface with the user via a text interface (simulates a DB access).
class PromptMissionComm : public MissionComm
{
public:
    PromptMissionComm(RoutePlanner & rp) : MissionComm(rp) { }

private:
    void updateStatus();
    void waitForMission();
};

#endif
