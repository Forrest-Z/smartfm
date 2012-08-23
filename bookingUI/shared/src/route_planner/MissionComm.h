#ifndef __MISSION_COMM_H__
#define __MISSION_COMM_H__

#include <map>
#include <string>

#include "RoutePlanner.h"
#include "DBInterface.h"
#include "PassengerComm.h"

/** A state machine and abstract base class to get and receive missions. */
class MissionComm : public Threaded
{
public:
    /** Constructor
     *
     * A RoutePlanner and a PassengerComm are required for collaboration.
     */
    MissionComm( RoutePlanner & rp, PassengerComm & pc );

protected:
    /// The list of stations (reference to the one in routePlanner_)
    const StationList & stationList_;

    /// The pickup, dropoff and current stations
    Station pickup_, dropoff_, currentStation_;

    /// Blocks until a mission is available. When a mission is obtained,
    /// it should set pickup_ and dropoff_.
    virtual void waitForMission() = 0;

    /// This is called when at the dropoff station before going to the
    /// sWaitingMission state. It is useful when communicating with the scheduler,
    /// to make sure that it has aknowledged that the mission has ended.
    virtual bool checkMissionCompleted() = 0;

    /// This is called in the sUninit state, after prompting for the initial
    /// currentStation_ and before moving to the sWaitingMission state.
    virtual void initialize() = 0;

    virtual void updateMissionStatus(std::string status) = 0;
    virtual void updateVehicleStatus(std::string status) = 0;
    virtual void updateGeoLocation(float lat, float lon) = 0;
    virtual void updateETA(float eta) = 0;
    virtual void updateCurrentLocation(std::string loc) = 0;
    virtual bool checkMissionCancelled(unsigned id) = 0;

private:
    enum State { sUninit, sWaitingMission, sGoingToPickup, sAtPickup, sGoingToDropoff, sAtDropoff };

    /// The RoutePlanner object we are collaborating with
    RoutePlanner & routePlanner_;

    /// The PassengerComm object we are collaborating with
    PassengerComm & passengerComm_;

    /// The current state
    State state_;

    /// mapping between state id and its string representation
    std::map<State, std::string> stateStr_;

    /// Set the new state and call updateVehicleStatus()
    void changeState(State new_state);

    /// The main thread function and the actual implementation of the state machine
    void run();
};


/// Interface with the user via a text interface (simulates a DB access).
class PromptMissionComm : public MissionComm
{
public:
    PromptMissionComm(RoutePlanner & rp, PassengerComm & pc) : MissionComm(rp,pc) { }

private:
    void waitForMission();
    bool checkMissionCompleted();
    void initialize();
    void updateMissionStatus(std::string status);
    void updateVehicleStatus(std::string status);
    void updateGeoLocation(float lat, float lon);
    void updateETA(float eta);
    void updateCurrentLocation(std::string loc);
    bool checkMissionCancelled(unsigned id);
};

/// Interface with the database using a DBInterface object
class DBMissionComm : public MissionComm
{
    DBInterface dbi;
    unsigned currentTaskID_;

public:
    DBMissionComm(RoutePlanner & rp, PassengerComm & pc, std::string url, std::string vehicleID);

private:
    void initialize();
    void updateMissionStatus(std::string status);
    void updateVehicleStatus(std::string status);
    void updateGeoLocation(float lat, float lon);
    void updateETA(float eta);
    void updateCurrentLocation(std::string loc);
    bool checkMissionCancelled(unsigned id);
    void waitForMission();
    bool checkMissionCompleted();
};

#endif
