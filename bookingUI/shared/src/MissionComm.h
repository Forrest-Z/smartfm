#ifndef __MISSION_COMM_H__
#define __MISSION_COMM_H__

#include <vector>
#include <string>

#include "RoutePlanner.h"
#include "DBInterface.h"

/// A base class to get and receive missions.
class MissionComm : public Threaded
{
	friend class RoutePlanner;

public:
    MissionComm( RoutePlanner & rp );

protected:
    enum State { sUninit, sWaitingMission, sGoingToPickup, sGoingToDropoff, sAtPickup };

    RoutePlanner & routePlanner_;
    const StationList & stationList_;
    Station & currentStation_;

    State state_;
    Station pickup_, dropoff_;
    std::vector<std::string> stateStr_;

    void run();

    virtual void identify() { };
    virtual void deidentify() { };
    virtual void updateMissionStatus(std::string status) { }
    virtual void updateVehicleStatus(std::string status) { }
    virtual void updateGeoLocation(float lat, float lon) { }
    virtual void updateETA(float eta) { }
    virtual void updateCurrentLocation(std::string loc) { }
    virtual bool checkMissionCancelled(unsigned id) { return false; }
    
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

/// Interface with the database using a DBInterface object
class DBMissionComm : public MissionComm
{
    DBInterface dbi;
    unsigned currentTaskID_;

public:
    DBMissionComm(RoutePlanner & rp, std::string url, std::string vehicleID);

private:
    void identify() { dbi.identify(); }
    void deidentify() { dbi.deleteVehicle(); }
    void updateMissionStatus(std::string status) { dbi.setMissionStatus(currentTaskID_, status); }
    void updateVehicleStatus(std::string status) { dbi.setVehicleStatus(status); }
    void updateGeoLocation(float lat, float lon) { dbi.setGeoLocation(lat,lon); }
    void updateETA(float eta) { dbi.setETA(eta); }
    void updateCurrentLocation(std::string loc) { dbi.setCurrentLocation(loc); }
    bool checkMissionCancelled(unsigned id) { return dbi.checkMissionCancelled(id); }
    void waitForMission();
};

#endif
