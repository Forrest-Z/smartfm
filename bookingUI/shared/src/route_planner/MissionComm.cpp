#include <stdlib.h>

#include <iostream>
#include <stdexcept>
using std::cout;
using std::endl;

#include "MissionComm.h"

MissionComm::MissionComm( RoutePlanner & rp, PassengerComm & pc )
    : stationList_(rp.sp_.knownStations()), routePlanner_(rp),
    passengerComm_(pc), state_(sUninit)
{
    stateStr_[sWaitingMission] = "WaitingForAMission";
    stateStr_[sGoingToPickup] = "GoingToPickupLocation";
    stateStr_[sAtPickup] = "AtPickupLocation";
    stateStr_[sGoingToDropoff] = "GoingToDropoffLocation";
    stateStr_[sAtDropoff] = "AtDropoffLocation";
}

void MissionComm::changeState(State new_state)
{
    state_ = new_state;
    updateVehicleStatus(stateStr_[state_]);
}

void MissionComm::run()
{
    switch( state_ )
    {
    case sUninit:
        stationList_.print();
        currentStation_ = stationList_.prompt("Current station? ");
        initialize();
        changeState(sWaitingMission);
        break;

    case sWaitingMission:
        waitForMission();
        if( currentStation_ != pickup_ ) {
            routePlanner_.setPath(currentStation_, pickup_);
            changeState(sGoingToPickup);
            updateCurrentLocation("");
            routePlanner_.start();
        }
        else {
            changeState(sAtPickup);
            updateCurrentLocation(pickup_.str());
        }
        break;

    case sGoingToPickup:
        // TODO: check for mission cancel

        updateETA(routePlanner_.get_ETA());
        if( routePlanner_.hasReached() ) {
            changeState(sAtPickup);
            updateCurrentLocation(pickup_.str());
            currentStation_ = pickup_;
        }
        break;

    case sAtPickup:
        // TODO: check for mission cancel
        passengerComm_.waitForPassengerInAtPickup();
        routePlanner_.setPath(pickup_, dropoff_);
        changeState(sGoingToDropoff);
        updateCurrentLocation("");
        routePlanner_.start();
        break;

    case sGoingToDropoff:
        updateETA(routePlanner_.get_ETA());
        if( routePlanner_.hasReached() ) {
            changeState(sAtDropoff);
            updateCurrentLocation(dropoff_.str());
            currentStation_ = dropoff_;
        }
        break;

    case sAtDropoff:
	updateMissionStatus("completed");
        passengerComm_.waitForPassengerOutAtDropoff();

        // we must wait until the scheduler has acknowledged that the mission is now
        // completed...
        if( checkMissionCompleted() ) changeState(sWaitingMission);
        break;

    default:
        throw std::runtime_error("This state should not happen.");

    }

    updateGeoLocation(routePlanner_.get_lat(), routePlanner_.get_lon());

    sleep(1);
}




void PromptMissionComm::waitForMission()
{
    stationList_.print();
    pickup_ = stationList_.prompt("Pickup station? ");
    dropoff_ = stationList_.prompt("Dropoff station? ");
}

bool PromptMissionComm::checkMissionCompleted() { return true; }

void PromptMissionComm::initialize() { }

void PromptMissionComm::updateMissionStatus(std::string status)
{
    //cout <<"mission status update: " <<status <<endl;
}

void PromptMissionComm::updateVehicleStatus(std::string status)
{
    //cout <<"vehicle status update: " <<status <<endl;
}

void PromptMissionComm::updateGeoLocation(float lat, float lon) { }

void PromptMissionComm::updateETA(float eta)
{
    //cout <<"ETA: " <<eta <<endl;
}

void PromptMissionComm::updateCurrentLocation(std::string loc) { }

bool PromptMissionComm::checkMissionCancelled(unsigned id) { return false; }




DBMissionComm::DBMissionComm(RoutePlanner & rp, PassengerComm & pc, std::string url, std::string vehicleID)
: MissionComm(rp,pc), dbi(url,vehicleID)
{

}

void DBMissionComm::waitForMission()
{
    DBInterface::Task task = dbi.waitForNewMission();
    pickup_ = stationList_(task.pickup);
    dropoff_ = stationList_(task.dropoff);
    currentTaskID_ = task.id;
    dbi.setVehicleCurrReq(task.id);
}

bool DBMissionComm::checkMissionCompleted()
{
    return strcasecmp(dbi.getTaskEntry(currentTaskID_).status.c_str(),"completed")==0;
}

void DBMissionComm::updateMissionStatus(std::string status)
{ dbi.setMissionStatus(currentTaskID_, status); }

void DBMissionComm::updateVehicleStatus(std::string status)
{ dbi.setVehicleStatus(status); }

void DBMissionComm::updateGeoLocation(float lat, float lon)
{ dbi.setGeoLocation(lat,lon); }

void DBMissionComm::updateETA(float eta) { dbi.setETA(eta); }

void DBMissionComm::updateCurrentLocation(std::string loc)
{ dbi.setCurrentLocation(loc); }

bool DBMissionComm::checkMissionCancelled(unsigned id)
{ return dbi.checkMissionCancelled(id); }


void DBMissionComm::initialize()
{
    dbi.deleteVehicle();
    dbi.identify();
    updateCurrentLocation(currentStation_.str());
}
