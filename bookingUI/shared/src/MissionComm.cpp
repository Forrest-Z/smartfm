#include <stdlib.h>

#include <iostream>
#include <stdexcept>

#include "MissionComm.h"

MissionComm::MissionComm( RoutePlanner & rp, PassengerComm & pc )
    : routePlanner_(rp), stationList_(rp.sp_.knownStations()),
    currentStation_(rp.currentStation_), passengerComm_(pc), state_(sUninit)
{
    stateStr_[sWaitingMission] = "WaitingForAMission";
    stateStr_[sGoingToPickup] = "GoingToPickupLocation";
    stateStr_[sAtPickup] = "AtPickupLocation";
    stateStr_[sGoingToDropoff] = "GoingToDropoffLocation";
    stateStr_[sAtDropoff] = "AtDropoffLocation";
}

void MissionComm::run()
{
    switch( state_ )
    {
    case sUninit:
        stationList_.print();
        currentStation_ = stationList_.prompt("Current station? ");
        deidentify();
        identify();
        updateCurrentLocation(currentStation_.str());
        state_ = sWaitingMission;
        break;

    case sWaitingMission:
        waitForMissionConfirmed();
        if( currentStation_ != pickup_ ) {
            routePlanner_.setDestination(pickup_);
            state_ = sGoingToPickup;
            updateVehicleStatus(stateStr_[state_]);
            updateCurrentLocation("");
            routePlanner_.start();
        }
        else {
            state_ = sAtPickup;
            updateVehicleStatus(stateStr_[state_]);
            updateCurrentLocation(pickup_.str());
        }
        break;

    case sGoingToPickup:
        // TODO: check for mission cancel

        updateETA(routePlanner_.eta_);
        if( routePlanner_.hasReached() ) {
            state_ = sAtPickup;
            updateVehicleStatus(stateStr_[state_]);
            updateCurrentLocation(pickup_.str());
        }
        break;

    case sAtPickup:
        // TODO: check for mission cancel
        passengerComm_.waitForPassengerInAtPickup();
        routePlanner_.setDestination(dropoff_);
        state_ = sGoingToDropoff;
        updateVehicleStatus(stateStr_[state_]);
        updateCurrentLocation("");
        routePlanner_.start();
        break;

    case sGoingToDropoff:
        updateETA(routePlanner_.eta_);
        if( routePlanner_.hasReached() ) {
            state_ = sAtDropoff;
            updateVehicleStatus(stateStr_[state_]);
            updateCurrentLocation(dropoff_.str());
        }
        break;

    case sAtDropoff:
        passengerComm_.waitForPassengerOutAtDropoff();
        if( checkMissionCompleted() ) {
            state_ = sWaitingMission;
            updateVehicleStatus(stateStr_[state_]);
        }
        break;

    default:
        throw std::runtime_error("This state should not happen.");

    }

    updateGeoLocation(routePlanner_.latitude_, routePlanner_.longitude_);

    sleep(1);
}




void PromptMissionComm::waitForMissionConfirmed()
{
    stationList_.print();
    pickup_ = stationList_.prompt("Pickup station? ");
    dropoff_ = stationList_.prompt("Dropoff station? ");
}

void PromptMissionComm::updateStatus()
{
    std::cout << "New status: " << stateStr_[state_] <<std::endl;
}




DBMissionComm::DBMissionComm(RoutePlanner & rp, PassengerComm & pc, std::string url, std::string vehicleID)
: MissionComm(rp,pc), dbi(url,vehicleID)
{

}

void DBMissionComm::waitForMissionConfirmed()
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
