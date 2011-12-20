#include <stdlib.h>

#include <iostream>

#include "MissionComm.h"

MissionComm::MissionComm( RoutePlanner & rp )
    : routePlanner_(rp), stationList_(rp.sp_.knownStations()),
      currentStation_(rp.currentStation_), state_(sUninit)
{
    stateStr_.push_back("WaitingForAMission");
    stateStr_.push_back("GoingToPickup");
    stateStr_.push_back("GoingToDropoff");
    stateStr_.push_back("AtPickup");
}

void MissionComm::run()
{
    switch( state_ )
    {
    case sUninit:
        stationList_.print();
        currentStation_ = stationList_.prompt("Current station? ");
        identify();
        updateCurrentLocation(currentStation_.str());
        state_ = sWaitingMission;
        break;
        
    case sWaitingMission:
        waitForMission();
        updateMissionStatus("Processing");
        if( currentStation_ != pickup_ ) {
            routePlanner_.setDestination(pickup_);
            state_ = sGoingToPickup;
            updateVehicleStatus("GoingToPickupLocation");
            updateCurrentLocation("");
            routePlanner_.start();
        }
        else {
            state_ = sAtPickup;
            updateVehicleStatus("AtPickupLocation");
            updateCurrentLocation(pickup_.str());
        }
        break;

    case sGoingToPickup:
        // TODO: check for mission cancel

    	updateETA(routePlanner_.eta_);
    	updateGeoLocation(routePlanner_.latitude_, routePlanner_.longitude_);

        if( routePlanner_.hasReached() ) {
            state_ = sAtPickup;
            updateVehicleStatus("AtPickupLocation");
            updateCurrentLocation(pickup_.str());
        }
        break;

    case sAtPickup:
        // TODO: check for mission cancel
        // TODO: wait for passenger to board
        routePlanner_.setDestination(dropoff_);
        state_ = sGoingToDropoff;
        updateVehicleStatus("GoingToDropoffLocation");
        updateCurrentLocation("");
        routePlanner_.start();
        break;

    case sGoingToDropoff:
    	updateETA(routePlanner_.eta_);
    	updateGeoLocation(routePlanner_.latitude_, routePlanner_.longitude_);
        if( routePlanner_.hasReached() ) {
            // TODO: wait for passenger to alight
        	updateMissionStatus("Completed");
        	updateCurrentLocation(dropoff_.str());
        	updateVehicleStatus("WaitingForAMission");
            state_ = sWaitingMission;
        }
        break;
    }

    sleep(1);
}




void PromptMissionComm::waitForMission()
{
    stationList_.print();
    pickup_ = stationList_.prompt("Pickup station? ");
    dropoff_ = stationList_.prompt("Dropoff station? ");
}

void PromptMissionComm::updateStatus()
{
    std::cout <<"New status: " <<stateStr_[state_] <<std::endl;
}




DBMissionComm::DBMissionComm(RoutePlanner & rp, std::string url, std::string vehicleID)
  : MissionComm(rp), dbi(url,vehicleID)
{

}

void DBMissionComm::waitForMission()
{
	DBInterface::Task task = dbi.waitForNewMission();
    pickup_ = stationList_(task.pickup);
    dropoff_ = stationList_(task.dropoff);
    currentTaskID_ = task.id;
}
