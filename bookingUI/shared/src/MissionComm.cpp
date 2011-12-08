#include <stdlib.h>

#include <iostream>

#include "MissionComm.h"

MissionComm::MissionComm( RoutePlanner & rp )
    : routePlanner_(rp), stationList_(rp.sp_.knownStations()),
      currentStation_(rp.currentStation_), state_(sWaitingMission)
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
    case sWaitingMission:
        waitForMission();
        if( currentStation_ != pickup_ ) {
            routePlanner_.setDestination(pickup_);
            state_ = sGoingToPickup;
        }
        else {
            state_ = sAtPickup;
        }
        updateStatus();
        break;

    case sGoingToPickup:
        // TODO: check for mission cancel

        if( routePlanner_.hasReached() ) {
            state_ = sAtPickup;
            updateStatus();
        }
        break;

    case sAtPickup:
        // TODO: check for mission cancel
        // TODO: wait for passenger to board
        routePlanner_.setDestination(dropoff_);
        state_ = sGoingToDropoff;
        updateStatus();
        break;

    case sGoingToDropoff:
        if( routePlanner_.hasReached() ) {
            // TODO: wait for passenger to alight
            state_ = sWaitingMission;
            updateStatus();
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