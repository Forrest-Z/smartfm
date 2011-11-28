#include "mission_state_machine.h"


MissionStateMachine::MissionStateMachine(RoutePlanner & rp)
    : rp_(rp), stationList_(rp.stationList_), state_(sWaitingMission),
      current_(rp.current_), pickup_(rp.pickup_), dropoff_(rp.dropoff_)
{

}


void MissionStateMachine::receivedNewMission(Mission m)
{
    if( state_ != sWaitingMission ) {
        ROS_DEBUG("Wrong transition");
        return;
    }
    rp_.pickup_ = StationList(m.pickUpLocation);
    rp_.dropoff_ = StationList(m.dropOffLocation);
    if( rp_.currentStation_ !=  rp_.pickup_ ) {
        state_ = sGoingToPickup;
        rp_.state_ = RoutePlanner::sGoingToPickupLocation;
    }
    else {
        state_ = sAtPickup;
        rp_.state_ = RoutePlanner::sAtPickupLocation;
    }
}


void MissionStateMachine::reachedStation()
{
    if( state_ == sGoingToPickup ) {
        state_ = sAtPickup;
        rp_.state_ = RoutePlanner::sAtPickupLocation;
    }
    else if( state_ == sGoingToDropoff ) {
        state_ = sWaitingMission;
        rp_state_ = RoutePlanner::sWaitingForAMission;
    }
    else
        ROS_DEBUG("Wrong transition");
}


void MissionStateMachine::passengerBoarded()
{
    if( state_ == sAtPickup ) {
        state_ = sGoingToDropoff;
        rp_.state_ = RoutePlanner::sGoingToDropOffLocation;
    }
    else
        ROS_DEBUG("Wrong transition");
}

