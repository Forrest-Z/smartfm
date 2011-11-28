#include <ros/ros.h>

#include "db_mission_comm.h"


DBMissionComm::DBMissionComm( RoutePlanner & rp )
    : routePlanner_(rp), stationList_(rp.sp_.knownStations()),
      currentStation_(rp.currentStation_), state_(sWaitingMission)
{
    stateStr_.push_back("WaitingForAMission");
    stateStr_.push_back("GoingToPickup");
    stateStr_.push_back("GoingToDropoff");
    stateStr_.push_back("AtPickup");

    startThread();
}

void DBMissionComm::run()
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
        if( routePlanner_.hasReached() ) {
            state_ = sAtPickup;
            updateStatus();
        }
        break;

    case sGoingToDropoff:
        if( routePlanner_.hasReached() ) {
            state_ = sWaitingMission;
            updateStatus();
        }
        break;

    case sAtPickup:
        routePlanner_.setDestination(dropoff_);
        state_ = sGoingToDropoff;
        updateStatus();
        break;
    }

    ros::Duration(1).sleep();
}



void PromptMissionComm::waitForMission()
{
    stationList_.print();
    pickup_ = stationList_.prompt("Pickup station? ");
    dropoff_ = stationList_.prompt("Dropoff station? ");
}

void PromptMissionComm::updateStatus()
{
    ROS_INFO("New status: %s", stateStr_[state_].c_str());
}



DBServerMissionComm::DBServerMissionComm(RoutePlanner & rp)
    : DBMissionComm(rp)
{
    sub = n.subscribe("missions/assignments", 1, &DBServerMissionComm::missionCB, this);
    pub = n.advertise<dbserver_comm::Mission>("missions/feedback", 1);
}

void DBServerMissionComm::missionCB( const dbserver_comm::Mission & m )
{

}

void DBServerMissionComm::updateStatus()
{

}

void DBServerMissionComm::waitForMission()
{

}
