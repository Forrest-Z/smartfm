#include <ros/ros.h>
#include <dbserver_comm/Mission.h>
#include <dbserver_comm/MissionRequest.h>
#include "db_mission_comm.h"

using std::string;

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
    : DBMissionComm(rp), currentMission_(0)
{
    client = n.serviceClient<dbserver_comm::MissionRequest>("missions/assignments");
    pub = n.advertise<dbserver_comm::Mission>("missions/feedback", 1);
}

void DBServerMissionComm::updateMission(const string & status, const string & vehicleStatus)
{
    currentMission_->status = status;
    currentMission_->vehicleStatus = vehicleStatus;
    pub.publish(*currentMission_);
}

void DBServerMissionComm::updateStatus()
{
    switch( state_ )
    {
    case sWaitingMission:
        updateMission("Completed", "WaitingForAMission");
        delete currentMission_;
        currentMission_ = 0;
        break;

    case sGoingToPickup:
        updateMission("Processing", "GoingToPickupLocation");
        break;

    case sGoingToDropoff:
        updateMission("GoingToDropoffLocation", "Processing");
        break;

    case sAtPickup:
        updateMission("AtPickupLocation", "Processing");
        break;
    }
}

void DBServerMissionComm::waitForMission()
{
    client.waitForExistence();
    dbserver_comm::MissionRequest srv;
    srv.request.blocking = true;
    srv.request.checkStatus = false;
    while( currentMission_==0 )
        if( client.call(srv) )
            currentMission_ = new dbserver_comm::Mission(srv.response.mission);
    ROS_INFO("Got a new mission: from %s to %s", currentMission_->pickUpLocation.c_str(), currentMission_->dropOffLocation.c_str());
    pickup_ = stationList_(currentMission_->pickUpLocation);
    dropoff_ = stationList_(currentMission_->dropOffLocation);
}