#include <ros/ros.h>
#include <dbserver_comm/Mission.h>
#include <dbserver_comm/MissionRequest.h>
#include "db_mission_comm.h"

using std::string;

DBServerMissionComm::DBServerMissionComm(RoutePlanner & rp)
    : MissionComm(rp), currentMission_(0)
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
