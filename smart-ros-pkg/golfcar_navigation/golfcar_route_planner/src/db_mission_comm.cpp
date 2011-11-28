#include <ros/ros.h>

#include "db_mission_comm.h"



void PromptMissionComm::run()
{
    stationList_.print();
    current_ = stationList_.prompt("Current station? ");
    ROS_INFO("Current station set to %s. READY!", current_.c_str());

    while( ros::ok() )
    {
        stationList_.print();
        pickup_ = stationList_.prompt("Pickup station? ");
        dropoff_ = stationList_.prompt("Drop off station? ");
    }
}





DBServerMissionComm::DBServerMissionComm(MissionStateMachine & sm)
    : DBMissionComm(sm)
{
    sub = n.subscribe("missions/assignments", 1, &DBServerMissionComm::missionCB, this);
    pub = n.advertise<dbserver_comm::Mission>("missions/feedback", 1);
}

void DBServerMissionComm::missionCB( const dbserver_comm::Mission & m )
{

}

dbserver_comm::Mission DBServerMissionComm::waitForNewMission()
{
    dbserver_comm::Mission m;
    while( ros::ok() /* && !currentMission_ */ )
        ros::Duration(1).sleep();
    return m;
}