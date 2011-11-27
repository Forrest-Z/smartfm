#ifndef __DB_SERVER_MISSION_COMM__H__
#define __DB_SERVER_MISSION_COMM__H__

#include "db_mission_comm.h"

class DBServerMissionComm : public MissionComm
{
public:

    DBServerMissionComm()
        : MissionComm()
    {
        sub = n.subscribe("missions/assignments", 1, &MissionStateMachine::missionCB, this);
        pub = n.advertize<dbserver_comm::Mission>("missions/feedback", 1);
    }

    void missionCB( const dbserver_comm::Mission & m )
    {

    }

    Mission waitForNewMission()
    {
        while( ros::ok() && !currentMission_ )
            ros::sleep(1);
    }


private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
};

#endif
