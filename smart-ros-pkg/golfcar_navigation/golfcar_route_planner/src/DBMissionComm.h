#ifndef __DB_MISSION_COMM__H__
#define __DB_MISSION_COMM__H__

#include <vector>
#include <string>

#include <StationPath.h>
#include <dbserver_comm/Mission.h>

#include <MissionComm.h>


/// Implements communication with the database via the Python ROS node.
class DBServerMissionComm : public MissionComm
{
public:
    DBServerMissionComm(RoutePlanner &);

private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::ServiceClient client;

    dbserver_comm::Mission *currentMission_;

    void updateStatus();
    void waitForMission();
    void updateMission(const std::string &, const std::string &);
};

#endif //__DB_MISSION_COMM__H__
