#include "route_planner_vehicle.h"
#include "db_mission_comm.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_planner_node");
    ros::NodeHandle nh("~");

    StationPaths sp;
    RoutePlanner *rp;
    DBMissionComm *comm;

    bool dummy_vehicle = false;
    nh.getParam("dummy_vehicle", dummy_vehicle);
    if( dummy_vehicle )
    {
        ROS_INFO("Using the dummy route planner (fake vehicle).");
        rp = new DummyRoutePlanner(sp);
    }
    else
    {
        ROS_INFO("Using the real vehicle route planner.");
        rp = new RoutePlannerVehicle(sp);
    }

    bool use_dbserver = true;
    nh.getParam("use_dbserver", use_dbserver);
    if( use_dbserver )
    {
        ROS_INFO("Retrieving mission requests from the DBServer (via topic missions/assignments).");
        comm = new DBServerMissionComm(*rp);
    }
    else
    {
        ROS_INFO("Directly prompting for missions in the terminal.");
        comm = new PromptMissionComm(*rp);
    }

    ros::spin();

    delete comm;
    delete rp;

    return 0;
}
