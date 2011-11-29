#include "route_planner_vehicle.h"
#include "db_mission_comm.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_planner_node");

    StationPaths sp;

    DummyRoutePlanner rp(sp);
    //RoutePlannerVehicle rp(sp);

    //PromptMissionComm comm(rp);
    DBServerMissionComm comm(rp);

    ros::spin();
    return 0;
}
