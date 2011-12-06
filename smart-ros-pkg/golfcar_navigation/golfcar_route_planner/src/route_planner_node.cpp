#include "route_planner_vehicle.h"
#include "db_mission_comm.h"

//TODO: use parameters to control behavior

int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_planner_node");

    StationPaths sp;
    RoutePlanner *rp;
    DBMissionComm *comm;

    rp = new DummyRoutePlanner(sp);
    //rp = new RoutePlannerVehicle(sp);

    //comm = new PromptMissionComm(*rp);
    comm = new DBServerMissionComm(*rp);

    ros::spin();

    delete comm;
    delete rp;

    return 0;
}
