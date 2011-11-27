#include "route_planner_vehicle.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_planner_node");

    StationPaths sp;
    RoutePlannerVehicle rpn(sp);
    MissionStateMachine msm(rpn);
    PromptMissionComm comm(msm);

    ros::spin();
    return 0;
}
