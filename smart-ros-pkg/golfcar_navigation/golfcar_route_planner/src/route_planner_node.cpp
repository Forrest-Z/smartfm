#include <ros/ros.h>

#include "RoutePlannerVehicle.h"
#include "SimulatedRoutePlanner.h"
#include "MissionComm.h"

using std::string;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_planner_node");
    ros::NodeHandle nh("~");

    StationPaths sp;
    RoutePlanner *rp;
    MissionComm *comm;

    bool dummy_vehicle = false;
    nh.getParam("dummy_vehicle", dummy_vehicle);
    if( dummy_vehicle )
    {
        double speed = 0;
        nh.getParam("vehicle_speed", speed);
        ROS_INFO("Using the simulated route planner (fake vehicle).");
        rp = new SimulatedRoutePlanner(sp,speed);
        ((SimulatedRoutePlanner *)rp)->verbose = true;
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
        string url = "http://fmautonomy.no-ip.info/dbserver", vehicleID="golfcart1";
        nh.getParam("/dbserver/url", url);
        nh.getParam("vehicleID", vehicleID);
        ROS_INFO("Connecting to database at URL %s with ID %s.",
                 url.c_str(), vehicleID.c_str());
        comm = new DBMissionComm(*rp, url, vehicleID);
    }
    else
    {
        ROS_INFO("Directly prompting for missions in the terminal.");
        comm = new PromptMissionComm(*rp);
    }

    rp->startThread();
    comm->startThread();

    ros::spin();

    delete comm;
    delete rp;

    return 0;
}
