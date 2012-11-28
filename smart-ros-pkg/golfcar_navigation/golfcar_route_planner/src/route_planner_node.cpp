#include <ros/ros.h>
#include <ros/console.h>

#include "RoutePlannerVehicle.h"
#include "SimpleGoal.h"
#include "SimulatedRoutePlanner.h"
#include "MissionComm.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_planner_node");
    ros::NodeHandle nh("~");

    StationPaths sp;
    RoutePlanner *rp = 0;
    MissionComm *comm = 0;
    PassengerComm *pc = 0;

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
        //rp = new RoutePlannerVehicle(sp);
        rp = new SimpleGoal(sp);
    }

    //pc = new DummyPassengerComm();
    pc = new SimplePassengerComm();


    bool use_dbserver = true;
    nh.getParam("use_dbserver", use_dbserver);
    if( use_dbserver )
    {
        std::string url;
//         std:string vehicleID="golfcart1";
        int vehicleID=1;
        
        if( ! nh.getParam("booking_url", url) )
        {
            ROS_FATAL("Parameter booking_url is missing");
            exit(1);
        }
        nh.getParam("vehicleID", vehicleID);
        ROS_INFO("Connecting to database at URL %s with ID %d.",
                 url.c_str(), vehicleID);
//         comm = new DBMissionComm(*rp, *pc, url, vehicleID);
        comm = new DBMissionComm(*rp, *pc, url, "1");
    }
    else
    {
        ROS_INFO("Directly prompting for missions in the terminal.");
        comm = new PromptMissionComm(*rp, *pc);
    }

    rp->startThread();
    comm->startThread();

    ros::spin();

    delete comm;
    delete rp;
    delete pc;

    return 0;
}
