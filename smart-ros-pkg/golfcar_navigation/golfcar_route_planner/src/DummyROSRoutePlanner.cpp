#include "DummyROSRoutePlanner.h"

DummyROSRoutePlanner::DummyROSRoutePlanner(StationPaths & sp) : ROSRoutePlanner(sp)
{
    vehicle_speed = 0.0;
    ros::NodeHandle nh("~");
    nh.getParam("vehicle_speed", vehicle_speed);
    ROS_INFO("Vehicle's speed is %lf", vehicle_speed);
}

bool DummyROSRoutePlanner::goToDest()
{
    if( vehicle_speed==0 )
        return true;

    if( last_time==0 ) {
        last_time = ros::Time::now().toSec();
        return false;
    }

    double current_time = ros::Time::now().toSec();
    distance_travelled += vehicle_speed * (current_time - last_time);
    last_time = current_time;

    ROS_INFO_THROTTLE(1, "Going to %s: travelled %dm out of %dm (%d\%). ETA=%d.",
                      destination_.c_str(), (int)distance_travelled,
                      (int)distance_to_travel,
                      (int)(distance_travelled/distance_to_travel*100),
                      (int)((distance_to_travel-distance_travelled)/vehicle_speed));

    if (distance_travelled > distance_to_travel) {
        ROS_INFO("Reached %s.", destination_.c_str());
        return true;
    }
    return false;
}

void DummyROSRoutePlanner::initDest()
{
    distance_travelled = 0;
    distance_to_travel = sp_.getPath(currentStation_, destination_).length();
    last_time = 0;
    ROS_INFO("Moving from %s to %s", currentStation_.c_str(), destination_.c_str());
}
