/*
 * simple_ped_client.cpp
 *
 *  Created on: Jan 16, 2012
 *      Author: golfcar
 */

#include "ros/ros.h"
#include "infrastructure_sensor/pedestrianDetected.h"
#include <cstdlib>

using namespace std;

class simple_ped
{
public:
    simple_ped(int ped_detected)
    {

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<infrastructure_sensor::pedestrianDetected>("pedestrian_crossing");
        ros::spinOnce();
        infrastructure_sensor::pedestrianDetected srv;
        srv.request.pedestrianDetected = ped_detected;
        ros::Time time_request_sent= ros::Time::now();

        while(time_request_sent.toSec()==0)
        {
            time_request_sent= ros::Time::now();
            ros::spinOnce();
        }

        srv.request.header.stamp = time_request_sent;
        if (client.call(srv))
        {
            if(srv.response.response == srv.request.pedestrianDetected)
            {
                ROS_INFO("Request successfully received");
                ROS_INFO("Delay: %lf s", (srv.response.header.stamp - time_request_sent).toSec());
            }
            else
                ROS_INFO("Request failed");
        }
        else
        {
            ROS_ERROR("Failed to call service pedestrian_crossing");
        }

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ped_client");
    if (argc != 2)
    {
        ROS_INFO("usage: ped_client 1/0");
        return 1;
    }
    simple_ped ped(atoll(argv[1]));
    return 0;
}
