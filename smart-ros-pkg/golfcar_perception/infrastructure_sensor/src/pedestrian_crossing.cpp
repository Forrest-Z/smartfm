/*
 * pedestrian_crossing.cpp
 *
 *  Created on: Jan 16, 2012
 *      Author: golfcar
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <infrastructure_sensor/pedestrianDetected.h>

using namespace std;

class pedestrian_crossing
{
public:
    pedestrian_crossing();

private:
    bool ped_response(infrastructure_sensor::pedestrianDetected::Request  &req,
                      infrastructure_sensor::pedestrianDetected::Response &res );
    ros::ServiceServer service;
    ros::Publisher pub;
    int seq;
};

pedestrian_crossing::pedestrian_crossing()
{
    ros::NodeHandle n;

    service = n.advertiseService("pedestrian_crossing", &pedestrian_crossing::ped_response, this);
    pub = n.advertise<sensor_msgs::PointCloud>("pedestrian_crossing",1);
    ROS_INFO("Ready receive pedestrian_crossing data.");
    seq = 0;
    ros::spin();

}

bool pedestrian_crossing::ped_response(infrastructure_sensor::pedestrianDetected::Request  &req,
                  infrastructure_sensor::pedestrianDetected::Response &res )
{
    res.response = req.pedestrianDetected;
    ROS_INFO("sending back response: [%d]", res.response);
    sensor_msgs::PointCloud pc;
    pc.header.seq = seq++;
    pc.header.frame_id = "/infra_sensor";
    pc.header.stamp = ros::Time::now();
    geometry_msgs::Point32 p;
    if(res.response)
    {
        p.x = 0; p.y = 0; p.z = 0.5;
        pc.points.push_back(p);
        p.x = -0.5; p.y = -0.5; p.z = 0.5;
        pc.points.push_back(p);
    }
    else
    {
        p.x = 10; p.y = 0; p.z = 0.5;
        pc.points.push_back(p);
    }
    pub.publish(pc);
    res.header.seq = 1;
    res.header.stamp = ros::Time::now();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ped_crossing");
    pedestrian_crossing ped;
    return 0;
}
