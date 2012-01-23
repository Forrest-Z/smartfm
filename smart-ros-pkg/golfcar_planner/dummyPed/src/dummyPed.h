
#ifndef DUMMY_PED_H_
#define DUMMY_PED_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class dummy_ped {
public:
	dummy_ped();
	
	void psgPedPoseCallback(const nav_msgs::OdometryConstPtr odo1, const nav_msgs::OdometryConstPtr odo2, const nav_msgs::OdometryConstPtr odo3);
	
	ros::Publisher psgPedpub_;

    
};


#endif /* DUMMY_PED_H_ */
