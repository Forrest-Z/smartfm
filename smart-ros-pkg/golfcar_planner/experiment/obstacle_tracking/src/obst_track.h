/*
 * obst_track.h
 *
 *  Created on: Jul 10, 2013
 *      Author: liuwlz
 */

#ifndef OBST_TRACK_H_
#define OBST_TRACK_H_

#include "particle_filter.h"
#include <ros/console.h>

class ObstTrack{
	ros::NodeHandle nh_;
	ros::Subscriber pc_sub_, move_status_sub_, speed_cmd_sub_;
	bool first_call_;
public:
	ObstTrack();
	~ObstTrack();
	double time_pre_;
	vector<ObstPF> filters_;
	int object_id_;
	int pre_size;
	ros::Publisher particles_pub_, mean_particles_pub_, pompdp_pub_, stopping_cmd_pub_;
	tf::TransformListener tf_;
	double distance_cur_;

	void publishParticles(vector<VehicleParticle> &particles);
	void filterPoints(geometry_msgs::Point32 init_pt, sensor_msgs::PointCloud &pc);
	void pcCallback(sensor_msgs::PointCloud::ConstPtr pc);
};

#endif /* OBST_TRACK_H_ */
