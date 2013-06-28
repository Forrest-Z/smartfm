/*
 * particle_filter.h
 *
 *  Created on: Jun 4, 2013
 *      Author: liuwlz
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

struct ObstParticle{
	double x;
	double y;
	double r;
	double v;
};

class ObstPF{

};

#endif /* PARTICLE_FILTER_H_ */
