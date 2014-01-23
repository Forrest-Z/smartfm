#ifndef MODT_PF_TRACKER_H
#define MODT_PF_TRACKER_H

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <fmutil/fm_math.h>
#include <tf/tf.h>
#include <numeric>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>

#include "OGM_model.h"

typedef boost::mt19937 ENG;    // Mersenne Twister
typedef boost::normal_distribution<double> NORM_DIST;   // Normal Distribution
typedef boost::variate_generator<ENG&,NORM_DIST> NORM_GEN;    // Variate generator

//for constant-speed-model;
struct motion_status
{
	double x, y, yaw, velocity, omega;
};

struct motion_shape_particle{
	double weight;
	motion_status motion;
	OGM_model* shape;
};

class PF_tracker
{
	OGM_model* best_shape_;
	std::vector<motion_shape_particle> particles_;

	double x_noise_sigma_;
	double y_noise_sigma_;
	double yaw_noise_sigma_;
	double velocity_noise_sigma_;
	double omega_noise_sigma_;

	void load_parameters();

	//core-functions for PF;
	void initialize();
	void predict();
	void update();
	void resample();
};

















#endif
