/*
 * particle_filter.h
 *
 *  Created on: Jun 4, 2013
 *      Author: liuwlz
 *
 *
 * Modifed from Demain's code in Package /pose_tracking_pf
 */

#ifndef OBST_TRACKING_H_
#define OBST_TRACKING_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <fmutil/fm_math.h>
#include <tf/tf.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <numeric>
#include <tf/transform_listener.h>
#include <pnc_msgs/move_status.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
using namespace std;

struct VehicleParticle{
  double x;
  double y;
  double r;
  double v;
  double weight;
};

struct ParticlesStat{
  double var_x;
  double var_y;
  double mean_x;
  double mean_y;
};

class ObstPF{
	typedef boost::mt19937 ENG;    // Mersenne Twister
	typedef boost::normal_distribution<double> NORM_DIST;   // Normal Distribution
	typedef boost::variate_generator<ENG&,NORM_DIST> NORM_GEN;    // Variate generator
	ENG eng;

public:
	bool initialized_;
	// remove observed points from pointcloud when the point is
	// outside from this range. Can be used to initialize another filter
	// for multiple target tracking
	double dist_stddev_;
	//Distribution range of obst's velocoity and orientation
	double v_noise_, r_noise_;
	// 0-1 resampling ratio, percentage of particles to be retained during sampling
	double resampling_ratio_;
	double current_speed_;
	vector<VehicleParticle> particles_;
	double mean_x_, mean_y_, mean_r_, mean_v_;
	double var_x_, var_y_;
	double time_pre_;

	ObstPF(int particle_num, double dist_stddev, double resampling_ratio, sensor_msgs::PointCloud pts);
	~ObstPF();
	void initialize_particles(int particle_num, sensor_msgs::PointCloud pt);
	bool updateParticleWeight(sensor_msgs::PointCloud &pc, VehicleParticle &particle);
	void updateObservation(sensor_msgs::PointCloud &pc);
	void updateMotion();
	int roll_weighted_die(vector<double> &probabilities);
	ParticlesStat getParticlesStat(vector<VehicleParticle> &new_particles);
};

#endif /* OBST_TRACKING_H_ */
