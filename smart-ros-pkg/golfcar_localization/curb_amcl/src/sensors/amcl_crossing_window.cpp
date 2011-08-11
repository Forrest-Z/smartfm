#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include "ros/ros.h"

#include "amcl_crossing_window.h"

#define MAX_DISTANCE 15.0

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLCrossing::AMCLCrossing(map_t* map) : AMCLSensor()
{
  this->time_ = 0.0;
  this->map_ = map;
  return;
}

void 
AMCLCrossing::SetCrossingModelBeam(	double z_hit,
									double z_short,
									double z_max,
									double z_rand,
									double sigma_hit,
									double lambda_short)
{
  this->z_hit = z_hit;
  this->z_short = z_short;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the curb sensor model
bool AMCLCrossing::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  AMCLCrossingData *ndata;

  ndata = (AMCLCrossingData*) data;
  
  pf_update_sensor(pf, (pf_sensor_model_fn_t) BeamModel, data);
  return true;
}

double AMCLCrossing::BeamModel(AMCLCrossingData *data, pf_sample_set_t* set)
{
  AMCLCrossing *self;
  int i, j;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  
  obs_range = MAX_DISTANCE;
  obs_bearing = data->tracyingAngle_;
 
  self = (AMCLCrossing*) data->sensor;
  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot

    p = 1.0;

    for (i = 0; i < data->FakeSensorPose_.size(); i ++)
    {
		pf_vector_t fakepose;
		fakepose = pf_vector_coord_add(data->FakeSensorPose_[i], pose);
		map_range = map_calc_range(self->map_, fakepose.v[0], fakepose.v[1], fakepose.v[2] + obs_bearing, MAX_DISTANCE);

		pz = 0.0;

		// Part 1: good, but noisy, hit
		z = obs_range - map_range;
		pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));

		// Part 2: short reading from unexpected obstacle (e.g., a person)
		if(z < 0)
        pz += self->z_short * self->lambda_short * exp(-self->lambda_short*obs_range);

		// Part 3: Failure to detect obstacle, reported as max-range
		if(obs_range == MAX_DISTANCE)
        pz += self->z_max * 1.0;

		// Part 4: Random measurements
		if(obs_range < MAX_DISTANCE)
        pz += self->z_rand * 1.0/MAX_DISTANCE;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }
    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}

