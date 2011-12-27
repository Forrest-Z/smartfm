#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include "ros/ros.h"

#include "amcl_curb_window.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLCurb::AMCLCurb(map_t* map) : AMCLSensor()
{
  this->time_ = 0.0;
  this->map_ = map;
  return;
}

void 
AMCLCurb::SetCurbModelLikelihoodField(double z_hit,
                                   double z_rand,
                                   double sigma_hit,
                                   double max_occ_dist,
                                   double rand_range)
{
  this->z_hit_ = z_hit;
  this->z_rand_ = z_rand;
  this->sigma_hit_ = sigma_hit;
  this->rand_range_ = rand_range;
  
  map_update_cspace(this->map_, max_occ_dist);
}


////////////////////////////////////////////////////////////////////////////////
// Apply the curb sensor model
bool AMCLCurb::UpdateSensor(pf_t *pf, AMCLSensorData *data, bool *UseFlag)
{
  AMCLCurbData *ndata;

  ndata = (AMCLCurbData*) data;
  
  pf_update_sensor(pf, (pf_sensor_model_fn_t) LikelihoodFieldModel, data, UseFlag);
  return true;
}

double AMCLCurb::LikelihoodFieldModel(AMCLCurbData *data, pf_sample_set_t* set)
{
  AMCLCurb *self;
  unsigned int i, j;
  
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;
  
  //still have some questions here, maybe not need in this way;
  self = (AMCLCurb*) data->sensor;
  
  total_weight = 0.0;

  double meas_total_score=0.0;
  // Compute the sample weights
  
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;
	
    p = 1.0;

	//double sigmatemp = self->sigma_hit_;
	//double rangetemp = self->rand_range_;
	
    // Pre-compute a couple of things
    double z_hit_denom = 2 * self->sigma_hit_ * self->sigma_hit_;
    double z_rand_mult = 1.0/self->rand_range_;
        
    for (i = 0; i < data->curbSegment_.points.size(); i++)
    {
      pz = 0.0;
      
      //////////////////////////////////////////////////////////////////////////////////////
      //Treat the curb points as laser-beam end points, to make the most of original code;
      //////////////////////////////////////////////////////////////////////////////////////
      double xtemp = data->curbSegment_.points[i].x;
      double ytemp = data->curbSegment_.points[i].y;
      obs_range = sqrt(xtemp*xtemp+ytemp*ytemp);
      obs_bearing = atan2(ytemp,xtemp); 
      
      if (obs_range > self->rand_range_) continue;
      
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map_, hit.v[0]);
      mj = MAP_GYWY(self->map_, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!MAP_VALID(self->map_, mi, mj))
        z = self->map_->max_occ_dist;
      else
        z = self->map_->cells[MAP_INDEX(self->map_,mi,mj)].occ_dist;
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit_ * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand_ * z_rand_mult;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      
      ///////////////////////////////////////////////////////////////////////////////
      //this ad-hoc weighting scheme may be adjusted according to actual performance;
      ///////////////////////////////////////////////////////////////////////////////
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }
    meas_total_score = meas_total_score + p;
    
    sample->weight *= p;
    total_weight += sample->weight;
  }
  
  set->meas_score = meas_total_score/(set->sample_count)-1; 
  return(total_weight);
}
