#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include "ros/ros.h"

#include "amcl_curb_window.h"

#define CURB_VALID_THRESH 1.5
#define RATE_VALID_THRESH 0.3

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
bool AMCLCurb::UpdateSensor(pf_t *pf, AMCLSensorData *data, bool *UseFlag, bool ValidSwitch)
{
  AMCLCurbData *ndata;
  ndata = (AMCLCurbData*) data;  
  
  if(ValidSwitch)
  {
      ValidateSensor(pf, data, UseFlag);
      if(*UseFlag == false){return false;}
  }
  
  pf_update_sensor(pf, (pf_sensor_model_fn_t) LikelihoodFieldModel, data, UseFlag);
  return true;
}

void AMCLCurb::ValidateSensor(pf_t *pf, AMCLSensorData *data, bool *UseFlag)
{
    AMCLCurb *self;
    unsigned int i;
    double z, pz;
    double p;
    double obs_range, obs_bearing;
    pf_vector_t pose;
    pf_vector_t hit;

    AMCLCurbData *ndata = (AMCLCurbData*) data;
    self = (AMCLCurb*) ndata->sensor;
    
    pose = pf->Pos_Est;
    
    ROS_DEBUG("pos_est: %3f, %3f, %3f", pose.v[0], pose.v[1], pose.v[2]);

    p = 1.0;
    
    ndata->validCurbSeg_.header = ndata->curbSegment_.header;
    ndata->validCurbSeg_.points.clear();
    
    ndata->invalidCurbSeg_.header = ndata->curbSegment_.header;
    ndata->invalidCurbSeg_.points.clear();
    
    for (i = 0; i < ndata->curbSegment_.points.size(); i++)
    {
      pz = 0.0;
      
      double xtemp = ndata->curbSegment_.points[i].x;
      double ytemp = ndata->curbSegment_.points[i].y;
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
        
      ROS_DEBUG("curb nearest dis %3f", z);
      //here "CURB_VALID_THRESH" can be changed into Mahalanobis distance;
      if(z < CURB_VALID_THRESH)
      {
          ndata->validCurbSeg_.points.push_back(ndata->curbSegment_.points[i]);
      }
      else
      {
          ndata->invalidCurbSeg_.points.push_back(ndata->curbSegment_.points[i]);
      }
    }
    
    unsigned int validNum = ndata->validCurbSeg_.points.size();
    unsigned int rawNum   = ndata->curbSegment_.points.size();
    ROS_DEBUG("valid, raw %d, %d", validNum, rawNum);
    bool valid_num_flag   = (float(validNum)) >= (RATE_VALID_THRESH*float(rawNum));
    
    if(valid_num_flag)
    {*UseFlag = true;
      ROS_DEBUG("CURB USE TRUE");
    }
    else
    {
      *UseFlag = false;
      ROS_DEBUG("CURB USE FALSE");
    }
}


double AMCLCurb::LikelihoodFieldModel(AMCLCurbData *data, pf_sample_set_t* set)
{
  AMCLCurb *self;
  unsigned int i;
  int j;
  
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
        
    for (i = 0; i < data->validCurbSeg_.points.size(); i++)
    {
      pz = 0.0;
      
      //////////////////////////////////////////////////////////////////////////////////////
      //Treat the curb points as laser-beam end points, to make the most of original code;
      //////////////////////////////////////////////////////////////////////////////////////
      double xtemp = data->validCurbSeg_.points[i].x;
      double ytemp = data->validCurbSeg_.points[i].y;
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
  
  //actualy here "meas_score" is not traditional "meas_score" for one single measurement, but sum of "meas_score"s for "pz*pz*pz";
  //the threshold value in "pf.c" should be learned from real data;
  //the structure should be improved later;
  set->meas_score = meas_total_score/(set->sample_count)-1; 
  return(total_weight);
}
