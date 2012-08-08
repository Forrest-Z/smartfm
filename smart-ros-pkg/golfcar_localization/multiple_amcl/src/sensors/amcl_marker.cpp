
///////////////////////////////////////////////////////////////////////////
//
// Desc: 	AMCL lane_marker routines
// Author: 	NUS golf cart
// Date: 	20120801
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> 
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include "amcl_marker.h"

using namespace amcl;

//"marker_measurement" used here should already be in the global coordinate;
bool match_mapMarker(marker_map *map, marker_feature marker_measurement, marker_feature& marker_from_map)
{
	assert(map->marker_count > 0);
	double nearest_distance = 1000.0; 	//here only consider x and y for the distance;
	int marker_index = -1;
	
	pf_vector_t delt_pose;
	
	for(int i = 0; i < map->marker_count; i++)
	{
		delt_pose = pf_vector_sub(map->markers[i].maker_pose, marker_measurement.maker_pose);
		double distance_tmp = sqrt(delt_pose.v[0]*delt_pose.v[0]+delt_pose.v[1]*delt_pose.v[1]);
		if(map->markers[i].marker_type == marker_measurement.marker_type && distance_tmp < nearest_distance)
		{
			marker_index = i;
			nearest_distance = distance_tmp;
		}
	}
	
	if(marker_index != -1)
	{
		marker_from_map = map->markers[marker_index];
		return true;
	}
	else return false;
}

marker_map *marker_map_alloc(void)
{
	marker_map *map;
	map = (marker_map*) malloc(sizeof(marker_map));
	// Allocate storage for main map
	map->marker_count = 0;
	map->markers = (marker_feature*) NULL;
	return map;
}

void marker_map_free(marker_map *map)
{
	free(map->markers);
	free(map);
	return;
}

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLMarker::AMCLMarker(marker_map* map) : AMCLSensor()
{
  this->time = 0.0;
  this->map = map;
  return;
}

void 
AMCLMarker::SetMarkerModel(	double x_sigma,
							double y_sigma,
							double thetha_sigma,
                            double dis_outlier)
{
	this->x_sigma = x_sigma;
	this->y_sigma = y_sigma;
	this->thetha_sigma = thetha_sigma;
	this->dis_outlier = dis_outlier;					
}

////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLMarker::UpdateSensor(pf_t *pf, AMCLSensorData *data, bool *UseFlag, bool ValidSwitch, void *otherSensingSource)
{
  if(ValidSwitch)
  {
      ValidateSensor(pf, data, UseFlag, otherSensingSource);
      if(*UseFlag == false){return false;}
  }
  
  printf("marker validation---true---");
  pf_update_sensor(pf, (pf_sensor_model_fn_t) SimpleMarkerModel, data, UseFlag);
  return true;
}

void AMCLMarker::ValidateSensor(pf_t *pf, AMCLSensorData *data, bool *UseFlag, void *otherSensingSource)
{
	AMCLMarker *self;
    pf_vector_t pose, pose_in_map, pose_in_baselink, delt_pose;
    AMCLMarkerData *ndata = (AMCLMarkerData*) data;
    self = (AMCLMarker*) ndata->sensor;
    
    pose = pf->Pos_Est;
    pose_in_baselink = ndata->marker_meas_.maker_pose;
    pose_in_map = pf_vector_coord_add( pose_in_baselink, pose);
    
    //transfer the marker measurement into the global coordinate;
	marker_feature marker_measurement; 
	marker_measurement.marker_type = ndata->marker_meas_.marker_type;
	marker_measurement.maker_pose = pose_in_map;
	
    marker_feature marker_from_map;
	bool mapMarker_exist = match_mapMarker (self->map, marker_measurement, marker_from_map);
	if(!mapMarker_exist) {*UseFlag = false; return;}
	else
	{
		printf("marker_in_baselink:(%3f, %3f, %3f)\n", pose_in_baselink.v[0], pose_in_baselink.v[1], pose_in_baselink.v[2]);
		printf("baselink_pose:(%3f, %3f, %3f)\n", pose.v[0], pose.v[1], pose.v[2]);
		printf("marker_from_map:(%3f, %3f, %3f)\n", marker_from_map.maker_pose.v[0], marker_from_map.maker_pose.v[1], marker_from_map.maker_pose.v[2]);
		printf("marker_measurement:(%3f, %3f, %3f)\n", marker_measurement.maker_pose.v[0], marker_measurement.maker_pose.v[1], marker_measurement.maker_pose.v[2]);
		delt_pose = pf_vector_sub (marker_from_map.maker_pose, marker_measurement.maker_pose);
		//check if this nearest marker from map is near enough to the measurement;
		//simple criteria here;
		bool x_within = fabs(delt_pose.v[0]) < 2.0;
		bool y_within = fabs(delt_pose.v[1]) < 2.0;
		bool thetha_within = fabs(delt_pose.v[2]) < M_PI/180.0*30.0;
		if(x_within&&y_within&&thetha_within) *UseFlag = true;
		else *UseFlag = false;
	}
}

////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double AMCLMarker::SimpleMarkerModel (AMCLMarkerData *data, pf_sample_set_t* set)
{
	AMCLMarker *self;
	double p;
	double total_weight, meas_total_score;
	pf_sample_t *sample;
	pf_vector_t pose, pose_in_map, pose_in_baselink, delt_pose;
	
	total_weight = 0.0;
	meas_total_score=0.0;
	self = (AMCLMarker*) data->sensor;
	
	pose_in_baselink = data->marker_meas_.maker_pose;
	
	marker_feature marker_measurement; 
	marker_measurement.marker_type = data->marker_meas_.marker_type;
	
	double x_sigma_power = self->x_sigma * self->x_sigma;
	double y_sigma_power = self->y_sigma * self->y_sigma;
	double thetha_sigma_power = self->thetha_sigma * self->thetha_sigma;
	//iterate all the particles, and process each particle with 2 steps:
	//1st step: find the nearest corresponding marker in the map;
	//2nd step: update samples in the sample_set;
	int j;
	for (j = 0; j < set->sample_count; j++)
	{
		sample = set->samples + j;
		pose = sample->pose;
		
		pose_in_map = pf_vector_coord_add( pose_in_baselink, pose);
		marker_measurement.maker_pose = pose_in_map;
		
		marker_feature marker_from_map;
		
		bool mapMarker_exist = match_mapMarker (self->map, marker_measurement, marker_from_map);
		
		if(!mapMarker_exist)
		{
			//penalize erratic particle;
			p = 1.0e-6;
		}
		else
		{	
			delt_pose = pf_vector_sub (marker_from_map.maker_pose, marker_measurement.maker_pose);
			//update particle weight;
			p = 1.0;
			p *= 1.0/sqrt(2*M_PI*x_sigma_power)*exp(-(delt_pose.v[0] * delt_pose.v[0]) / (2 * x_sigma_power));
			p *= 1.0/sqrt(2*M_PI*y_sigma_power)*exp(-(delt_pose.v[1] * delt_pose.v[1]) / (2 * y_sigma_power));
			p *= 1.0/sqrt(2*M_PI*thetha_sigma_power)*exp(-(delt_pose.v[2] * delt_pose.v[2]) / (2 * thetha_sigma_power));
		}
		meas_total_score = meas_total_score+p;
		sample->weight *= p;
		total_weight += sample->weight;
     }
  
  set->meas_score = meas_total_score/(set->sample_count); 
  
  printf("--------set->meas_score-------- %3f", set->meas_score);

  return(total_weight);
}
