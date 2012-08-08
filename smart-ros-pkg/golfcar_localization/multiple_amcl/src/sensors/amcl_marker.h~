
#ifndef AMCL_MARKER_H
#define AMCL_MARKER_H

#include "amcl_sensor.h"
#include "../map/map.h"


//newly defined marker map, specially for vision_lane_marker;
typedef struct
{
  int marker_type;
  pf_vector_t maker_pose;

} marker_feature;

typedef struct
{
  // Map origin;
  //double origin_x, origin_y;
  marker_feature *markers;
  size_t marker_count;
} marker_map;

//-----------------------3 functions to be completed, follow previous format---------------------------
//4th "map initialization" should be completed in the main class like line-538 in "mix_amcl_node.cpp";
bool match_mapMarker(marker_map *map, marker_feature marker_measurement, marker_feature& marker_from_map);
marker_map *marker_map_alloc(void);
void marker_map_free(marker_map *map);


namespace amcl
{

// Laser Marker data
class AMCLMarkerData : public AMCLSensorData
{
  public:
    AMCLMarkerData () {};
    virtual ~AMCLMarkerData() {};
	
  public: pf_vector_t      Pos_Est;
  public: marker_feature   marker_meas_;
};


//Marker model
class AMCLMarker : public AMCLSensor
{
  // Default constructor
  public: AMCLMarker(marker_map* map);

  public: void SetMarkerModel(	double x_sigma,
                            	double y_sigma,
								double thetha_sigma,
                            	double dis_outlier);

  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data, bool *UseFlag, bool ValidSwitch, void *otherSensingSource);

  public: virtual void ValidateSensor(pf_t *pf, AMCLSensorData *data, bool *UseFlag, void *otherSensingSource);

  // Determine the probability for the given pose
  private: static double SimpleMarkerModel (AMCLMarkerData *data, 
                                            pf_sample_set_t* set);
  // Current data timestamp
  private: double time;

  // The laser map
  private: marker_map *map;

  // this parameter no need, since marker has been transferred int to "base_link" frame;
  // Camera offset relative to robot
  // private: pf_vector_t camera_pose;
  
  // Laser model params
  //
  // Mixture params for the components of the model; must sum to 1
  private: double x_sigma;
  private: double y_sigma;
  private: double thetha_sigma;
  private: double dis_outlier;
};


}

#endif
