
#ifndef AMCL_CURB_WINDOW_H
#define AMCL_CURB_WINDOW_H

#include "amcl_sensor.h"
#include "../map/map.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>


namespace amcl
{

// AMCLCurbData 
class AMCLCurbData : public AMCLSensorData
{
  public:
    AMCLCurbData () {accumNum_ = 0; reinit_=true; };
    virtual ~AMCLCurbData(){};

	int 										accumNum_;
	bool                                        reinit_;
	sensor_msgs::PointCloud 					curbSegment_;
    pf_vector_t                                 Pose_Est_;
    sensor_msgs::PointCloud 					validCurbSeg_;
    sensor_msgs::PointCloud 					invalidCurbSeg_;

    tf::StampedTransform 						beginningTf_;
};


// AMCLCurb sensor model
class AMCLCurb : public AMCLSensor
{
  // Default constructor
  public: AMCLCurb(map_t* map);
  
  //AMCLCurb also use similar "likelihood model"; but its meaning change a little bit.
  //"z_rand" takes certain randomness into account.
  public: void SetCurbModelLikelihoodField(double z_hit,
                                       double z_rand,
                                       double sigma_hit,
                                       double max_occ_dist,
													double rand_range);
  
  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data, bool *UseFlag, bool ValidSwitch, void *otherSensingSource);

  public: virtual void ValidateSensor(pf_t *pf, AMCLSensorData *data, bool *UseFlag, void *otherSensingSource);

  // Determine the probability for the given pose
  private: static double LikelihoodFieldModel(AMCLCurbData *data, 
                                              pf_sample_set_t* set);

  // Current data timestamp
  private: double time_;

  // The laser map
  private: map_t *map_;

  // Laser model params
  //
  // Mixture params for the components of the model; must sum to 1
  private: double z_hit_;
  private: double z_rand_;
  //
  // Stddev of Gaussian model for laser hits.
  private: double sigma_hit_;

  // "rand_range_" is used to give kind of randomness
  // with value to be set together with "SetCurbModelLikelihoodField";
  private: double rand_range_;
};

};

#endif
