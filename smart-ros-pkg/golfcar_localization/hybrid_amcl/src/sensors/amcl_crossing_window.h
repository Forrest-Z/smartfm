
#ifndef AMCL_CROSSING_WINDOW_H
#define AMCL_CROSSING_WINDOW_H

#include "amcl_sensor.h"
#include "../map/map.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <string>

namespace amcl
{

// AMCLCurbData 
class AMCLCrossingData : public AMCLSensorData
{

  public:

	//"tracyingAngle_" differs according to "left" and "right";
	double 											tracyingAngle_;
    pf_vector_t                                     Pose_Est_;
	// "crossing" uses "reinit_" in "curb", because curb will be used to update at the same time;
	//bool                                   reinit_;

	// CrossingPtNum_ = FakeSensorPose_.size();
	//int 											CrossingPtNum_;
	std::vector <pf_vector_t> 					FakeSensorPose_;

	//no need, because all of them have same maximum values;
	//std::vector <float> 						  CrossingRanges_;

   tf::StampedTransform 						  beginningTf_;

	AMCLCrossingData (double tracyingAngle) { this->tracyingAngle_ = tracyingAngle;};
   virtual ~AMCLCrossingData(){};

};


// AMCLCurb sensor model
class AMCLCrossing : public AMCLSensor
{
  // Default constructor
  public: AMCLCrossing(map_t* map);
  
  //AMCLCurb also use similar "likelihood model"; but its meaning change a little bit.
  //"z_rand" takes certain randomness into account.
  public: void SetCrossingModelBeam(double z_hit,
                      			   	double z_short,
                      	  				double z_max,
                        				double z_rand,
                       				  	double sigma_hit,
                        				double lambda_short);

  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data, bool *UseFlag);

  // Determine the probability for the given pose
  private: static double BeamModel(AMCLCrossingData *data, pf_sample_set_t* set);

  // Current data timestamp
  private: double time_;

  // The laser map
  private: map_t *map_;

  // Laser model params
  // Mixture params for the components of the model; must sum to 1
  private: double z_hit;
  private: double z_short;
  private: double z_max;
  private: double z_rand;
  //
  // Stddev of Gaussian model for laser hits.
  private: double sigma_hit;
  // Decay rate of exponential model for short readings.
  private: double lambda_short;
};

};

#endif
