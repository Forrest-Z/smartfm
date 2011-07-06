//NUS Golf Cart
//curb_track

#include <ros/ros.h>
#include <nonlinearanalyticconditionalgaussiantrack.h>
#include <cmath>
#include <wrappers/rng/rng.h> // Wrapper around several rng
                              // libraries
#define NUMCONDARGUMENTS_TRACK 2

namespace BFL
{
  using namespace MatrixWrapper;


  NonLinearAnalyticConditionalGaussianTrack::NonLinearAnalyticConditionalGaussianTrack(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_TRACK)
  {
	//these two values should be changed according to input, using member function;
	delt_phi_=0.0;
  }


  NonLinearAnalyticConditionalGaussianTrack::~NonLinearAnalyticConditionalGaussianTrack(){}

  ColumnVector NonLinearAnalyticConditionalGaussianTrack::ExpectedValueGet() const
  {
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel  = ConditionalArgumentGet(1);
    state(1) = cos(delt_phi_) * state(1)+sin(delt_phi_)*state(2)+vel(1);
    state(2) = -sin(delt_phi_) * state(1)+cos(delt_phi_)*state(2)+vel(2);
    state(3) = state(3)+vel(3);
    return state + AdditiveNoiseMuGet();
  }

  Matrix NonLinearAnalyticConditionalGaussianTrack::dfGet(unsigned int i) const
  {
	if (i==0)//derivative to the first conditional argument (x)
    {
	Matrix df(3,3);
	
	df = 0;

	df(1,1)=cos(delt_phi_);
	df(1,2)=sin(delt_phi_);
	df(1,3)=0;
	df(2,1)=-sin(delt_phi_);
	df(2,2)=cos(delt_phi_);
	df(2,3)=0;
	df(3,1)=0;
	df(3,2)=0;
	df(3,3)=1;

	return df;
    }
    else
      {
	if (i >= NumConditionalArgumentsGet())
	  {
	    cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
	    exit(-BFL_ERRMISUSE);
	  }
	else{
	  cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
	  exit(-BFL_ERRMISUSE);
	}
      }
  }
  
  void NonLinearAnalyticConditionalGaussianTrack::SetParameterOnline(const float& delt_phi)
  {
	  delt_phi_ = delt_phi; 
	  //for debugging purpose
	  ROS_INFO("-----1.Debugging------ set delt_phi: %5f", delt_phi_);
  }

}//namespace BFL

