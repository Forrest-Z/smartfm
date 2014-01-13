/*************************************************
 * constant speed model;
 * ***********************************************
 * x_(t+1) = x_t + v_t*cos(thetha_t)*delt_time;
 * y_(t+1) = y_t + v_t*sin(thetha_t)*delt_time;
 * thetha_(t+1) = thetha_t + omega_t*delt_time;
 * v_(t+1) = v_t;
 * omega_(t+1) = omega_t;
 * ***********************************************
 */

#include "nonlinearanalyticconditionalgaussianodo.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng
                              // libraries
#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{
  using namespace MatrixWrapper;

  NonLinearAnalyticConditionalGaussianOdo::NonLinearAnalyticConditionalGaussianOdo(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_MOBILE),
      df(5,5)
  {
    // initialize df matrix
    for (unsigned int i=1; i<=5; i++){
      for (unsigned int j=1; j<=5; j++){
	if (i==j) df(i,j) = 1;
	else df(i,j) = 0;
      }
    }
  }


  NonLinearAnalyticConditionalGaussianOdo::~NonLinearAnalyticConditionalGaussianOdo(){}

  ColumnVector NonLinearAnalyticConditionalGaussianOdo::ExpectedValueGet() const
  {
    ColumnVector state = ConditionalArgumentGet(0);
    //ColumnVector vel  = ConditionalArgumentGet(1);
    state(1) += state(4)*cos(state(3))*delt_time;
    state(2) += state(4)*sin(state(3))*delt_time;
    state(3) += state(5)*delt_time;
    state(4) = state(4);
    state(5) = state(5);

    return state + AdditiveNoiseMuGet();
  }

  Matrix NonLinearAnalyticConditionalGaussianOdo::dfGet(unsigned int i) const
  {
	ColumnVector state = ConditionalArgumentGet(0);
    if (i==0)//derivative to the first conditional argument (x)
      {
    	df(1,3)= -state(4)*sin(state(3))*delt_time;
    	df(1,4)= cos(state(3))*delt_time;
    	df(2,3)= state(4)*cos(state(3))*delt_time;
    	df(2,4)= sin(state(3))*delt_time;
    	df(3,5)= delt_time;
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

  MatrixWrapper::ColumnVector NonLinearAnalyticConditionalGaussianOdo::getEstimate(double delt_time_para)
  {
    ColumnVector state = ConditionalArgumentGet(0);
    state(1) += state(4)*cos(state(3))*delt_time_para;
    state(2) += state(4)*sin(state(3))*delt_time_para;
    state(3) += state(5)*delt_time_para;
    state(4) = state(4);
    state(5) = state(5);
    return state + AdditiveNoiseMuGet();
  }


}//namespace BFL

