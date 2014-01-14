/***************************************************************************************************************************
 * constant speed model;
 * *************************************************************************************************************************
 * 	model-1:
 * 	x_(t+1) = x_t + v_t*cos(thetha_t)*delt_time;
 * 	y_(t+1) = y_t + v_t*sin(thetha_t)*delt_time;
 * 	thetha_(t+1) = thetha_t + omega_t*delt_time;
 * 	v_(t+1) = v_t;
 * 	omega_(t+1) = omega_t;
 *
 *  remarks:
 *  this one doesn't work because it involves thetha_t inside.
 * 	thetha in our implementation is not observed directly, and
 * 	BFL gives erratic estimation (1*e9) about thetha, and this
 * 	may be due to bad handleg of thetha.
 * 	Handling thetha as a gaussian distribution is tricky;
 * 	we need special attention to it in our implementation;
 * *************************************************************************************************************************
 * model-2:
 * x_(t+1) = x_t+ u_t*delt_time;
 * y_(t+1) = y_t+ v_t*delt_time;
 * u_(t+1) = speed*[cos(thetha+omega_t*delt_time)-cos(thetha)]; thetha = atan2(v_t, u_t), speed = sqrt(u_t*u_t+v_t*v_t);
 * v_(t+1) = speed*(sin(thetha+omega_t*delt_time)-sin(thetha));
 * omega_(t+1) = omega_t
 *
 * after simplification:
 * x_(t+1) = x_t+ v_t*delt_time;
 * y_(t+1) = y_t+ u_t*delt_time;
 * u_(t+1) = u_t*cos(omega*delt_time)-v_t*sin(omega*delt_time)
 * v_(t+1) = v_t*cos(omega*delt_time)+u_t*sin(omega*delt_time)
 * omega_(t+1) = omega_t
 * *************************************************************************************************************************
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
		if (i==j)
		{
			df(i,j) = 1;

		}
		else
		{
			df(i,j) = 0;
		}
      }
    }
  }


  NonLinearAnalyticConditionalGaussianOdo::~NonLinearAnalyticConditionalGaussianOdo(){}

  ColumnVector NonLinearAnalyticConditionalGaussianOdo::ExpectedValueGet() const
  {
    ColumnVector state = ConditionalArgumentGet(0);
    double delt_thetha = state(5)*delt_time;

    state(1) = state(1) + state(3)*delt_time;
    state(2) = state(2) + state(4)*delt_time;

    state(3) = state(3)*cos(delt_thetha)-state(4)*sin(delt_thetha);
    state(4) = state(4)*cos(delt_thetha)+state(3)*sin(delt_thetha);

    //state(3) = state(3);
    //state(4) = state(4);
    state(5) = state(5);
    return state + AdditiveNoiseMuGet();
  }

  Matrix NonLinearAnalyticConditionalGaussianOdo::dfGet(unsigned int i) const
  {
	ColumnVector state = ConditionalArgumentGet(0);
    if (i==0)//derivative to the first conditional argument (x)
	{
    	double delt_thetha = state(5)*delt_time;

		df(1,3)= delt_time;
		df(2,4)= delt_time;

		df(3,3)= cos(delt_thetha);
		df(3,4)= -sin(delt_thetha);
		df(3,5)= -state(3)*delt_time*sin(delt_thetha)-state(4)*delt_time*cos(delt_thetha);

		df(4,3)= sin(delt_thetha);
		df(4,4)= cos(delt_thetha);
		df(4,5)= -state(4)*delt_time*sin(delt_thetha)+state(3)*delt_time*cos(delt_thetha);

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

    double delt_thetha = state(5)*delt_time_para;
    state(1) = state(1) + state(3)*delt_time_para;
    state(2) = state(2) + state(4)*delt_time_para;
    state(3) = state(3)*cos(delt_thetha)-state(4)*sin(delt_thetha);
    state(4) = state(4)*cos(delt_thetha)+state(3)*sin(delt_thetha);
    state(5) = state(5);

    return state + AdditiveNoiseMuGet();
  }


}//namespace BFL

