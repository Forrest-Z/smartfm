//NUS Golf Cart
//curb_track

#ifndef __NON_LINEAR_ANALYTIC_CONDITIONAL_GAUSSIAN_TRACK__
#define __NON_LINEAR_ANALYTIC_CONDITIONAL_GAUSSIAN_TRACK__

#include <pdf/analyticconditionalgaussian_additivenoise.h>

namespace BFL
{
  /// Non Linear Conditional Gaussian
  /**
     - \f$ \mu = Matrix[1] . ConditionalArguments[0] +
     Matrix[2]. ConditionalArguments[1]  + ... + Noise.\mu \f$
     - Covariance is independent of the ConditionalArguments, and is
     the covariance of the Noise pdf
  */
  class NonLinearAnalyticConditionalGaussianTrack : public AnalyticConditionalGaussianAdditiveNoise
  {
    public:
      /// Constructor
      /** @pre:  Every Matrix should have the same amount of rows!
	  This is currently not checked.  The same goes for the number
	  of columns, which should be equal to the number of rows of
	  the corresponding conditional argument!
	  @param ratio: vector containing the different matrices of
	  the linear relationship between the conditional arguments
	  and \f$\mu\f$
	  @param additiveNoise Pdf representing the additive Gaussian uncertainty
      */
      NonLinearAnalyticConditionalGaussianTrack( const Gaussian& additiveNoise);
		virtual void SetParameterOnline(const float& delt_phi);

      /// Destructor
      virtual ~NonLinearAnalyticConditionalGaussianTrack();

      // redefine virtual functions
      virtual MatrixWrapper::ColumnVector    ExpectedValueGet() const;
      virtual MatrixWrapper::Matrix          dfGet(unsigned int i)       const;

    private:
		mutable float delt_phi_;
    };
} // End namespace BFL
 
#endif //  
