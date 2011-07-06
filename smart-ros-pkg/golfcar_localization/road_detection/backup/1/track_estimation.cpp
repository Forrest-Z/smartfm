#include <track_estimation.h>

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;
using namespace ros;

//the bigger, the more noise; in other words, less confident in prediction;
#define PRIOR_COV_X pow(0.05,2)
#define PRIOR_COV_Y pow(0.05,2)
#define PRIOR_COV_THETA pow(5*M_PI/180,2)

#define SIGMA_SYSTEM_NOISE_X pow(0.1,2)
#define SIGMA_SYSTEM_NOISE_Y pow(0.1,2)
#define SIGMA_SYSTEM_NOISE_THETA pow(10*M_PI/180,2)

#define SIGMA_MEAS_NOISE_X pow(0.3,2)
#define SIGMA_MEAS_NOISE_Y pow(0.3,2)
#define SIGMA_MEAS_NOISE_THETA pow(10*M_PI/180,2)

namespace estimation
{
	
  TrackEstimation::TrackEstimation():
  vehicles_flag_(false),
  filter_initialized_(false),
  old_forward_dis_(5.40)
  {
	// create SYSTEM MODEL
    ColumnVector sysNoise_Mu(3);  sysNoise_Mu = 0;
    SymmetricMatrix sysNoise_Cov(3); sysNoise_Cov = 0; 
	sysNoise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X;
	sysNoise_Cov(1,2) = 0;
	sysNoise_Cov(1,3) = 0;
	sysNoise_Cov(2,1) = 0;
	sysNoise_Cov(2,2) = SIGMA_SYSTEM_NOISE_Y;
	sysNoise_Cov(2,3) = 0;
	sysNoise_Cov(3,1) = 0;
	sysNoise_Cov(3,2) = 0;
	sysNoise_Cov(3,3) = SIGMA_SYSTEM_NOISE_THETA;
	
    Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    sys_pdf_   = new NonLinearAnalyticConditionalGaussianTrack(system_Uncertainty);
    sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_ );
    
    // create MEASUREMENT MODEL Curb
    ColumnVector measNoiseCurb_Mu(3);  measNoiseCurb_Mu = 0;
    SymmetricMatrix measNoiseCurb_Cov(3);  measNoiseCurb_Cov = 0;
    
    measNoiseCurb_Cov(1,1) = SIGMA_MEAS_NOISE_X;
    measNoiseCurb_Cov(1,2) = 0;
    measNoiseCurb_Cov(1,3) = 0;
    measNoiseCurb_Cov(2,1) = 0;
    measNoiseCurb_Cov(2,2) = SIGMA_MEAS_NOISE_Y;
    measNoiseCurb_Cov(2,3) = 0;
    measNoiseCurb_Cov(3,1) = 0;
    measNoiseCurb_Cov(3,2) = 0;
    measNoiseCurb_Cov(3,3) = SIGMA_MEAS_NOISE_THETA;
    
    Gaussian measurement_Uncertainty_Curb(measNoiseCurb_Mu, measNoiseCurb_Cov);
    Matrix HCurb(3,3);  HCurb = 0;
    HCurb(1,1) = 1;
    
    HCurb(2,2) = 1;    
    HCurb(3,3) = 1;
    
    meas_pdf_   = new LinearAnalyticConditionalGaussian(HCurb, measurement_Uncertainty_Curb);
    meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf_);
  }
    
  TrackEstimation::~TrackEstimation(){
    if (filter_) delete filter_;
    if (prior_)  delete prior_;
    delete meas_model_;
    delete meas_pdf_;
    delete sys_pdf_;
    delete sys_model_;
  }
  
  void TrackEstimation::initialize(const road_detection::vec_point& prior)
  {	  
	  ROS_INFO("Initialize the Filter;");
	  ColumnVector prior_Mu(3);
	  SymmetricMatrix prior_Cov(3);
	  
	  prior_Cov(1,1) = PRIOR_COV_X;
	  prior_Cov(1,2) = 0;
	  prior_Cov(1,3) = 0;
	  
	  prior_Cov(2,1) = 0;
	  prior_Cov(2,2) = PRIOR_COV_Y;
	  prior_Cov(2,3) = 0;
	  
	  prior_Cov(3,1) = 0;
	  prior_Cov(3,2) = 0;
	  prior_Cov(3,3) = PRIOR_COV_THETA;
	  
	  prior_Mu(1) = prior.x;
	  prior_Mu(2) = prior.y;
	  prior_Mu(3) = prior.thetha;
	  
	  prior_  = new Gaussian(prior_Mu,prior_Cov);
	  filter_ = new ExtendedKalmanFilter(prior_);
	  
	  ROS_INFO("---------Initialization finished-------");
      ROS_INFO("prior vector: (x, y, thetha) %3f, %3f, %3f", prior_Mu(1),prior_Mu(2),prior_Mu(3));
      ROS_INFO("prior covariance: (sigma1, sigma2, sigma3) %3f, %3f, %3f", prior_Cov(1,1),prior_Cov(2,2),prior_Cov(3,3));
	  
	  filter_initialized_ = true;
	  
	  old_observation_ = prior;
	  old_forward_dis_ = prior.y;
  }
  
  bool TrackEstimation::update(const road_detection::vec_point& curb_observation, float& association_gate, bool& meas_update, bool& reinitial, const float& delt_phi, const float& tx, const float& ty)
  {
	  // to guarantee that the filter has been initialized before being updated.
	  if (!filter_initialized_)
	  {
      ROS_INFO("Cannot update filter when filter was not initialized first.");
      return false;
      }
      
      sys_pdf_->SetParameterOnline(delt_phi);
      
      //--------------------1----------prediction-----------step-------------------------
      // calculate velocity from given values;
      ColumnVector vel_desi(3);
      
      float XA = tx-sinf(delt_phi)*old_forward_dis_;
      float YA = ty+cosf(delt_phi)*old_forward_dis_;
      
      float XC = old_observation_.x;
      float YC = old_observation_.y;
      float phiC = old_observation_.thetha;
      
      float XB, YB;
            
      if(phiC==M_PI_2)
      {
		  XB=XC;
		  YB=tanf(delt_phi)*(XB-XA)+YA;
	  }
	  else
	  {
		  if(tanf(delt_phi)==tanf(phiC))
		  {
		  ROS_INFO("delt_phi==phiC, B will be ultimately far away, the filter need to be reinitialized.");
		  reinitial = true;
          return false;
          }
          else
          {
			  float nominator=(YC-YA-tanf(phiC)*XC+tanf(delt_phi)*XA);
			  float denominator=tanf(delt_phi)-tanf(phiC);
			  XB=nominator/denominator;
			  YB=tanf(delt_phi)*(XB-XA)+YA;
		  }
	  }
	  
      vel_desi(1)=cosf(delt_phi)*(XB-XC)+sinf(delt_phi)*(YB-YC)-cosf(delt_phi)*tx-sinf(delt_phi)*ty;
      vel_desi(2)=-sinf(delt_phi)*(XB-XC)+cosf(delt_phi)*(YB-YC)+sinf(delt_phi)*tx-cosf(delt_phi)*ty;
      vel_desi(3)=delt_phi;
      
      filter_->Update(sys_model_, vel_desi);
      
      //for debugging purposes;
      MatrixWrapper::ColumnVector estimate_vector = filter_->PostGet()->ExpectedValueGet();
      MatrixWrapper::SymmetricMatrix estimate_cov = filter_->PostGet()->CovarianceGet();
      
      ROS_INFO("---------1. prediction finished-------");
      ROS_INFO("Estimate vector: (x, y, thetha) %f, %f, %f", estimate_vector(1),estimate_vector(2),estimate_vector(3));
	  ROS_INFO("Estimate vector: (sigma1, sigma2, sigma3) %f, %f, %f", estimate_cov(1,1), estimate_cov(2,2), estimate_cov(3,3));
      
      //------------------2--------data association: analyze data and make judgement----------------------------
      ColumnVector meas_vec(3);
	  meas_vec(1) = curb_observation.x;
      meas_vec(2) = curb_observation.y;
      meas_vec(3) = curb_observation.thetha;
      
      ROS_INFO("---------2. data association: analyze data and make judgement-------");
      ROS_INFO("observation vector: (x, y, thetha) %3f, %3f, %3f", meas_vec(1),meas_vec(2),meas_vec(3));
      
      
      //judgement made from the difference between "prediction" and "observation";
      float dis_x = estimate_vector(1)-meas_vec(1);
      float dis_y = estimate_vector(2)-meas_vec(2);
      float dis_pred_meas = sqrtf(dis_x*dis_x+dis_y*dis_y);
      ROS_INFO("dis_x %f, dis_y %f, dis_pred_meas %f", dis_x, dis_y, dis_pred_meas);
      
      //a.for normal noise
      if(dis_x<-association_gate||dis_x>association_gate){meas_update = false;}
      
      //b.curb segmentation;
      if((dis_x<-3||dis_x>3)&& (meas_vec(1)>=4||meas_vec(1)<-4)) 
      {
	   ROS_INFO("-----------------Curb Segmentation-----------Curb Segmentation---------");
	   meas_update = true; 	 
	   reinitial = true;return false;
	  }
	  
	  //c.vehicles or pedestrians; only for the right filter, because of the "left-driving" mode;
      if(meas_vec(1)<3 && dis_x>1)
      {
	   ROS_INFO("---------------Vehicles or Pedestrians, set meas_update false------------");
	   vehicles_flag_ = true;
	   meas_update = false;
	  }

      
      //------------------3-------measurement---------update--------------------------
      
      if(meas_update) 
      {
		  ROS_INFO("measurement update");
		  filter_->Update(meas_model_, meas_vec);
	  }
	  else {ROS_INFO("meas_update false, no update");}
      
      MatrixWrapper::ColumnVector estimate_vector_meas = filter_->PostGet()->ExpectedValueGet();
      MatrixWrapper::SymmetricMatrix estimate_cov_meas = filter_->PostGet()->CovarianceGet();
      
      ROS_INFO("---------measurement update finished, if have-------");
      ROS_INFO("Estimate vector: (x, y, thetha) %3f, %3f, %3f", estimate_vector(1),estimate_vector(2),estimate_vector(3));
      ROS_INFO("Estimate vector: (sigma1, sigma2, sigma3) %f, %f, %f", estimate_cov(1,1),estimate_cov(2,2),estimate_cov(3,3));

      //------------------4-------preparing--for--next--round----------
      estimate_value_.x = estimate_vector_meas(1);
      estimate_value_.y = estimate_vector_meas(2);
      estimate_value_.thetha = estimate_vector_meas(3);
      estimate_covariance_ = estimate_cov_meas;
      
      if(meas_update)
      {
		old_observation_ = curb_observation;
		old_forward_dis_ = curb_observation.y;
		vehicles_flag_ = false;
		return true;
	  }
	  else 
	  {
		old_observation_ = estimate_value_;
		old_forward_dis_ = estimate_value_.y;
	  }
	  return true;
  }
  
  void TrackEstimation::getEstimate(road_detection::vec_point& estimate_value, MatrixWrapper::SymmetricMatrix& estimate_covariance)
  {
	  estimate_value = estimate_value_;
	  estimate_covariance = estimate_covariance_;
  }

  
}
