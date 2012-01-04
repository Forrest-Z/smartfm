/* NUS Golf Cart
 * "curb_intersection_node" is based on the original Amcl node;
 */
  
#include <algorithm>
#include <vector>
#include <map>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "map/map.h"
#include "pf/pf.h"
#include "sensors/amcl_odom.h"

//change to "amcl_curb_window.h";
//remember to change the cmake file;
#include "sensors/amcl_curb_window.h"
#include "sensors/amcl_crossing_window.h"

#include "sensors/amcl_laser.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/GetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

using namespace amcl;

#define RANDOM_SAMPLE_BOX_X 7.0
#define RANDOM_SAMPLE_BOX_Y 7.0
#define RANDOM_SAMPLE_BOX_THETHA M_PI/3.0

#define MAX_DISTANCE 10.0
#define CROSSING_TRESH 10

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}

static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

class MixAmclNode
{
  public:
    MixAmclNode();
    ~MixAmclNode();

    // @_@ this function is not used;
    int process();

  private:
    tf::TransformBroadcaster* tfb_;
    tf::TransformListener* tf_;

    bool sent_first_transform_;

    tf::Transform latest_tf_;
    bool latest_tf_valid_;

    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);
    // Message callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,std_srvs::Empty::Response& res);        
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void initialPoseReceivedOld(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    double getYaw(tf::Pose& t);
    // the two most fundamental functions for mix_amcl;
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);                                
    void curbReceived (const sensor_msgs::PointCloud::ConstPtr& cloud_in);

    //parameter for what odom to use
    std::string odom_frame_id_;
    //parameter for what base to use
    std::string base_frame_id_;
    // one global frame, with two maps overlaped;
    std::string global_frame_id_;
	
	//this parameter is not used;
    ros::Duration gui_publish_period;
    
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    map_t* curb_map_;
    map_t* planar_map_;
    
    // @_@ the following parameters are not used;
    char* mapdata;
    int sx, sy;
    double resolution;
    
    //parameters related to "odom";
    //move it here as class member; 
    //"alpha6_" takes into accound "pitch noise";
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_, alpha6_;
    
    ////////////////////////////////////////////////////////////////////////////
    //the incoming information data is actually "two beginning points for curb";
    ////////////////////////////////////////////////////////////////////////////
    message_filters::Subscriber<sensor_msgs::PointCloud> *curb_sidepoints_sub_;
	tf::MessageFilter<sensor_msgs::PointCloud> * curb_filter_;
	message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
		
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_sub_;
    tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_filter_;
    
    std::vector< AMCLLaser* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;

    // Particle filter
    pf_t *pf_;
    boost::mutex pf_mutex_;
    double pf_err_, pf_z_;
    
    bool pf_init_;
    bool curb_init_;
    bool laser_init_;
    
    bool LaserUseFlag_;	
    bool CurbUseFlag_; 
    bool CrossingUseFlag_; 
    bool LaserOn_;
    
    pf_vector_t pf_odom_pose_;
    pf_vector_t pf_curb_odom_pose_;
    pf_vector_t pf_laser_odom_pose_;
    
    double curb_d_thresh_, curb_a_thresh_;
    int numTresh_;
    double laser_d_thresh_, laser_a_thresh_;
        
    int resample_interval_;
    int resample_count_;
    
    double laser_min_range_;
    double laser_max_range_;

    AMCLOdom* odom_;
    AMCLCurb* curb_;
    AMCLCurbData* curbdata_; 
    
    AMCLLaser* laser_;
    
    AMCLCrossing* crossing_;
    AMCLCrossingData* LeftCroData_;
	AMCLCrossingData* RightCroData_;
	
	//@_@ the following two parameters are not used;
    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    map_t* requestMap(const std::string& map_service);

    // Helper to get odometric pose from transform system;
    // add "pitch" for original function;
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,double& x, double& y, double& yaw, double& pitch,
                     const ros::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_pub_;
    ros::Publisher particlecloud_pub_; 
    ros::ServiceServer global_loc_srv_;
    ros::Subscriber initial_pose_sub_old_;
    
    /////////////////////////////////////////////////////////////////
    //for debugging purposes;
    /////////////////////////////////////////////////////////////////
    ros::Publisher curbbaselink_;

    /////////////////////////////////////////////////////////////////
    //for analysis purposes;
    //analyze left and right individually to get more information;
    ////////////////////////////////////////////////////////////////
    sensor_msgs::PointCloud 					oldLeftMeas_;
    sensor_msgs::PointCloud 					newLeftMeas_;
    geometry_msgs::PointStamped 				LeftAnalysis_;
    ros::Publisher 								LeftAnalysis_pub_;
     
    sensor_msgs::PointCloud 					oldRightMeas_;
    sensor_msgs::PointCloud 					newRightMeas_;
    geometry_msgs::PointStamped 				RightAnalysis_;
    ros::Publisher 								RightAnalysis_pub_;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //a crude function to analyze characteristics of particles, already not in use
	//void measAnalysis(sensor_msgs::PointCloud& oldMeas, sensor_msgs::PointCloud& newMeas, const double Disnormalizer, geometry_msgs::PointStamped& AnalysisResult, const ros::Publisher& Analysis_pub);
     
    //add on 20111222, to publish invalid curb data;
    ros::Publisher invalid_curb_pub_;
};

#define USAGE "USAGE: amcl"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "mix_amcl");
  ros::NodeHandle nh;

  MixAmclNode an;

  ros::spin();

  return(0);
}

MixAmclNode::MixAmclNode() :
        sent_first_transform_(false),
        latest_tf_valid_(false),
        curb_map_(NULL),
        planar_map_(NULL),
        pf_(NULL),
        resample_count_(0),
	private_nh_("~")
{
  // Grab params off the param server
  int max_beams, min_particles, max_particles;
  //double alpha1, alpha2, alpha3, alpha4, alpha5;
  double alpha_slow, alpha_fast;
  
  //two sets for any parameter related to "sensor" or "measurement";
  double curb_z_hit, curb_z_rand, curb_sigma_hit, curb_rand_range;
  double laser_z_hit, laser_z_short, laser_z_max, laser_z_rand, laser_sigma_hit, laser_lambda_short;
  double curb_likelihood_max_dist, laser_likelihood_max_dist;
  double pf_err, pf_z;

  //not used;
  double tmp;
  private_nh_.param("gui_publish_rate", tmp, -1.0);
  gui_publish_period = ros::Duration(1.0/tmp);
  private_nh_.param("save_pose_rate", tmp, 1.0);
  save_pose_period = ros::Duration(1.0/tmp);
  private_nh_.param("laser_min_range", laser_min_range_, -1.0);
  private_nh_.param("laser_max_range", laser_max_range_, -1.0);
  private_nh_.param("min_particles", min_particles, 1000);
  private_nh_.param("max_particles", max_particles, 5000);
  private_nh_.param("laser_max_beams", max_beams, 30);
  private_nh_.param("kld_err", pf_err, 0.01);
  private_nh_.param("kld_z", pf_z, 0.99);
  private_nh_.param("odom_alpha1", alpha1_, 0.2);
  private_nh_.param("odom_alpha2", alpha2_, 0.2);
  private_nh_.param("odom_alpha3", alpha3_, 0.2);
  private_nh_.param("odom_alpha4", alpha4_, 0.2);
  private_nh_.param("odom_alpha5", alpha5_, 0.2);
  private_nh_.param("odom_alpha6", alpha6_, 0.09);

  private_nh_.param("curb_z_hit", curb_z_hit, 0.95);
  private_nh_.param("curb_z_rand", curb_z_rand, 0.05);
  private_nh_.param("curb_sigma_hit", curb_sigma_hit, 0.25);
  
  //this value depends on your threshold in "road_detection" pkg;
  private_nh_.param("curb_rand_range", curb_rand_range, 10.0);
  
  double cro_z_hit, cro_z_short, cro_z_max, cro_z_rand, cro_sigma_hit, cro_lambda_short;
  private_nh_.param("cro_z_hit", cro_z_hit, 0.95);
  private_nh_.param("cro_z_short", cro_z_short, 0.1);
  private_nh_.param("cro_z_max", cro_z_max, 0.05);
  private_nh_.param("cro_z_rand", cro_z_rand, 0.05);
  private_nh_.param("cro_sigma_hit", cro_sigma_hit, 0.2);
  private_nh_.param("cro_lambda_short", cro_lambda_short, 0.1); 
  
  private_nh_.param("laser_z_hit", laser_z_hit, 0.95);
  private_nh_.param("laser_z_short", laser_z_short, 0.1);
  private_nh_.param("laser_z_max", laser_z_max, 0.05);
  private_nh_.param("laser_z_rand", laser_z_rand, 0.05);
  private_nh_.param("laser_sigma_hit", laser_sigma_hit, 0.2);
  private_nh_.param("laser_lambda_short", laser_lambda_short, 0.1);
  
  private_nh_.param("curb_likelihood_max_dist",curb_likelihood_max_dist, 3.0);
  private_nh_.param("laser_likelihood_max_dist",laser_likelihood_max_dist, 2.0);
  
  std::string tmp_model_type;
  laser_model_t laser_model_type;
  private_nh_.param("laser_model_type", tmp_model_type, std::string("likelihood_field"));
  if(tmp_model_type == "beam") laser_model_type = LASER_MODEL_BEAM;
  else if(tmp_model_type == "likelihood_field") laser_model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  else
  {
    ROS_WARN("Unknown laser model type \"%s\"; defaulting to likelihood_field model",tmp_model_type.c_str());
    laser_model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  }
 
  odom_model_t odom_model_type;
  private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
  if(tmp_model_type == "diff") odom_model_type = ODOM_MODEL_DIFF;
  else if(tmp_model_type == "omni") odom_model_type = ODOM_MODEL_OMNI;
  else
  {
    ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model", tmp_model_type.c_str());
    odom_model_type = ODOM_MODEL_DIFF;
  }

  //for pure curb_intersection (3)
  //private_nh_.param("laser_d_thresh_", laser_d_thresh_, 0.2);	 	  //0.2 
  //private_nh_.param("laser_a_thresh_", laser_a_thresh_, M_PI/6);    //M_PI/6.0 
  private_nh_.param("laser_d_thresh_", laser_d_thresh_, 0.5);	 	  //0.2 
  private_nh_.param("laser_a_thresh_", laser_a_thresh_, M_PI/6);    //M_PI/6.0 
  
  private_nh_.param("curb_d_thresh_", curb_d_thresh_, 0.5);
  private_nh_.param("curb_a_thresh_", curb_a_thresh_, M_PI/6.0);
  private_nh_.param("update_min_num", numTresh_, 15);          //15
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  
  private_nh_.param("resample_interval_", resample_interval_, 1); // this parameter matters quite much;
  
  private_nh_.param("recovery_alpha_slow", alpha_slow, 0.001);     //0.001; here I just do not allowed uniform random particles generated;
  private_nh_.param("recovery_alpha_fast", alpha_fast, 0.1);	   
  
  double tmp_tol;
  private_nh_.param("transform_tolerance", tmp_tol, 0.5);
  transform_tolerance_.fromSec(tmp_tol);

  double init_pose[3];
  private_nh_.param("initial_pose_x", init_pose[0], 0.00);
  private_nh_.param("initial_pose_y", init_pose[1], 0.00);
  private_nh_.param("initial_pose_a", init_pose[2], 0.00);  
  
  double init_cov[3];
  private_nh_.param("initial_cov_xx", init_cov[0], 0.3 * 0.3);
  private_nh_.param("initial_cov_yy", init_cov[1], 0.3 * 0.3);
  private_nh_.param("initial_cov_aa", init_cov[2], (M_PI/12.0) * (M_PI/12.0));

  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new tf::TransformListener();

  std::string curb_svc   ="static_curb_map";
  std::string planar_svc ="static_planar_map";
  
  curb_map_   = requestMap(curb_svc);
  planar_map_ = requestMap(planar_svc);
  
  // Create the particle filter
  pf_ = pf_alloc(min_particles, max_particles,
                 alpha_slow, alpha_fast,
                 (pf_init_model_fn_t)MixAmclNode::uniformPoseGenerator,
                 (void *)curb_map_);
  pf_->pop_err = pf_err;
  pf_->pop_z = pf_z;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose[0];
  pf_init_pose_mean.v[1] = init_pose[1];
  pf_init_pose_mean.v[2] = init_pose[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov[0];
  pf_init_pose_cov.m[1][1] = init_cov[1];
  pf_init_pose_cov.m[2][2] = init_cov[2];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_    = false;
  curb_init_  = false;
  laser_init_ = false;
  
  //for pure curb_intersection (1)
  LaserUseFlag_= false;

  CurbUseFlag_ = true;
  CrossingUseFlag_ = true;
  LaserOn_ = true;
  // Instantiate the sensor objects
  // Odometry
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  if(odom_model_type == ODOM_MODEL_OMNI)
    odom_->SetModelOmni(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
  else
    odom_->SetModelDiff(alpha1_, alpha2_, alpha3_, alpha4_, alpha6_);   
  
  
  
  // Laser: Using "planar_map_" here; 
  laser_ = new AMCLLaser(max_beams, planar_map_);
  ROS_ASSERT(laser_);
  if(laser_model_type == LASER_MODEL_BEAM)
    laser_->SetModelBeam(laser_z_hit, laser_z_short, laser_z_max, laser_z_rand, laser_sigma_hit, laser_lambda_short, 0.0);
  else
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(laser_z_hit, laser_z_rand, laser_sigma_hit, laser_likelihood_max_dist);
    ROS_INFO("Done initializing likelihood field model.");
  }
  
  // Curb: Using "curb_map_" here; 
  curb_ = new AMCLCurb(curb_map_);
  curbdata_= new AMCLCurbData();
  ROS_ASSERT(curb_);
  
  ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
  curb_->SetCurbModelLikelihoodField(curb_z_hit, curb_z_rand, curb_sigma_hit, curb_likelihood_max_dist, curb_rand_range);
  ROS_INFO("Done initializing likelihood field model.");
  
  //Crossing
  crossing_ = new AMCLCrossing(curb_map_);
  LeftCroData_ = new AMCLCrossingData(M_PI_2);
  RightCroData_ = new AMCLCrossingData(-M_PI_2);
  ROS_ASSERT(crossing_);
  
  ROS_INFO("Initializing crossing beam model");
  crossing_->SetCrossingModelBeam(cro_z_hit, cro_z_short, cro_z_max, cro_z_rand, cro_sigma_hit, cro_lambda_short);
  ROS_INFO("Done initializing crossing beam model");

  //initial publisher and subscriber;
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2);
  
  curbbaselink_ = nh_.advertise<sensor_msgs::PointCloud>("curbbaselink_", 2);
  
  global_loc_srv_ = nh_.advertiseService("global_localization", &MixAmclNode::globalLocalizationCallback, this);

  
  curb_sidepoints_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud>(nh_, "hybrid_pt", 10);  //200
  curb_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(*curb_sidepoints_sub_, *tf_, odom_frame_id_,100);                                                     
  curb_filter_->registerCallback(boost::bind(&MixAmclNode::curbReceived, this, _1));
  
  
  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "assembled_sick_scan2", 10);         //200
  laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, *tf_, odom_frame_id_, 100);
  laser_scan_filter_->registerCallback(boost::bind(&MixAmclNode::laserReceived,this, _1));
  
  
  initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(nh_, "initialpose", 2);
  initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*initial_pose_sub_, *tf_, global_frame_id_, 2);
  initial_pose_filter_->registerCallback(boost::bind(&MixAmclNode::initialPoseReceived, this, _1));

  initial_pose_sub_old_ = nh_.subscribe("initialpose", 2, &MixAmclNode::initialPoseReceivedOld, this);
  
  invalid_curb_pub_ = nh_.advertise<sensor_msgs::PointCloud>("invalid_curb", 2);
  
  ROS_INFO("Construction Finished");
}


map_t* MixAmclNode::requestMap(const std::string& map_service)
{
	map_t* map = map_alloc();
	ROS_ASSERT(map);

	// get map via RPC
	nav_msgs::GetMap::Request  req;
	nav_msgs::GetMap::Response resp;
	ROS_INFO("Requesting the map...");
	while(!ros::service::call(map_service, req, resp))
	{
		ROS_WARN("Request for map failed; trying again...");
		ros::Duration d(0.5);
		d.sleep();
	}
	ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
			resp.map.info.width,
			resp.map.info.height,
			resp.map.info.resolution);

	map->size_x = resp.map.info.width;
	map->size_y = resp.map.info.height;
	map->scale = resp.map.info.resolution;
	map->origin_x = resp.map.info.origin.position.x + (map->size_x / 2) * map->scale;
	map->origin_y = resp.map.info.origin.position.y + (map->size_y / 2) * map->scale;
	// Convert to player format
	map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
	ROS_ASSERT(map->cells);
	for(int i=0;i<map->size_x * map->size_y;i++)
	{
		if(resp.map.data[i] == 0)
		map->cells[i].occ_state = -1;
		else if(resp.map.data[i] == 100)
		map->cells[i].occ_state = +1;
		else
		map->cells[i].occ_state = 0;
	}

	return map;
}

MixAmclNode::~MixAmclNode()
{
	map_free(curb_map_);
	map_free(planar_map_);
	delete curb_filter_;
	delete curb_sidepoints_sub_;
	delete laser_scan_filter_;
	delete laser_scan_sub_;
	delete initial_pose_filter_;
	delete initial_pose_sub_;
	delete tfb_;
	delete tf_;
	pf_free(pf_);
	
	delete RightCroData_;
	delete LeftCroData_;
	delete crossing_;
	
	delete curbdata_;
	delete curb_;
	delete laser_;
	delete odom_;
	// TODO: delete everything allocated in constructor
}

bool MixAmclNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose, double& x, double& y, double& yaw, double& pitch,
							  const ros::Time& t, const std::string& f)
{
	// Get the robot's pose
	tf::Stamped<tf::Pose> ident (btTransform(tf::createIdentityQuaternion(), btVector3(0,0,0)), t, f);
	try
	{
		this->tf_->transformPose(odom_frame_id_, ident, odom_pose);
	}
	catch(tf::TransformException e)
	{
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return false;
	}
  
	x = odom_pose.getOrigin().x();
	y = odom_pose.getOrigin().y();
	double roll;
	odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);
	return true;
}


pf_vector_t MixAmclNode::uniformPoseGenerator(void* arg)
{
	//map_t* map = (map_t*)arg;
	double min_x, max_x, min_y, max_y, min_thetha, max_thetha;

	min_x 	   	=  -RANDOM_SAMPLE_BOX_X/2.0;
	max_x 		=   RANDOM_SAMPLE_BOX_X/2.0;
	min_y 		=  -RANDOM_SAMPLE_BOX_Y/2.0;
	max_y 		=  	RANDOM_SAMPLE_BOX_Y/2.0;
	min_thetha  =  -RANDOM_SAMPLE_BOX_THETHA/2.0;
	max_thetha  =   RANDOM_SAMPLE_BOX_THETHA/2.0;

	pf_vector_t p;

	//for(;;)
	{
		p.v[0] = min_x + drand48() * (max_x - min_x);
		p.v[1] = min_y + drand48() * (max_y - min_y);
		p.v[2] = min_thetha + drand48() * (max_thetha - min_thetha);
	
		// Check that it's a free cell
		/*
		int i,j;
		i = MAP_GXWX(map, p.v[0]);
		j = MAP_GYWY(map, p.v[1]);
		if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
		break;
		*/ 
	}
	return p;
}

bool MixAmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req,std_srvs::Empty::Response& res)
{
	pf_mutex_.lock();
	ROS_INFO("Initializing with uniform distribution");
	pf_init_model(pf_, (pf_init_model_fn_t)MixAmclNode::uniformPoseGenerator,(void *)curb_map_);
	pf_init_ = false;
	curb_init_ =false;
	laser_init_=false;
	pf_mutex_.unlock();
	return true;
}

void MixAmclNode::curbReceived (const sensor_msgs::PointCloud::ConstPtr& cloud_in)
{
	pf_sample_set_t* set = pf_->sets + pf_->current_set;
	pf_sample_t *sample = set->samples;
	
	ros::Time curbRece_time = cloud_in->header.stamp;
	
    //----------------------------------------------------------------------------------------------------------------------
    //-- For curb measurement validation, "pf_->pos_est" is needed, so tf between global_frame and base_frame is necessary;
    //-- Luckily, because we also use planar laser as input, tf in demand will always be available;
    //-- However, when we only rely on one single measurement input, some other preparation should be done;
    //----------------------------------------------------------------------------------------------------------------------
    bool prior_tf_available       = false;
    bool validate_function_switch = false;
    tf::StampedTransform baseMapTemp;
    try
    {
        tf_->lookupTransform(global_frame_id_, base_frame_id_, curbRece_time, baseMapTemp);
        prior_tf_available       = true;
        validate_function_switch = true;
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Curb Failed to get fresh tf between map and baselink, (%s)", e.what());
        //return;
    }
    
	//ros::Time now1 = ros::Time::now();
	//ROS_DEBUG("1. curbReceived: Time now %lf, curbRece_time %lf", now1.toSec(), curbRece_time.toSec());

	//try to get current transform between "odom" and "base_link";
	tf::StampedTransform 	baseOdomTemp;
	try
	{
		tf_->lookupTransform(odom_frame_id_, base_frame_id_, curbRece_time, baseOdomTemp);
	}
	catch(tf::TransformException e)
	{
		ROS_WARN("Failed to get fresh tf between odom and baselink, (%s)", e.what());
		return;
	}
  
	tf::Stamped<tf::Pose> odom_pose;
	pf_vector_t pose;
	double pitch;
	if(!getOdomPose(odom_pose, pose.v[0], pose.v[1], pose.v[2], pitch, curbRece_time, base_frame_id_))
	{
		ROS_ERROR("Couldn't determine robot's pose associated with curb input");
		return;
	}

	pf_mutex_.lock();

	pf_vector_t curb_delta = pf_vector_zero();
	pf_vector_t odom_delta = pf_vector_zero();
  
	//After curb window updates the filter, it need to be reinitialized; 
	bool force_publication = false;
	bool resampled = false;
	
	///////////////////////////////////////////////////////////////////////////////////////////
	////following block: process the hybrid information, to get "curbdata" and "crossingdata";
	///////////////////////////////////////////////////////////////////////////////////////////

	sensor_msgs::PointCloud leftCrossing;
	sensor_msgs::PointCloud rightCrossing;
	sensor_msgs::PointCloud curbpoints;
	
	for(unsigned int ip=0; ip<cloud_in->points.size(); ip++)
	{
		//actually "cloud_in" only has one point every time; 
		if(cloud_in->points[ip].y == MAX_DISTANCE){leftCrossing.points.push_back(cloud_in->points[ip]);}
		else if(cloud_in->points[ip].y == -MAX_DISTANCE){rightCrossing.points.push_back(cloud_in->points[ip]);}
		else{curbpoints.points.push_back(cloud_in->points[ip]);}
	}
	
	
	if(!pf_init_)
	{
		pf_odom_pose_ = pose; 
		pf_init_ = true;
	}
	
	if(!curb_init_)
	{
		curb_init_ = true;
		force_publication  = true;
		pf_curb_odom_pose_ = pose;
		resample_count_=0;
	}
	
	if(curbdata_->reinit_==true)
	{
		ROS_DEBUG("Curbdata reinit");	
		curbdata_->reinit_ = false;
		curbdata_->beginningTf_ = baseOdomTemp;
		LeftCroData_->beginningTf_ = baseOdomTemp;
		RightCroData_->beginningTf_ = baseOdomTemp;
		
        //------------------------------------------------------
		//to set time to be "beginning time" of the new window;
        //------------------------------------------------------
        
		curbdata_->curbSegment_.header= cloud_in->header;
		
		curbdata_->curbSegment_.points.clear();
		LeftCroData_->FakeSensorPose_.clear();
		RightCroData_->FakeSensorPose_.clear();
		
		for(unsigned int ip=0; ip<curbpoints.points.size(); ip++)
		{
			curbdata_->curbSegment_.points.push_back(curbpoints.points[ip]);
		}
		curbdata_->accumNum_ = curbdata_->curbSegment_.points.size();
		
		for(unsigned int ip=0; ip<leftCrossing.points.size(); ip++)
		{
			pf_vector_t fakepose;
			fakepose.v[0]= leftCrossing.points[ip].x;
			fakepose.v[1]= 0.0;
			fakepose.v[2]= 0.0;
			LeftCroData_->FakeSensorPose_.push_back(fakepose);
		}
		
		for(unsigned int ip=0; ip<rightCrossing.points.size(); ip++)
		{
			pf_vector_t fakepose;
			fakepose.v[0]= rightCrossing.points[ip].x;
			fakepose.v[1]= 0.0;
			//fakepose.v[2] is the angle;
			fakepose.v[2]= 0.0;
			
			RightCroData_->FakeSensorPose_.push_back(fakepose);
		}
		
	}
	else
	{
		//ROS_DEBUG("curbdata accumulate");		
		geometry_msgs::Pose temppose;
		temppose.position.x=0;
		temppose.position.y=0;
		temppose.position.z=0;
		temppose.orientation.x=1;
		temppose.orientation.y=0;
		temppose.orientation.z=0;
		temppose.orientation.w=0;
	
		//--------------------------------------------------------------------------------------------------
		//transfer all the points into the "base_link" coordinate at the beginning time of this curb window;
		//refer to function "initialPoseReceived" for better understanding;
		//--------------------------------------------------------------------------------------------------
		//pay attention here when debugging;
		tf::Transform baselink_old_new  = curbdata_->beginningTf_.inverse() * baseOdomTemp;
		tf::Transform baselink_new_old  = baseOdomTemp.inverse() * curbdata_->beginningTf_;
	
		for(unsigned int ip=0; ip<curbpoints.points.size(); ip++)
		{
			ROS_DEBUG("curb accumulated");
			//remember this, to eliminate "crossing" noise;
			if(curbpoints.points[ip].y>0) {LeftCroData_->FakeSensorPose_.clear();}
			else {RightCroData_->FakeSensorPose_.clear();}
			
			temppose.position.x = curbpoints.points[ip].x;
			temppose.position.y = curbpoints.points[ip].y;
			temppose.position.z = curbpoints.points[ip].z;
			
			tf::Pose tempTfPose;
			tf::poseMsgToTF(temppose, tempTfPose);
		
			tf::Pose oldBaselinkPose = 	baselink_old_new * tempTfPose;
			geometry_msgs::Point32 pointtemp;
		
			pointtemp.x=(float)oldBaselinkPose.getOrigin().x();
			pointtemp.y=(float)oldBaselinkPose.getOrigin().y();
			pointtemp.z=(float)oldBaselinkPose.getOrigin().z();
		
			curbdata_->curbSegment_.points.push_back(pointtemp);
		}
		curbbaselink_.publish(curbdata_->curbSegment_);
		curbdata_->accumNum_ = curbdata_->curbSegment_.points.size();
		
		for(unsigned int ip=0; ip<leftCrossing.points.size(); ip++)
		{
			ROS_DEBUG("LeftCroData_ accumulated");
			
			pf_vector_t fakepose;
			temppose.position.x = leftCrossing.points[ip].x;
			temppose.position.y = 0.0;
			temppose.position.z = 0.0;
			
			tf::Pose tempTfPose;
			tf::poseMsgToTF(temppose, tempTfPose);
		
			tf::Pose oldBaselinkPose = 	baselink_old_new * tempTfPose;
		
			fakepose.v[0] = oldBaselinkPose.getOrigin().x();
			fakepose.v[1] = oldBaselinkPose.getOrigin().y();
			
			double yyaw,ttemp;
			baselink_old_new.getBasis().getEulerYPR(yyaw, ttemp, ttemp);
			fakepose.v[2] = yyaw;
			
			LeftCroData_->FakeSensorPose_.push_back(fakepose);
		}
		
		for(unsigned int ip=0; ip<rightCrossing.points.size(); ip++)
		{
			ROS_DEBUG("RightCroData_ accumulated");
			pf_vector_t fakepose;
			temppose.position.x = rightCrossing.points[ip].x;
			temppose.position.y = 0.0;
			temppose.position.z = 0.0;
			
			tf::Pose tempTfPose;
			tf::poseMsgToTF(temppose, tempTfPose);
		
			tf::Pose oldBaselinkPose = 	baselink_old_new * tempTfPose;
		
			fakepose.v[0] = oldBaselinkPose.getOrigin().x();
			fakepose.v[1] = oldBaselinkPose.getOrigin().y();
			
			double yyaw,ttemp;
			baselink_old_new.getBasis().getEulerYPR(yyaw, ttemp, ttemp);
			fakepose.v[2] = yyaw;
			
			RightCroData_->FakeSensorPose_.push_back(fakepose);
		}
	
		odom_delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
		odom_delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
		odom_delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);
		
		curb_delta.v[0] = pose.v[0] - pf_curb_odom_pose_.v[0];
		curb_delta.v[1] = pose.v[1] - pf_curb_odom_pose_.v[1];
		curb_delta.v[2] = angle_diff(pose.v[2], pf_curb_odom_pose_.v[2]);
		

		// served as the normalizer when doing analysis;
		double distTolastCurb 	  = sqrt(curb_delta.v[0]*curb_delta.v[0]+curb_delta.v[1]*curb_delta.v[1]);
		double distTolastOdom     = sqrt(odom_delta.v[0]*odom_delta.v[0]+odom_delta.v[1]*odom_delta.v[1]);
		
		// See if we should update the filter
		bool update1 = fabs(curb_delta.v[0]) > curb_d_thresh_ ||
					   fabs(curb_delta.v[1]) > curb_d_thresh_ ||
					   fabs(curb_delta.v[2]) > curb_a_thresh_;

		bool update2 = (curbdata_->accumNum_) > numTresh_;
		bool update3 = (curbdata_->accumNum_>10) && (distTolastOdom>0.6||odom_delta.v[2]> M_PI/6.0);
		
		bool update4 = (LeftCroData_->FakeSensorPose_.size()+RightCroData_->FakeSensorPose_.size())> CROSSING_TRESH || LeftCroData_->FakeSensorPose_.size()>5 || RightCroData_->FakeSensorPose_.size()>5 ;
		if(update4){ROS_DEBUG("~~~Using Crossing Information~~");}
		
		bool updateAction = (update1 && (update2||update4))||update3;
		bool updateMeas= (update1 && (update2||update4))||update3;
		
		bool Shift_Flag = (distTolastOdom >1||odom_delta.v[2]> M_PI/3.0);
		
		// Prediction Step;
		if(updateAction)
		{
			if(!update4)
			{
				double temp_alpha1=0.04;	//0.09
				double temp_alpha2=0.04;	//0.09
				double temp_alpha3=0.2;
				double temp_alpha4=0.2;
				double temp_alpha6=0.01;
				odom_->SetModelDiff(temp_alpha1, temp_alpha2, temp_alpha3, temp_alpha4, temp_alpha6);
			}
            else
            {
                //added on 2011-12-13;
                //later after changing the crossing data type, this can be further improved;
                double temp_alpha1=0.0009;	
				double temp_alpha2=0.0009;	
				double temp_alpha3=0.2;
				double temp_alpha4=0.2;
				double temp_alpha6=0.01;
				odom_->SetModelDiff(temp_alpha1, temp_alpha2, temp_alpha3, temp_alpha4, temp_alpha6);
            }
			
			if(Shift_Flag)
			{
				double temp_alpha1=0.01;
				double temp_alpha2=0.01;
				double temp_alpha3=0.01;
				double temp_alpha4=0.01;
				double temp_alpha6=0.01;
				odom_->SetModelDiff(temp_alpha1, temp_alpha2, temp_alpha3, temp_alpha4, temp_alpha6);
			}
			else if(LaserUseFlag_)
			{
				double temp_alpha1=0.2;
				double temp_alpha2=0.2;
				double temp_alpha3=0.2;
				double temp_alpha4=0.2;
				double temp_alpha6=0.09;
				odom_->SetModelDiff(temp_alpha1, temp_alpha2, temp_alpha3, temp_alpha4, temp_alpha6);
			}
			
			ROS_DEBUG("---------------------update action----------------");
			ROS_DEBUG("distTolastOdom: %5f", distTolastOdom);
			
			AMCLOdomData odata;
			odata.pose  = pose;
			odata.delta = odom_delta;
			odata.pitch = pitch;
			odom_->UpdateAction(pf_, (AMCLSensorData*)&odata, (void *)curb_map_);
	
            //--------------------------------------------------------------------------------
            //to calculate "pf_->Pos_Est" based on (1)latest max_weight_hyp and (2)odom_delta;
            //--------------------------------------------------------------------------------
            if(prior_tf_available)
            {
                double x_tmp, y_tmp, tmp, yaw_tmp;
                x_tmp = baseMapTemp.getOrigin().x();
                y_tmp = baseMapTemp.getOrigin().y();
                baseMapTemp.getBasis().getEulerYPR(yaw_tmp, tmp, tmp);
                pf_->Pos_Est.v[0] = float(x_tmp);
                pf_->Pos_Est.v[1] = float(y_tmp);
                pf_->Pos_Est.v[2] = float(yaw_tmp);
            }
                
			pf_odom_pose_ = pose;
			//the following line is quite important and interesting;
			//seperate "action updating" and "measurement updating" 
			pf_curb_odom_pose_ = pose;
		}
	
    
		// Measurement Update;
		/////////////////////////////////////////////////////////////
		// if update is true, prepare the curbsegment data;
		// transfer them into the new baselink coordinate;
		// update the filter;
		// also remember to do some post procession after using it;
		/////////////////////////////////////////////////////////////    

		if(updateMeas)
		{
			ROS_DEBUG("---------------------update Meas----------------");
			ROS_DEBUG("distTolastCurb: %5f, curb accumNum: %d, left crossing: %d, right crossing: %d", distTolastCurb, curbdata_->accumNum_, LeftCroData_->FakeSensorPose_.size(), RightCroData_->FakeSensorPose_.size());
			
            LeftCroData_->Pose_Est_ = pf_->Pos_Est;
            RightCroData_->Pose_Est_ = pf_->Pos_Est;
            curbdata_ ->Pose_Est_ = pf_->Pos_Est;
            
			//this is quite important!!!
			//Because in the beginning I missed this command, the whole program cannot work properly;
			curbdata_->reinit_ = true;
			
			sensor_msgs::PointCloud temppointcloud;
			
			//Do some preparing work for our analysis;
			//These pointclouds here are in the "base_link" coordinate; 
			newLeftMeas_.header =curbdata_->curbSegment_.header;
			newLeftMeas_.points.clear();
			newRightMeas_.header =curbdata_->curbSegment_.header;
			newRightMeas_.points.clear();

			for(unsigned int ip=0; ip<curbdata_->curbSegment_.points.size(); ip++)
			{
				temppose.position.x = curbdata_->curbSegment_.points[ip].x;
				temppose.position.y = curbdata_->curbSegment_.points[ip].y;
				temppose.position.z = curbdata_->curbSegment_.points[ip].z;
			
				tf::Pose tempTfPose;
				tf::poseMsgToTF(temppose, tempTfPose);
				tf::Pose newBaselinkPose = 	baselink_new_old*tempTfPose;
				geometry_msgs::Point32 pointtemp;
			
				pointtemp.x=(float)newBaselinkPose.getOrigin().x();
				pointtemp.y=(float)newBaselinkPose.getOrigin().y();
				pointtemp.z=(float)newBaselinkPose.getOrigin().z();
			
				temppointcloud.points.push_back(pointtemp);
				//****************to emphysize the curb measurements;***************
				temppointcloud.points.push_back(pointtemp);
			}
			curbdata_->curbSegment_.points=temppointcloud.points;
            //----------------------------------------------------
            //important when showing invalidCurbSeg_;
            //right now we use current time stamp;
            //----------------------------------------------------
            curbdata_->curbSegment_.header.stamp = curbRece_time;
			curbdata_->sensor = curb_;
			
			for(unsigned int ip=0; ip< RightCroData_->FakeSensorPose_.size(); ip++)
			{
				temppose.position.x = (float)(RightCroData_->FakeSensorPose_[ip].v[0]);
				temppose.position.y = (float)(RightCroData_->FakeSensorPose_[ip].v[1]);
				temppose.position.z = 0.0;
			
				tf::Pose tempTfPose;
				tf::poseMsgToTF(temppose, tempTfPose);
				tf::Pose newBaselinkPose = 	baselink_new_old*tempTfPose;
				
				RightCroData_->FakeSensorPose_[ip].v[0] = newBaselinkPose.getOrigin().x();
				RightCroData_->FakeSensorPose_[ip].v[1] = newBaselinkPose.getOrigin().y();
			
				double yyaw,ttemp;
				baselink_new_old.getBasis().getEulerYPR(yyaw, ttemp, ttemp);
				RightCroData_->FakeSensorPose_[ip].v[2] = RightCroData_->FakeSensorPose_[ip].v[2]+yyaw;
			}
			
			for(unsigned int ip=0; ip< LeftCroData_->FakeSensorPose_.size(); ip++)
			{
				temppose.position.x = (float)(LeftCroData_->FakeSensorPose_[ip].v[0]);
				temppose.position.y = (float)(LeftCroData_->FakeSensorPose_[ip].v[1]);
				temppose.position.z = 0.0;
			
				tf::Pose tempTfPose;
				tf::poseMsgToTF(temppose, tempTfPose);
				tf::Pose newBaselinkPose = 	baselink_new_old*tempTfPose;
				
				LeftCroData_->FakeSensorPose_[ip].v[0] = newBaselinkPose.getOrigin().x();
				LeftCroData_->FakeSensorPose_[ip].v[1] = newBaselinkPose.getOrigin().y();
			
				double yyaw,ttemp;
				baselink_new_old.getBasis().getEulerYPR(yyaw, ttemp, ttemp);
				LeftCroData_->FakeSensorPose_[ip].v[2] = LeftCroData_->FakeSensorPose_[ip].v[2]+yyaw;
			}
			
			//////////////////////////////////////////////////////////////////////////////////////
			// to keep in accordance with original code;
			// Segmentation Fault once found here!!!
			// because my curbdata_ here is a pointer, I shouldn't use "&" as original code does;
			//////////////////////////////////////////////////////////////////////////////////////
				
				
            curbdata_->sensor = curb_;
            bool *FlagPointer = &CurbUseFlag_;
            curb_->UpdateSensor(pf_, (AMCLSensorData*) curbdata_, FlagPointer, validate_function_switch);
            invalid_curb_pub_.publish(curbdata_->invalidCurbSeg_);
            
            if(LaserUseFlag_){numTresh_ = 15;}
            else{numTresh_ = 15;}
            
            bool *tempflagpointer = &CrossingUseFlag_;
            if(!LaserUseFlag_)
            {
                LeftCroData_->sensor = crossing_;
                crossing_->UpdateSensor(pf_, (AMCLSensorData*) LeftCroData_, tempflagpointer, validate_function_switch);
                RightCroData_->sensor = crossing_;
                crossing_->UpdateSensor(pf_, (AMCLSensorData*) RightCroData_, tempflagpointer, validate_function_switch);
            }
        
            if(CurbUseFlag_||CrossingUseFlag_)
            {
                ROS_DEBUG("----------Curb--Crossing--Curb---------");
                if(!(++resample_count_ % resample_interval_))
                {
                    pf_->w_diff_tresh=0.2;
                    pf_->w_diff_setvalue=0.02;
                    
                    pf_update_resample(pf_);
                    resampled = true;
                }
                
                set = pf_->sets + pf_->current_set;
                sample = set->samples;

                // Publish the resulting cloud
                // TODO: set maximum rate for publishing
                geometry_msgs::PoseArray cloud_msg;
                cloud_msg.header.stamp = ros::Time::now();
                cloud_msg.header.frame_id = global_frame_id_;
                cloud_msg.poses.resize(set->sample_count);
                for(int i=0;i<set->sample_count;i++)
                {
                    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                                    btVector3(set->samples[i].pose.v[0],set->samples[i].pose.v[1], 0)),
                                    cloud_msg.poses[i]);
                }
                particlecloud_pub_.publish(cloud_msg);
            }
        }
	}
	
	if(resampled||force_publication)
	{
		
		/////////////////////////////////////////////////////////////////////////////////////////////////////
		////if(resampled||(force_publication && (!curb_init_)))
		//////curb_init_ = true;
		// Once found a small bug here;
		// "curb_init_" is used to flag the filter init, also quite important; 
		// if this flag is not used, every time curbdata_ gets reinit, the loop function into this branch;
		// the branch use stored hypotheses to generate new tf_;
		// However, except the time of curb_init_ when hypothese is determined by initial poses, 
		// run-time hypotheses are always stale because they are set from last trigger;
		////////////////////////////////////////////////////////////////////////////////////////////////////
		
		//ROS_DEBUG("resampled or force_publish");
		// Read out the current hypotheses
		
		
		set = pf_->sets + pf_->current_set;
		sample = set->samples;
		
		double max_weight = 0.0;
		int max_weight_hyp = -1;
		std::vector<amcl_hyp_t> hyps;
		hyps.resize(pf_->sets[pf_->current_set].cluster_count);
		
		for(int hyp_count = 0; hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
		{
			double weight;
			pf_vector_t pose_mean;
			pf_matrix_t pose_cov;
			
			if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
			{
				ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
				break;
			}
		
			hyps[hyp_count].weight = weight;
			hyps[hyp_count].pf_pose_mean = pose_mean;
			hyps[hyp_count].pf_pose_cov = pose_cov;
		
			if(hyps[hyp_count].weight > max_weight)
			{
				max_weight = hyps[hyp_count].weight;
				max_weight_hyp = hyp_count;
			}
		}

		if(max_weight > 0.0)
		{
			//ROS_DEBUG("max_weight>0");
	  
			ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
					hyps[max_weight_hyp].pf_pose_mean.v[0],
					hyps[max_weight_hyp].pf_pose_mean.v[1],
					hyps[max_weight_hyp].pf_pose_mean.v[2]);
			
			pf_->Pos_Est = hyps[max_weight_hyp].pf_pose_mean;

			geometry_msgs::PoseWithCovarianceStamped p;
			// Fill in the header
			p.header.frame_id = global_frame_id_;
			p.header.stamp = curbRece_time;
			// Copy in the pose
			p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
			p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
									p.pose.pose.orientation);
			// Copy in the covariance, converting from 3-D to 6-D
			pf_sample_set_t* set = pf_->sets + pf_->current_set;
			for(int i=0; i<2; i++)
			{
				for(int j=0; j<2; j++)
				{
				// Report the overall filter covariance, rather than the
				// covariance for the highest-weight cluster
				//p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
				p.pose.covariance[6*i+j] = set->cov.m[i][j];
				}
			}
		
			// Report the overall filter covariance, rather than the
			// covariance for the highest-weight cluster
			//p.covariance[6*3+3] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
			p.pose.covariance[6*3+3] = set->cov.m[2][2];
	
			pose_pub_.publish(p);
			last_published_pose = p;
	
			ROS_DEBUG("New pose: %6.3f %6.3f %6.3f",
					hyps[max_weight_hyp].pf_pose_mean.v[0],
					hyps[max_weight_hyp].pf_pose_mean.v[1],
					hyps[max_weight_hyp].pf_pose_mean.v[2]);
					
			ROS_DEBUG("Covariance: %6.3f %6.3f %6.3f",
					p.pose.covariance[6*0+0],
					p.pose.covariance[6*1+1],
					p.pose.covariance[6*3+3]);

			// subtracting "base to odom" from "map to base" and send map to odom instead
			tf::Stamped<tf::Pose> odom_to_map;
			try
			{
				tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
									tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
												hyps[max_weight_hyp].pf_pose_mean.v[1],
												0.0));
												
				//tmp_tf.inverse() is tf from "map" to "base_link";						
				tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
													curbRece_time,
													base_frame_id_);
													
				//get the tf from "map" to "odom";										
				this->tf_->transformPose(odom_frame_id_,
										tmp_tf_stamped,
										odom_to_map);
			}
			catch(tf::TransformException)
			{
				ROS_DEBUG("Failed to subtract base to odom transform");
				pf_mutex_.unlock();
				return;
			}

			latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),tf::Point(odom_to_map.getOrigin()));
			latest_tf_valid_ = true;

			// We want to send a transform that is good up until a
			// tolerance time so that odom can be used
			ros::Time transform_expiration = (curbRece_time +transform_tolerance_);
			tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),transform_expiration, global_frame_id_, odom_frame_id_);
			this->tfb_->sendTransform(tmp_tf_stamped);
      
			//ROS_DEBUG("newly generated tf Time %lf",transform_expiration.toSec());
			
			//ros::Time nowa = ros::Time::now();
			//ROS_DEBUG("Curb: Time Now %lf, generate new tf %lf", nowa.toSec(), transform_expiration.toSec());
			
			sent_first_transform_ = true;
		}
		else
		{
			ROS_ERROR("No pose!");
		}
    
	}  
	else if(latest_tf_valid_)
	{	
		//ROS_DEBUG("Be happy: republish last tf");
		//Nothing changed, so we'll just republish the last transform, to keep everybody happy.

		ros::Time transform_expiration = (curbRece_time + transform_tolerance_);
		tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration, global_frame_id_, odom_frame_id_);	
		
		//ros::Time nowb = ros::Time::now();
		//ROS_DEBUG("Curb: Time Now %lf, Publish old tf %lf", nowb.toSec(), transform_expiration.toSec());
										
		this->tfb_->sendTransform(tmp_tf_stamped);
	 
		//ROS_DEBUG("last tf republish at %lf",transform_expiration.toSec());

		// Is it time to save our last pose to the param server
		ros::Time now = ros::Time::now();
		if((save_pose_period.toSec() > 0.0) && (now - save_pose_last_time) >= save_pose_period)
		{
			// We need to apply the last transform to the latest odom pose to get
			// the latest map pose to store.  We'll take the covariance from
			// last_published_pose.
			tf::Pose map_pose = latest_tf_.inverse() * odom_pose;
			double yaw,pitch,roll;
			map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

			private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
			private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
			private_nh_.setParam("initial_pose_a", yaw);
			private_nh_.setParam("initial_cov_xx",last_published_pose.pose.covariance[6*0+0]);
			private_nh_.setParam("initial_cov_yy",last_published_pose.pose.covariance[6*1+1]);
			private_nh_.setParam("initial_cov_aa",last_published_pose.pose.covariance[6*3+3]);
			save_pose_last_time = now;
		}	
	}

	pf_mutex_.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// for pure curb_intersection: here laser is just used to trigger tf publication...
// the subscribed topic can be the scan used to extract curb features; but it doesn't really matter;
// here I just choose a lazy way to do all the things, and you may compare "curb_intersection_node.cpp" to "mix_amcl_node.cpp".
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MixAmclNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
	ros::Time laserRece_time = laser_scan->header.stamp;
    
    tf::StampedTransform baseMapTemp;
    try
    {
        tf_->lookupTransform(global_frame_id_, base_frame_id_, laserRece_time, baseMapTemp);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Laser Failed to get fresh tf between map and baselink, (%s)", e.what());
        //return;
    }
    
	// Where was the robot when this scan was taken?
	tf::Stamped<tf::Pose> odom_pose;
	pf_vector_t pose;
	double pitch;
	if(!getOdomPose(odom_pose, pose.v[0], pose.v[1], pose.v[2], pitch, laser_scan->header.stamp, base_frame_id_))
	{
		ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
		return;
	}
    
	pf_mutex_.lock();
	bool force_publication = false;
    
	if(!pf_init_) 
	{
		pf_odom_pose_ = pose;
		pf_init_ = true;
        force_publication = true;
	}

	if(!laser_init_)
	{
		// Pose at last filter update
		pf_laser_odom_pose_ = pose;
	
		// Filter is now initialized
		laser_init_ = true;
		force_publication = true;
		resample_count_ = 0;
	}

    if(force_publication)
	{
		// Read out the current hypotheses
		double max_weight = 0.0;
		int max_weight_hyp = -1;
		std::vector<amcl_hyp_t> hyps;
		hyps.resize(pf_->sets[pf_->current_set].cluster_count);
		for(int hyp_count = 0;hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
		{
			double weight;
			pf_vector_t pose_mean;
			pf_matrix_t pose_cov;
			if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
			{
				ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
				break;
			}

			hyps[hyp_count].weight = weight;
			hyps[hyp_count].pf_pose_mean = pose_mean;
			hyps[hyp_count].pf_pose_cov = pose_cov;
		
			if(hyps[hyp_count].weight > max_weight)
			{
				max_weight = hyps[hyp_count].weight;
				max_weight_hyp = hyp_count;
			}
		}
        
		if(max_weight > 0.0)
		{
			ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
						hyps[max_weight_hyp].pf_pose_mean.v[0],
						hyps[max_weight_hyp].pf_pose_mean.v[1],
						hyps[max_weight_hyp].pf_pose_mean.v[2]);

			pf_->Pos_Est = hyps[max_weight_hyp].pf_pose_mean;
			
			geometry_msgs::PoseWithCovarianceStamped p;
			// Fill in the header
			p.header.frame_id = global_frame_id_;
			p.header.stamp = laser_scan->header.stamp;
			// Copy in the pose
			p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
			p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
									p.pose.pose.orientation);
			// Copy in the covariance, converting from 3-D to 6-D
			pf_sample_set_t* set = pf_->sets + pf_->current_set;
			for(int i=0; i<2; i++)
			{
				for(int j=0; j<2; j++)
				{
				// Report the overall filter covariance, rather than the
				// covariance for the highest-weight cluster
				//p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
				p.pose.covariance[6*i+j] = set->cov.m[i][j];
				}
			}
			// Report the overall filter covariance, rather than the
			// covariance for the highest-weight cluster
			//p.covariance[6*3+3] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
			p.pose.covariance[6*3+3] = set->cov.m[2][2];
			
            //pose_pub_.publish(p);
			last_published_pose = p;
	
			ROS_DEBUG("New pose: %6.3f %6.3f %6.3f",
					hyps[max_weight_hyp].pf_pose_mean.v[0],
					hyps[max_weight_hyp].pf_pose_mean.v[1],
					hyps[max_weight_hyp].pf_pose_mean.v[2]);

			// subtracting base to odom from map to base and send map to odom instead
			tf::Stamped<tf::Pose> odom_to_map;
			try
			{
				tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
									tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
												hyps[max_weight_hyp].pf_pose_mean.v[1],
												0.0));
				tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
													laser_scan->header.stamp,
													base_frame_id_);
				this->tf_->transformPose(odom_frame_id_, tmp_tf_stamped, odom_to_map);
			}
			catch(tf::TransformException)
			{
				ROS_ERROR("Failed to subtract base to odom transform");
				pf_mutex_.unlock();
				return;
			}

			latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),tf::Point(odom_to_map.getOrigin()));
			latest_tf_valid_ = true;

			// We want to send a transform that is good up until a
			// tolerance time so that odom can be used
			ros::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
			tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration, global_frame_id_, odom_frame_id_);
			
			//ros::Time nowc = ros::Time::now();
			//ROS_DEBUG("Laser: Time Now %lf, Publish new tf %lf", nowc.toSec(), transform_expiration.toSec());
			
			this->tfb_->sendTransform(tmp_tf_stamped);
			sent_first_transform_ = true;
		}
		else
		{
			ROS_ERROR("No pose!");
		}
        
        pf_sample_set_t* set = pf_->sets + pf_->current_set;
		
        
        //////////////////////////////////////////////////////////////////////////////////////
        //for pure curb_intersection;
        //add to show particles when new initial pose is given, or odom propagate particles;
        //////////////////////////////////////////////////////////////////////////////////////
        geometry_msgs::PoseArray cloud_msg;
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = global_frame_id_;
        cloud_msg.poses.resize(set->sample_count);
        ROS_DEBUG("Sample count: %d", set->sample_count);
        for(int i=0;i<set->sample_count;i++)
        {
            tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                            btVector3(set->samples[i].pose.v[0],set->samples[i].pose.v[1], 0)),
                            cloud_msg.poses[i]);
        }
        ROS_DEBUG("cloud_msg size: %d", cloud_msg.poses.size());
        particlecloud_pub_.publish(cloud_msg);
        
        
	}
	else if(latest_tf_valid_)
	{
		ROS_DEBUG("laser triggers last tf republication");
		// Nothing changed, so we'll just republish the last transform, to keep
		// everybody happy.
		
		ros::Time transform_expiration = (laser_scan->header.stamp + transform_tolerance_);
		tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),transform_expiration, global_frame_id_, odom_frame_id_);
		
		//ros::Time nowd = ros::Time::now();
		//ROS_DEBUG("Laser: Time Now %lf, Publish last tf %lf", nowd.toSec(), transform_expiration.toSec());
		
		this->tfb_->sendTransform(tmp_tf_stamped);

		// Is it time to save our last pose to the param server
		ros::Time now = ros::Time::now();
		if((save_pose_period.toSec() > 0.0) && (now - save_pose_last_time) >= save_pose_period)
		{
			// We need to apply the last transform to the latest odom pose to get
			// the latest map pose to store.  We'll take the covariance from
			// last_published_pose.
			tf::Pose map_pose = latest_tf_.inverse() * odom_pose;
			double yaw,pitch,roll;
			map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

			private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
			private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
			private_nh_.setParam("initial_pose_a", yaw);
			private_nh_.setParam("initial_cov_xx", last_published_pose.pose.covariance[6*0+0]);
			private_nh_.setParam("initial_cov_yy", last_published_pose.pose.covariance[6*1+1]);
			private_nh_.setParam("initial_cov_aa", last_published_pose.pose.covariance[6*3+3]);
			save_pose_last_time = now;
		}
	}
	
	pf_mutex_.unlock();
}

double MixAmclNode::getYaw(tf::Pose& t)
{
	double yaw, pitch, roll;
	btMatrix3x3 mat = t.getBasis();
	mat.getEulerYPR(yaw,pitch,roll);
	return yaw;
}

void MixAmclNode::initialPoseReceivedOld(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	// Support old behavior, where null frame ids were accepted.
	if(msg->header.frame_id == "")
	{
		ROS_WARN("Received initialpose message with header.frame_id == "".  This behavior is deprecated; you should always set the frame_id");
		initialPoseReceived(msg);
	}
}

void MixAmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	ROS_INFO("***********************initial PoseReceived*************************");
	// In case the client sent us a pose estimate in the past, integrate the
	// intervening odometric change.
	tf::StampedTransform tx_odom;
	try
	{
		tf_->lookupTransform(base_frame_id_, ros::Time::now(),
							base_frame_id_, msg->header.stamp,
							global_frame_id_, tx_odom);
	}
	catch(tf::TransformException e)
	{
		// If we've never sent a transform, then this is normal, because the
		// global_frame_id_ frame doesn't exist.  We only care about in-time
		// transformation for on-the-move pose-setting, so ignoring this
		// startup condition doesn't really cost us anything.
		if(sent_first_transform_)
		ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
		tx_odom.setIdentity();
	}

	tf::Pose pose_old, pose_new;
	tf::poseMsgToTF(msg->pose.pose, pose_old);
	pose_new = tx_odom.inverse() * pose_old;

	ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
			ros::Time::now().toSec(),
			pose_new.getOrigin().x(),
			pose_new.getOrigin().y(),
			getYaw(pose_new));
	// Re-initialize the filter
	pf_vector_t pf_init_pose_mean = pf_vector_zero();
	pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
	pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
	pf_init_pose_mean.v[2] = getYaw(pose_new);
	pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
	// Copy in the covariance, converting from 6-D to 3-D
	for(int i=0; i<2; i++)
	{
		for(int j=0; j<2; j++)
		{
		pf_init_pose_cov.m[i][j] = msg->pose.covariance[6*i+j];
		}
	}
	pf_init_pose_cov.m[2][2] = msg->pose.covariance[6*3+3];

	pf_mutex_.lock();
	pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
	pf_init_ = false;
	pf_mutex_.unlock();
  
	ROS_INFO("************************pf_ gets initialized with initialPosereceived *************************");
}
