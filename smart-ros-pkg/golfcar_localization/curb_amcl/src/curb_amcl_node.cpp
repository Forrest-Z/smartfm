/* NUS Golf Cart
 * CurbAmclNode is based on the original Amcl node;
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

class CurbAmclNode
{
  public:
    CurbAmclNode();
    ~CurbAmclNode();

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
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
                                    
    void curbReceived (const sensor_msgs::PointCloud::ConstPtr& cloud_in);
    
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void initialPoseReceivedOld(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    double getYaw(tf::Pose& t);

    //parameter for what odom to use
    std::string odom_frame_id_;
    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    map_t* map_;
    
    // @_@ the following parameters are not used;
    char* mapdata;
    int sx, sy;
    double resolution;
    
    //move it here as class member;
    double alpha1, alpha2, alpha3, alpha4, alpha5;
    double alpha6;
    
    ////////////////////////////////////////////////////////////////////////////
    //the incoming information data is actually "two beginning points for curb";
    ////////////////////////////////////////////////////////////////////////////
    message_filters::Subscriber<sensor_msgs::PointCloud> *curb_sidepoints_sub_;
	tf::MessageFilter<sensor_msgs::PointCloud> * curb_filter_;
		
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_sub_;
    tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_filter_;

    // Particle filter
    pf_t *pf_;
    boost::mutex pf_mutex_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int numTresh_;
    
    int resample_interval_;
    int resample_count_;

    AMCLOdom* odom_;
    AMCLCurb* curb_;
    AMCLCurbData* curbdata_;

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    map_t* requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,double& x, double& y, double& yaw, double& pitch,
                     const ros::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_pub_;
    ros::Publisher particlecloud_pub_;
    
    //for debugging purposes;
    ros::Publisher curbbaselink_;
    
    ros::ServiceServer global_loc_srv_;
    ros::Subscriber initial_pose_sub_old_;
    
    ////////////////////////////////////////////////////////////////
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
    

	void measAnalysis(sensor_msgs::PointCloud& oldMeas, sensor_msgs::PointCloud& newMeas, const double Disnormalizer, geometry_msgs::PointStamped& AnalysisResult, const ros::Publisher& Analysis_pub);
	
	geometry_msgs::PointStamped 			    base_link_point_;    
	ros::Publisher								baselink_pt_pub_;
	geometry_msgs::PointStamped 			    position_covariance_;
    ros::Publisher								cov_pub_;
};

#define USAGE "USAGE: amcl"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "curb_amcl");
  ros::NodeHandle nh;

  CurbAmclNode an;

  ros::spin();

  return(0);
}

CurbAmclNode::CurbAmclNode() :
        sent_first_transform_(false),
        latest_tf_valid_(false),
        map_(NULL),
        pf_(NULL),
        resample_count_(0),
	private_nh_("~")
{
  // Grab params off the param server
  int min_particles, max_particles;
  //double alpha1, alpha2, alpha3, alpha4, alpha5;
  double alpha_slow, alpha_fast;
  double z_hit, z_rand, sigma_hit, rand_range;
  double pf_err, pf_z;

  double tmp;
  private_nh_.param("gui_publish_rate", tmp, -1.0);
  gui_publish_period = ros::Duration(1.0/tmp);
  private_nh_.param("save_pose_rate", tmp, 1.0);
  save_pose_period = ros::Duration(1.0/tmp);

  private_nh_.param("min_particles", min_particles, 1000);
  private_nh_.param("max_particles", max_particles, 5000);
  private_nh_.param("kld_err", pf_err, 0.01);
  private_nh_.param("kld_z", pf_z, 0.99);
  private_nh_.param("odom_alpha1", alpha1, 0.01);
  private_nh_.param("odom_alpha2", alpha2, 0.01);
  private_nh_.param("odom_alpha3", alpha3, 0.2);
  private_nh_.param("odom_alpha4", alpha4, 0.2);
  private_nh_.param("odom_alpha5", alpha5, 0.01);
  private_nh_.param("odom_alpha6", alpha6, 0.09);

  private_nh_.param("laser_z_hit", z_hit, 0.95);
  private_nh_.param("laser_z_rand", z_rand, 0.05);
  private_nh_.param("laser_sigma_hit", sigma_hit, 0.25);
  private_nh_.param("rand_range", rand_range, 40.0);
  
  double laser_likelihood_max_dist;
  private_nh_.param("laser_likelihood_max_dist",laser_likelihood_max_dist, 3.0);
  std::string tmp_model_type;
 
  odom_model_t odom_model_type;
  private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
  if(tmp_model_type == "diff") odom_model_type = ODOM_MODEL_DIFF;
  else if(tmp_model_type == "omni") odom_model_type = ODOM_MODEL_OMNI;
  else
  {
    ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model", tmp_model_type.c_str());
    odom_model_type = ODOM_MODEL_DIFF;
  }

  private_nh_.param("update_min_d", d_thresh_, 0.5);
  private_nh_.param("update_min_a", a_thresh_, M_PI/6.0);
  private_nh_.param("update_min_num", numTresh_, 25);
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh_.param("resample_interval", resample_interval_, 1);
  double tmp_tol;
  private_nh_.param("transform_tolerance", tmp_tol, 0.1);
  private_nh_.param("recovery_alpha_slow", alpha_slow, 0.1);  // alpha_slow, 0.001; alpha_fast, 0.1;
  private_nh_.param("recovery_alpha_fast", alpha_fast, 0.1);

  transform_tolerance_.fromSec(tmp_tol);
  
  //for debugging purposes, select several sets of init_pose[i];
  // straight-line: (20.0, 148.6, 1.50);

  double init_pose[3];
  private_nh_.param("initial_pose_x", init_pose[0], 20.0);
  private_nh_.param("initial_pose_y", init_pose[1], 148.6);
  private_nh_.param("initial_pose_a", init_pose[2], 1.50);  
  
  double init_cov[3];
  private_nh_.param("initial_cov_xx", init_cov[0], 0.3 * 0.3);
  private_nh_.param("initial_cov_yy", init_cov[1], 0.3 * 0.3);
  private_nh_.param("initial_cov_aa", init_cov[2], (M_PI/12.0) * (M_PI/12.0));

  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new tf::TransformListener();

  map_ = requestMap();
  // Create the particle filter
  pf_ = pf_alloc(min_particles, max_particles,
                 alpha_slow, alpha_fast,
                 (pf_init_model_fn_t)CurbAmclNode::uniformPoseGenerator,
                 (void *)map_);
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
  pf_init_ = false;
  
  
  // Instantiate the sensor objects
  // Odometry
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  if(odom_model_type == ODOM_MODEL_OMNI)
    odom_->SetModelOmni(alpha1, alpha2, alpha3, alpha4, alpha5);
  else
    odom_->SetModelDiff(alpha1, alpha2, alpha3, alpha4, alpha6);   
  
  // Curb
  curb_ = new AMCLCurb(map_);
  curbdata_= new AMCLCurbData();
  ROS_ASSERT(curb_);
  
  ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
  curb_->SetCurbModelLikelihoodField(z_hit, z_rand, sigma_hit,laser_likelihood_max_dist,rand_range);
  ROS_INFO("Done initializing likelihood field model.");

  //initial publisher and subscriber;
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2);
  
  curbbaselink_ = nh_.advertise<sensor_msgs::PointCloud>("curbbaselink_", 2);
  
  global_loc_srv_ = nh_.advertiseService("global_localization", &CurbAmclNode::globalLocalizationCallback, this);
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  curb_sidepoints_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud>(nh_, "curb_points", 100);
  curb_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(*curb_sidepoints_sub_, 
                                                        *tf_, 
                                                        odom_frame_id_, 
                                                        100);
                                                        
  curb_filter_->registerCallback(boost::bind(&CurbAmclNode::curbReceived, this, _1));
  
  initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(nh_, "initialpose", 2);
  initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*initial_pose_sub_, *tf_, global_frame_id_, 2);
  initial_pose_filter_->registerCallback(boost::bind(&CurbAmclNode::initialPoseReceived, this, _1));

  initial_pose_sub_old_ = nh_.subscribe("initialpose", 2, &CurbAmclNode::initialPoseReceivedOld, this);

  //for analysis purposes;
  LeftAnalysis_pub_  = nh_.advertise<geometry_msgs::PointStamped>("LeftAnalysis",  5);
  RightAnalysis_pub_ = nh_.advertise<geometry_msgs::PointStamped>("RightAnalysis", 5);
  
  baselink_pt_pub_  = nh_.advertise<geometry_msgs::PointStamped>("baselink_in_map",  5);
  cov_pub_		    = nh_.advertise<geometry_msgs::PointStamped>("localization_cov", 5);
  
  ROS_INFO("Construction Finished");
}


map_t*
CurbAmclNode::requestMap()
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
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

CurbAmclNode::~CurbAmclNode()
{
  map_free(map_);
  delete curb_filter_;
  delete curb_sidepoints_sub_;
  delete initial_pose_filter_;
  delete initial_pose_sub_;
  delete tfb_;
  delete tf_;
  pf_free(pf_);
  delete curbdata_;
  delete curb_;
  delete odom_;
  // TODO: delete everything allocated in constructor
}

bool
CurbAmclNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      double& x, double& y, double& yaw, double& pitch,
                      const ros::Time& t, const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (btTransform(tf::createIdentityQuaternion(),
                                           btVector3(0,0,0)), t, f);
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


pf_vector_t
CurbAmclNode::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

  pf_vector_t p;

  for(;;)
  {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;

    // Check that it's a free cell
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }
  return p;
}

bool
CurbAmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  pf_mutex_.lock();
  ROS_INFO("Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)CurbAmclNode::uniformPoseGenerator,
                (void *)map_);
  pf_init_ = false;
  pf_mutex_.unlock();
  return true;
}

void
CurbAmclNode::curbReceived (const sensor_msgs::PointCloud::ConstPtr& cloud_in)
{
	//ROS_INFO("-----------------------------curbReceived-------------------------------");
	
	//--------------------------------------------------------------------------------//
	ros::Time now1 = ros::Time::now();
	//ROS_INFO("now1 %lf", now1.toSec());
	//--------------------------------------------------------------------------------//
  
	ros::Time curbRece_time = cloud_in->header.stamp;
	//ROS_INFO("curbRece_time %lf", curbRece_time.toSec());
	
	base_link_point_.header.frame_id = "map";
	base_link_point_.header.stamp = curbRece_time;	
	position_covariance_.header.frame_id = "map";
	position_covariance_.header.stamp = curbRece_time;	

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
		ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
		return;
	}

	pf_mutex_.lock();

	pf_vector_t delta = pf_vector_zero();
  
	//After curb window updates the filter, it need to be reinitialized; 
	bool force_publication = false;
	bool resampled = false;

	if(curbdata_->reinit_==true)
	{
		//ROS_INFO("Curbdata reinit");
	
		pf_odom_pose_ = pose;
		force_publication = true;
		curbdata_->reinit_ = false;
		curbdata_->beginningTf_ = baseOdomTemp;
	
		//to set time to be "beginning time" of the new window;
		curbdata_->curbSegment_.header= cloud_in->header;
		curbdata_->curbSegment_.points.clear();
	
		for(unsigned int ip=0; ip<cloud_in->points.size(); ip++)
		{
			curbdata_->curbSegment_.points.push_back(cloud_in->points[ip]);
		}
		curbdata_->accumNum_ = curbdata_->curbSegment_.points.size();
	
	}
	else
	{
		//ROS_INFO("curbdata accumulate");		
		geometry_msgs::Pose temppose;
		temppose.position.x=0;
		temppose.position.y=0;
		temppose.position.z=0;
		temppose.orientation.x=1;
		temppose.orientation.y=0;
		temppose.orientation.z=0;
		temppose.orientation.w=0;
	
		////////////////////////////////////////////////////////////////////////////////////////////////////
		//transfer all the points into the "base_link" coordinate at the beginning time of this curb window;
		//refer to function "initialPoseReceived" for better understanding;
		//////////////////////////////////////////////////////////////////////////////////////////////////// 
	
		//pay attention here when debugging;
		tf::Transform baselink_old_new  = curbdata_->beginningTf_.inverse() * baseOdomTemp;
		tf::Transform baselink_new_old  = baseOdomTemp.inverse() * curbdata_->beginningTf_;
	
		for(unsigned int ip=0; ip<cloud_in->points.size(); ip++)
		{
			temppose.position.x = cloud_in->points[ip].x;
			temppose.position.y = cloud_in->points[ip].y;
			temppose.position.z = cloud_in->points[ip].z;
			
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
		//delta = pf_vector_coord_sub(pose, pf_odom_pose_);
		delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
		delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
		delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

		// served as the normalizer when doing analysis;
		double distTolast = sqrt(delta.v[0]*delta.v[0]+delta.v[1]*delta.v[1]);
		
		// See if we should update the filter
		bool update1 = fabs(delta.v[0]) > d_thresh_ ||
					fabs(delta.v[1]) > d_thresh_ ||
					fabs(delta.v[2]) > a_thresh_;

		bool update2 = (curbdata_->accumNum_) > numTresh_;
		
		bool update3 = distTolast < 4.0;
    
		bool updateAction = update1 && update2;
		bool updateMeas= update1 && update2 && update3;
		
		if(update3)
		{
			odom_->SetModelDiff(alpha1, alpha2, alpha3, alpha4, alpha6); 
		}
		else
		{
			//just help to shift these particles;
			double temp_alpha1=0.0001;
			double temp_alpha2=0.0001;
			double temp_alpha3=0.0001;
			double temp_alpha4=0.0001;
			double temp_alpha6=0.0001;
			
			odom_->SetModelDiff(temp_alpha1, temp_alpha2, temp_alpha3, temp_alpha4, temp_alpha6);
		}
		
		// Prediction Step;
		if(updateAction)
		{
			ROS_INFO("---------------------update------------------");
			ROS_INFO("distTolast: %5f, accumNum: %d", distTolast, curbdata_->accumNum_);
			ROS_INFO("pitch: %5f", pitch);
			
			AMCLOdomData odata;
			odata.pose  = pose;
			odata.delta = delta;
			odata.pitch = pitch;
			odom_->UpdateAction(pf_, (AMCLSensorData*)&odata, (void *)map_);
			
			//the following line is quite important and interesting;
			//seperate "action updating" and "measurement updating" 
			pf_odom_pose_ = pose;
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

				if(pointtemp.y>0){newLeftMeas_.points.push_back(pointtemp);}
				else{newRightMeas_.points.push_back(pointtemp);}
			}
			
			curbdata_->curbSegment_.points=temppointcloud.points;
			curbdata_->sensor = curb_;
			
			///////////////////////////////////////////////////////////////////////////////////////
			//The following block is added for analysis purposes;
			///////////////////////////////////////////////////////////////////////////////////////
			measAnalysis(oldLeftMeas_,  newLeftMeas_,  distTolast, LeftAnalysis_,  LeftAnalysis_pub_ );
			measAnalysis(oldRightMeas_, newRightMeas_, distTolast, RightAnalysis_, RightAnalysis_pub_);
		
			//////////////////////////////////////////////////////////////////////////////////////
			// to keep in accordance with original code;
			// Segmentation Fault once found here!!!
			// because my curbdata_ here is a pointer, I shouldn't use "&" as original code does;
			//////////////////////////////////////////////////////////////////////////////////////
						
			curb_->UpdateSensor(pf_, (AMCLSensorData*) curbdata_);
		    curb_->UpdateSensor(pf_, (AMCLSensorData*) curbdata_);
		    
			if(!(++resample_count_ % resample_interval_))
			{
				pf_update_resample(pf_);
				resampled = true;
				//ROS_INFO("Resampled is true");
			}
			
			pf_sample_set_t* set = pf_->sets + pf_->current_set;
			ROS_DEBUG("Num samples: %d\n", set->sample_count);

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
	
	if(resampled||(force_publication && (!pf_init_)))
	{
		/////////////////////////////////////////////////////////////////////////////////////////////////////
		// Once found a small bug here;
		// "pf_init_" is used to flag the filter init, also quite important; 
		// if this flag is not used, every time curbdata_ gets reinit, the loop function into this branch;
		// the branch use stored hypotheses to generate new tf_;
		// However, except the time of pf_init_ when hypothese is determined by initial poses, 
		// run-time hypotheses are always stale because they are set from last trigger;
		////////////////////////////////////////////////////////////////////////////////////////////////////
 
		pf_init_ = true;
		
		//ROS_INFO("resampled or force_publish");
		// Read out the current hypotheses
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
			//ROS_INFO("max_weight>0");
	  
			ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
					hyps[max_weight_hyp].pf_pose_mean.v[0],
					hyps[max_weight_hyp].pf_pose_mean.v[1],
					hyps[max_weight_hyp].pf_pose_mean.v[2]);

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
	
			ROS_INFO("New pose: %6.3f %6.3f %6.3f",
					hyps[max_weight_hyp].pf_pose_mean.v[0],
					hyps[max_weight_hyp].pf_pose_mean.v[1],
					hyps[max_weight_hyp].pf_pose_mean.v[2]);
					
			ROS_INFO("Covariance: %6.3f %6.3f %6.3f",
					p.pose.covariance[6*0+0],
					p.pose.covariance[6*1+1],
					p.pose.covariance[6*3+3]);
			
			
			base_link_point_.point.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
			base_link_point_.point.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
			base_link_point_.point.z = hyps[max_weight_hyp].pf_pose_mean.v[2];
			
			position_covariance_.point.x = p.pose.covariance[6*0+0];
			position_covariance_.point.y = p.pose.covariance[6*1+1];
			position_covariance_.point.z = p.pose.covariance[6*3+3];

			// subtracting base to odom from map to base and send map to odom instead
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

			latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
									tf::Point(odom_to_map.getOrigin()));
			latest_tf_valid_ = true;

			// We want to send a transform that is good up until a
			// tolerance time so that odom can be used
			ros::Time transform_expiration = (curbRece_time +transform_tolerance_);
			tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),transform_expiration, global_frame_id_, odom_frame_id_);
		
			this->tfb_->sendTransform(tmp_tf_stamped);
      
			//ROS_INFO("newly generated tf Time %lf",transform_expiration.toSec());
		
			sent_first_transform_ = true;
		}
		else
		{
			ROS_ERROR("No pose!");
		}
    
		//--------------------------------------------------------------------------------//
		ros::Time now9 = ros::Time::now();
		//ROS_INFO("now9 %lf", now9.toSec());
		//--------------------------------------------------------------------------------//
		
		baselink_pt_pub_.publish(base_link_point_);
		cov_pub_.publish(position_covariance_);
	}  
	else if(latest_tf_valid_)
	{
		//--------------------------------------------------------------------------------//
		ros::Time now10 = ros::Time::now();
		//ROS_INFO("now10 %lf", now10.toSec());
		//--------------------------------------------------------------------------------//
    
		//ROS_INFO("Be happy: republish last tf");
	
		// Nothing changed, so we'll just republish the last transform, to keep everybody happy.

		ros::Time transform_expiration = (curbRece_time + transform_tolerance_);
		tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),transform_expiration,
											global_frame_id_, odom_frame_id_);
											
		this->tfb_->sendTransform(tmp_tf_stamped);
	 
		//ROS_INFO("last tf republish at %lf",transform_expiration.toSec());

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
  
	//--------------------------------------------------------------------------------//
	ros::Time now11 = ros::Time::now();
	//ROS_INFO("now11 %lf", now11.toSec());
	//--------------------------------------------------------------------------------//
	
	

	pf_mutex_.unlock();
}

double
CurbAmclNode::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  btMatrix3x3 mat = t.getBasis();
  mat.getEulerYPR(yaw,pitch,roll);
  return yaw;
}

void
CurbAmclNode::initialPoseReceivedOld(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  // Support old behavior, where null frame ids were accepted.
  if(msg->header.frame_id == "")
  {
    ROS_WARN("Received initialpose message with header.frame_id == "".  This behavior is deprecated; you should always set the frame_id");
    initialPoseReceived(msg);
  }
}

void
CurbAmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
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


void CurbAmclNode::measAnalysis(sensor_msgs::PointCloud& oldMeas, sensor_msgs::PointCloud& newMeas, const double Disnormalizer, 
								geometry_msgs::PointStamped& AnalysisResult, const ros::Publisher& Analysis_pub)
 {
	try
	{
		tf_->transformPointCloud(global_frame_id_, newMeas, newMeas);
	}
	catch(tf::TransformException e)
	{
		ROS_ERROR("Failed to transform newMeas between baselink and map, (%s)", e.what());
		return;
	}
	
	if(newMeas.points.size()>0)
	{
		if(oldMeas.points.size()>0)
		{
			AnalysisResult.header = newMeas.header;
			
			//analyze the normalized distance;
			//use "AnalysisResult.point.x" to store "the normalized distance";
			unsigned int analyzeNum =0;
			double disSum =0;
			for(unsigned int i=0; i<newMeas.points.size(); i++)
			{
				if(i == oldMeas.points.size()){break;}
				else
				{
					analyzeNum = analyzeNum+1;
					
					float xnewtemp = newMeas.points[i].x;
					float ynewtemp = newMeas.points[i].y;
					float xoldtemp = oldMeas.points[i].x;
					float yoldtemp = oldMeas.points[i].y;
					
					float xdis = xnewtemp-xoldtemp;
					float ydis = ynewtemp-yoldtemp;
					float distemp = sqrtf(xdis*xdis+ydis*ydis);
					
					disSum = disSum+distemp;
				}
			}
			double fullNormalizer = double(analyzeNum)*Disnormalizer;
			AnalysisResult.point.x = double(disSum)/fullNormalizer;
			
			//analyze the angle difference;
			//use "AnalysisResult.point.y" to store it;
			if(analyzeNum == 1){ AnalysisResult.point.y=0;}
			else
			{
				float oldAngle=0;
				float newAngle=0;
				
				float olddeltx = oldMeas.points[analyzeNum-1].x-oldMeas.points[0].x;
				float olddelty = oldMeas.points[analyzeNum-1].y-oldMeas.points[0].y;
				if(olddeltx==0)
				{
					if(olddelty>0){oldAngle=M_PI_2;}
					else{oldAngle=-M_PI_2;}
				}
				else{oldAngle = atan2f(olddelty,olddeltx);}
				
				float newdeltx = newMeas.points[analyzeNum-1].x-newMeas.points[0].x;
				float newdelty = newMeas.points[analyzeNum-1].y-newMeas.points[0].y;
				if(newdeltx==0)
				{
					if(newdelty>0){newAngle=M_PI_2;}
					else{newAngle=-M_PI_2;}
				}
				else{newAngle = atan2f(newdelty,newdeltx);}

				AnalysisResult.point.y = newAngle - oldAngle;
			}
			AnalysisResult.point.z = 0;	
		}
		//at the very beginning;
		else
		{
			oldMeas = newMeas;
			return;
		}
	}
	else
	{
		return;
	}	
	//prepare for next rounds;
	oldMeas = newMeas;
	Analysis_pub.publish(AnalysisResult);	
 }
