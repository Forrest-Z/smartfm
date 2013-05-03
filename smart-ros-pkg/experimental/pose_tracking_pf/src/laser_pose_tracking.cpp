#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <fmutil/fm_math.h>
#include <tf/tf.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <numeric>
#include <tf/transform_listener.h>
#include <ped_momdp_sarsop/ped_local_frame_vector.h>
#include <pnc_msgs/move_status.h>
#include <rrts/rrts_status.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
using namespace std;

struct VehicleParticle{
  double x;
  double y;
  double r;
  double v;
  double weight;
};

struct ParticlesStat{
  double var_x;
  double var_y;
  double mean_x;
  double mean_y;
};
class PosePF{
  typedef boost::mt19937                     ENG;    // Mersenne Twister
  typedef boost::normal_distribution<double> NORM_DIST;   // Normal Distribution
  typedef boost::variate_generator<ENG&,NORM_DIST> NORM_GEN;    // Variate generator
  
  ENG eng;
  
  bool initialized_;
  // remove observed points from pointcloud when the point is 
  // outside from this range. Can be used to initialize another filter
  // for multiple target tracking
  double dist_stddev_;
  // 0-1 resampling ratio, percentage of particles to be retained during sampling
  double resampling_ratio_;
  
  int roll_weighted_die(vector<double> &probabilities) {
    std::vector<double> cumulative;

    std::partial_sum(probabilities.begin(), probabilities.end(),
		    std::back_inserter(cumulative));
    boost::uniform_real<> dist(0, cumulative.back());
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > die(eng, dist);

    return (std::lower_bound(cumulative.begin(), cumulative.end(), die()) - cumulative.begin());
  }
  double v_noise_, r_noise_;
  void initialize_particles(int particle_num, sensor_msgs::PointCloud pt){
    //take the first point for now
    NORM_DIST x_dist(pt.points[0].x, dist_stddev_);
    NORM_DIST y_dist(pt.points[0].y, dist_stddev_);
    NORM_DIST r_dist(-0.79, r_noise_);
    NORM_DIST v_dist(0.05, v_noise_);
    NORM_GEN gen_x(eng, x_dist);
    NORM_GEN gen_y(eng, y_dist);
    NORM_GEN gen_r(eng, r_dist);
    NORM_GEN gen_v(eng, v_dist);
    particles_.resize(particle_num);
    for(int i=0; i<particle_num; i++){
      particles_[i].x = gen_x();
      particles_[i].y = gen_y();
      particles_[i].r = gen_r();
      particles_[i].v = gen_v();
    }
    mean_x_ = pt.points[0].x;
    mean_y_ = pt.points[0].y;
    time_pre_ = ros::Time::now().toSec();
  }
  ParticlesStat getParticlesStat(vector<VehicleParticle> &new_particles){
    //get the mean and deviation of the current filter
    double mean_x = 0, mean_y = 0;
    double mean_sq_x = 0, mean_sq_y = 0;
    for(size_t i=0; i<new_particles.size(); i++){
      mean_x += new_particles[i].x;
      mean_y += new_particles[i].y;
      mean_sq_x += new_particles[i].x*new_particles[i].x;
      mean_sq_y += new_particles[i].y*new_particles[i].y;
    }
    ParticlesStat ps;
    mean_x/=new_particles.size();
    mean_y/=new_particles.size();
    
    double var_x = mean_sq_x/new_particles.size() - mean_x*mean_x;
    double var_y = mean_sq_y/new_particles.size() - mean_y*mean_y;
    
    ps.mean_x = mean_x;
    ps.mean_y = mean_y;
    ps.var_x = var_x;
    ps.var_y = var_y;
    return ps;
  }
public:
  PosePF(int particle_num, double dist_stddev, 
	 double resampling_ratio, sensor_msgs::PointCloud pts): 
	initialized_(false), dist_stddev_(dist_stddev),
	resampling_ratio_(resampling_ratio){
	unsigned int seed = static_cast<unsigned int>(std::time(0));
	eng.seed(seed);
	initialize_particles(particle_num, pts);
	v_noise_ = 0.02;
	r_noise_ = 1./180*M_PI;
  }
  vector<VehicleParticle> particles_;
  double mean_x_, mean_y_, mean_r_, mean_v_;
  double var_x_, var_y_;
  double time_pre_;
  bool updateParticleWeight(sensor_msgs::PointCloud &pc, VehicleParticle &particle){
    double dist=1e99;
    for(size_t i=0; i<pc.points.size(); i++){
      double dist_temp = fmutil::mag(pc.points[i].x-particle.x, pc.points[i].y-particle.y);
      if(dist > dist_temp)
	dist = dist_temp;
    }
    double a_t = 10; //maximum score
    double b_t = 0.5; //gradient
    
    //not to be influenced by observation far far away
    //return false if not observed
    if(dist > 10) {
      particle.weight = 0;
      return false;
    }
    else
      particle.weight = a_t*exp(-b_t*dist);
    return true;
    
  }
  
  //perform observation, sampling and injection
  void updateObservation(sensor_msgs::PointCloud &pc){
    if(pc.points.size() == 0) return;
    //assigning weight by observed points
    bool at_least_one_updated_weight = false;
    for(size_t i=0; i<particles_.size(); i++){
      bool updated = updateParticleWeight(pc, particles_[i]);
      if(!at_least_one_updated_weight && updated)
	at_least_one_updated_weight = true;
    }
    //resolve the particles running away issue
    if(!at_least_one_updated_weight) return;
    //collect weighted particle and clean up observed points
    vector<double> weighted_particles;
    for(size_t i=0; i<particles_.size(); i++){
      for(size_t j=0; j<pc.points.size();){
	double dist_temp = fmutil::mag(pc.points[j].x-particles_[i].x, 
				       pc.points[j].y-particles_[i].y);
	if(dist_temp<dist_stddev_){
	  pc.points.erase(pc.points.begin()+j);
	}
	else j++;
      }
      weighted_particles.push_back(particles_[i].weight);
    }
    
    //sample particles according to weight
    vector<VehicleParticle> new_particles;
    for(size_t i=0; i<particles_.size()*resampling_ratio_; i++){
      int new_particle_idx = roll_weighted_die(weighted_particles);
      new_particles.push_back(particles_[new_particle_idx]);
    }
    
    ParticlesStat ps = getParticlesStat(new_particles);
    double mean_x = ps.mean_x;
    double mean_y = ps.mean_y;
    var_x_ = ps.var_x;
    var_y_ = ps.var_y;
    double diff_y = mean_y_-mean_y;
    double diff_x = mean_x_-mean_x;
    double time_now = ros::Time::now().toSec();//pc.header.stamp.toSec();
    //not using the time for now, assuming same period of time is used
    //when propagate motion
    mean_v_ = sqrt(diff_y*diff_y+diff_x*diff_x);//(time_now - time_pre_);
    time_pre_ = time_now;
    
    //estimate rotation from mean position
    mean_r_ = -0.79;//atan2(mean_y-mean_y_, mean_x-mean_x_);
    mean_x_ = mean_x;
    mean_y_ = mean_y;
    //cout<<"x,y,r,v = "<<mean_x_<<","<<mean_y_<<","<<mean_r_<<","<<mean_v_
    //<<","<<var_x_<<","<<var_y_<<","<<pc.points.size()<<endl;
    //insert random particles using mean
    NORM_DIST x_dist(mean_x, dist_stddev_);
    NORM_DIST y_dist(mean_y, dist_stddev_);
    NORM_GEN gen_x(eng,x_dist);
    NORM_GEN gen_y(eng, y_dist);
    NORM_DIST r_dist(-0.79, r_noise_);
    NORM_DIST v_dist(0.05, v_noise_);
    NORM_GEN gen_r(eng, r_dist);
    NORM_GEN gen_v(eng, v_dist);
    while(particles_.size()-new_particles.size()){
      VehicleParticle part_temp;
      part_temp.x = gen_x();
      part_temp.y = gen_y();
      part_temp.r = gen_r();
      part_temp.v = gen_v();
      new_particles.push_back(part_temp);
    }
    particles_.clear();
    particles_ = new_particles;
  }
  double current_speed_;
  void updateMotion(){
   //a naive constant acceleration model
   if(mean_v_ > current_speed_)
     current_speed_ += 0.05;
   else
     current_speed_ -= 0.05;
   if(current_speed_ < 0) current_speed_ = 0;
   NORM_DIST v_dist(0, v_noise_);
   NORM_DIST r_dist(0, r_noise_);
   NORM_GEN v_gen(eng, v_dist);
   NORM_GEN r_gen(eng, r_dist);
   for(size_t i=0; i<particles_.size(); i++){
     particles_[i].v += v_gen();
     particles_[i].r += r_gen();
     particles_[i].x += particles_[i].v*cos(particles_[i].r);
     particles_[i].y += particles_[i].v*sin(particles_[i].r);
   }
   
   ParticlesStat ps = getParticlesStat(particles_);
   var_x_ = ps.var_x;
   var_y_ = ps.var_y;
   //cout<<"x,y,r,v = "<<mean_x_<<","<<mean_y_<<","<<mean_r_<<","<<mean_v_
      //<<","<<var_x_<<","<<var_y_<<endl;
  }
  
};

class LaserPoseTracking{
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_, move_status_sub_, speed_cmd_sub_;
  bool first_call_;
public:
  LaserPoseTracking():first_call_(true) {
    pc_sub_ = nh_.subscribe("pedestrian_poi", 10, &LaserPoseTracking::pcCallback, this);
    move_status_sub_ = nh_.subscribe("move_status", 10, &LaserPoseTracking::moveStatusCallback, this);
    speed_cmd_sub_ = nh_.subscribe("momdp_vel", 10, &LaserPoseTracking::speedCmdCallback, this);
    particles_pub_ = nh_.advertise<geometry_msgs::PoseArray>("pose_particles", 10);
    mean_particles_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pose_mean", 10);
    pompdp_pub_ = nh_.advertise<ped_momdp_sarsop::ped_local_frame_vector>("ped_local_frame_vector", 10);
    stopping_cmd_pub_ = nh_.advertise<rrts::rrts_status>("rrts_status", 10);
    object_id_ = 0;
    ros::spin();
  }
  
private:
  double time_pre_;
  vector<PosePF> filters_;
  int object_id_;
  ros::Publisher particles_pub_, mean_particles_pub_, pompdp_pub_, stopping_cmd_pub_;
  tf::TransformListener tf_;
  double distance_cur_;
  double robot_distance_;
  void moveStatusCallback(pnc_msgs::move_statusConstPtr move_status){
    robot_distance_ = move_status->dist_to_goal;
  }
  
  void speedCmdCallback(geometry_msgs::TwistConstPtr speed_cmd){
    //larger number is expected as this is the distance to goal
    bool stop = false;
    if(robot_distance_ > 58 ){
      if(speed_cmd->linear.x < 0.1) stop = true;
    }
    rrts::rrts_status stat;
    stat.robot_in_collision = stop;
    stopping_cmd_pub_.publish(stat);
  }
  void publishParticles(vector<VehicleParticle> &particles){
    geometry_msgs::PoseArray pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "/map";
    for(size_t i=0; i<particles.size(); i++){
      geometry_msgs::Pose pt;
      pt.position.x = particles[i].x;
      pt.position.y = particles[i].y;
      pt.position.z = 0;
      tf::Quaternion quad = tf::createQuaternionFromYaw(particles[i].r);
      tf::quaternionTFToMsg(quad, pt.orientation);
      pc.poses.push_back(pt);
    }
    particles_pub_.publish(pc);
  }
  
  void filterPoints(geometry_msgs::Point32 init_pt, sensor_msgs::PointCloud &pc){
    for(size_t i=0; i<pc.points.size(); ){
      if(fmutil::distance<geometry_msgs::Point32>(pc.points[i],
	init_pt)>10) pc.points.erase(pc.points.begin()+i);
      else i++;
    }
  }
  void pcCallback(sensor_msgs::PointCloud::ConstPtr pc){
    //cout<<pc->points.size()<<" observations received"<<endl;
    //remember to check the time to reset if neg time received
    geometry_msgs::Point32 init_pt;
    init_pt.x = 161;
    init_pt.y = 185.3;
    if(first_call_) {
      time_pre_ = ros::Time::now().toSec();//pc->header.stamp.toSec();
      first_call_ = false;
      object_id_++;
      distance_cur_ = 0.;
    }
    else {
      if(ros::Time::now().toSec() - time_pre_ < 0) {
	filters_.clear();
	first_call_ = true;
	cout<<"Neg time detected, reseting filter"<<endl;
	return;
      }
      time_pre_ = ros::Time::now().toSec();
    }
    sensor_msgs::PointCloud pc_copy = *pc;
    if(filters_.size() == 0 && pc->points.size() > 0){
      //only initialize those points that are within the
      //starting area
      filterPoints(init_pt, pc_copy);
      if(pc_copy.points.size() == 0) return;
      PosePF filter(200, 3.0, 0.9, pc_copy);
      filters_.push_back(filter);
      return;
    }
    if(filters_.size() > 0){
      filters_[0].updateObservation(pc_copy);
      if(filters_[0].var_x_ > 5 || filters_[0].var_y_ > 5){
	cout<<endl<<"Variance too big, resetting filter"<<filters_[0].var_x_<<" "<<filters_[0].var_y_<<endl;
	filters_.clear();
	first_call_ = true;
	return;
      }
      publishParticles(filters_[0].particles_);
      sensor_msgs::PointCloud pose;
      pose.points.resize(1);
      pose.points[0].x = filters_[0].mean_x_;
      pose.points[0].y = filters_[0].mean_y_;
      //tf::Quaternion quad = tf::createQuaternionFromYaw(filters_[0].mean_r_);
      //tf::quaternionTFToMsg(quad, pose.orientation);
      pose.header = pc_copy.header;
      
      mean_particles_pub_.publish(pose);
      filters_[0].updateMotion();
      //also consider variance here
      if(pc->points.size() == 0){
	if(filters_[0].var_x_ > 5 || filters_[0].var_y_ > 5){
	  cout<<"Variance too big, resetting filter"<<filters_[0].var_x_<<" "<<filters_[0].var_y_<<endl;
	  filters_.clear();
	  first_call_ = true;
	  return;
	}
      }
      try{
	tf_.transformPointCloud("opposite_traffic", pose, pose);
      }
      catch (tf::ExtrapolationException ex){return;}
      if(distance_cur_<pose.points[0].x) {
	distance_cur_ = pose.points[0].x;
	cout<<object_id_<<": "<<distance_cur_<<"\xd"<<flush;
	ped_momdp_sarsop::ped_local_frame single_stat;
	single_stat.ped_id = object_id_;
	if(robot_distance_ > 85)
	  single_stat.rob_pose.z = 0;
	else
	  single_stat.rob_pose.z = 85 - robot_distance_;
	if(single_stat.rob_pose.z > 19) single_stat.rob_pose.z = 19;
	single_stat.ped_pose.z = distance_cur_;
	single_stat.header.stamp = ros::Time::now();
	ped_momdp_sarsop::ped_local_frame_vector pomdp_stat;
	pomdp_stat.ped_local.push_back(single_stat);
	pompdp_pub_.publish(pomdp_stat);
      }
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "LaserPoseTracking");
  LaserPoseTracking lp;
  return 0;
}
