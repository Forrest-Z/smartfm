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
using namespace std;

struct VehicleParticle{
  double x;
  double y;
  double r;
  double weight;
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
  
  void initialize_particles(int particle_num, sensor_msgs::PointCloud pt){
    //take the first point for now
    NORM_DIST x_dist(pt.points[0].x, dist_stddev_);
    NORM_DIST y_dist(pt.points[0].y, dist_stddev_);
    NORM_GEN gen_x(eng, x_dist);
    NORM_GEN gen_y(eng, y_dist);
    particles_.resize(particle_num);
    for(int i=0; i<particle_num; i++){
      particles_[i].x = gen_x();
      particles_[i].y = gen_y();
    }
    mean_x_ = pt.points[0].x;
    mean_y_ = pt.points[0].y;
    time_pre_ = pt.header.stamp.toSec();
  }
      
public:
  PosePF(int particle_num, double dist_stddev, 
	 double resampling_ratio, sensor_msgs::PointCloud pts): 
	initialized_(false), dist_stddev_(dist_stddev),
	resampling_ratio_(resampling_ratio){
	unsigned int seed = static_cast<unsigned int>(std::time(0));
	eng.seed(seed);
	initialize_particles(particle_num, pts);
  }
  vector<VehicleParticle> particles_;
  double mean_x_, mean_y_, mean_r_, mean_v_;
  double var_x_, var_y_;
  double time_pre_;
  void updateParticleWeight(sensor_msgs::PointCloud &pc, VehicleParticle &particle){
    double dist=1e99;
    for(size_t i=0; i<pc.points.size(); i++){
      double dist_temp = fmutil::mag(pc.points[i].x-particle.x, pc.points[i].y-particle.y);
      if(dist > dist_temp)
	dist = dist_temp;
    }
    double a_t = 10; //maximum score
    double b_t = 0.5; //gradient
    particle.weight = a_t*exp(-b_t*dist);
    
    
  }
  
  //perform observation, sampling and injection
  void updateObservation(sensor_msgs::PointCloud &pc){
    if(pc.points.size() == 0) {
      mean_v_ = 0;
      return;
    }
    //assigning weight by observed points
    for(size_t i=0; i<particles_.size(); i++){
      updateParticleWeight(pc, particles_[i]);
    }
    
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
    
    //get the mean and deviation of the current filter
    double mean_x = 0, mean_y = 0;
    double mean_sq_x = 0, mean_sq_y = 0;
    for(size_t i=0; i<new_particles.size(); i++){
      mean_x += new_particles[i].x;
      mean_y += new_particles[i].y;
      mean_sq_x += new_particles[i].x*new_particles[i].x;
      mean_sq_y += new_particles[i].y*new_particles[i].y;
    }
    
    mean_x/=new_particles.size();
    mean_y/=new_particles.size();
    var_x_ = mean_sq_x/new_particles.size() - mean_x*mean_x;
    var_y_ = mean_sq_y/new_particles.size() - mean_y*mean_y;
    double diff_y = mean_y_-mean_y;
    double diff_x = mean_x_-mean_x;
    double time_now = pc.header.stamp.toSec();
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
    while(particles_.size()-new_particles.size()){
      VehicleParticle part_temp;
      part_temp.x = gen_x();
      part_temp.y = gen_y();
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
   NORM_DIST v_dist(current_speed_, 0.01);
   NORM_DIST r_dist(mean_r_, 1.0/180.0*M_PI);
   NORM_GEN v_gen(eng, v_dist);
   NORM_GEN r_gen(eng, r_dist);
   for(size_t i=0; i<particles_.size(); i++){
     double v_predicted = v_gen();
     double r_predicted = r_gen();
     particles_[i].x += v_predicted*cos(r_predicted);
     particles_[i].y += v_predicted*sin(r_predicted);
   }
  }
  
};

class LaserPoseTracking{
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
  bool first_call_;
public:
  LaserPoseTracking():first_call_(true) {
    pc_sub_ = nh_.subscribe("pedestrian_poi", 10, &LaserPoseTracking::pcCallback, this);
    particles_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pose_particles", 10);
    mean_particles_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pose_mean", 10);
    ros::spin();
  }
  
private:
  double time_pre_;
  vector<PosePF> filters_;
  int object_id_;
  ros::Publisher particles_pub_, mean_particles_pub_;
  tf::TransformListener tf_;
  double distance_cur_;
  void publishParticles(vector<VehicleParticle> &particles){
    sensor_msgs::PointCloud pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "/map";
    for(size_t i=0; i<particles.size(); i++){
      geometry_msgs::Point32 pt;
      pt.x = particles[i].x;
      pt.y = particles[i].y;
      pt.z = 0;
      pc.points.push_back(pt);
    }
    particles_pub_.publish(pc);
  }
  void pcCallback(sensor_msgs::PointCloud::ConstPtr pc){
    //cout<<pc->points.size()<<" observations received"<<endl;
    //remember to check the time to reset if neg time received
    if(first_call_) {
      time_pre_ = pc->header.stamp.toSec();
      first_call_ = false;
      object_id_++;
      distance_cur_ = 0.;
    }
    else {
      if(pc->header.stamp.toSec() - time_pre_ < 0) {
	filters_.clear();
	first_call_ = true;
	cout<<"Neg time detected, reseting filter"<<endl;
	return;
      }
      time_pre_ = pc->header.stamp.toSec();
    }
    sensor_msgs::PointCloud pc_copy = *pc;
    if(filters_.size() == 0 && pc->points.size() > 0){
      PosePF filter(200, 3.0, 0.8, pc_copy);
      filters_.push_back(filter);
      return;
    }
    if(filters_.size() > 0){
      filters_[0].updateObservation(pc_copy);
      if(filters_[0].var_x_ > 7 || filters_[0].var_y_ > 7){
	cout<<"Variance too big, resetting filter"<<endl;
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
      tf_.transformPointCloud("opposite_traffic", pose, pose);
      if(distance_cur_<pose.points[0].x) distance_cur_ = pose.points[0].x;
      cout<<object_id_<<": "<<distance_cur_<<"\xd"<<flush;
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "LaserPoseTracking");
  LaserPoseTracking lp;
  return 0;
}
