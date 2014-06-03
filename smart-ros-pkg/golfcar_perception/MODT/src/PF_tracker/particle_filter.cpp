/*
 * particle_filter.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: liuwlz
 */

#include "particle_filter.h"

ObstPF::ObstPF(int particle_num, double dist_stddev, double resampling_ratio, sensor_msgs::PointCloud init_pose):
	initialized_(false),
	dist_stddev_(dist_stddev),
	resampling_ratio_(resampling_ratio){
	unsigned int seed = static_cast<unsigned int>(std::time(0));
	eng.seed(seed);
	v_noise_ = 0.02;
	r_noise_ = M_PI/90;
	initialize_particles(particle_num, init_pose);
}

ObstPF::~ObstPF(){

}

void ObstPF::initialize_particles(int particle_num, sensor_msgs::PointCloud init_pose){

	//TODO:Enable orientation distribution estimation
	//TODO:Enable velocity track
	//TODO:Enable multiple objest tracking
    //Norm distribution to obstacle's pose and vel
	NORM_DIST x_dist(init_pose.points[0].x, dist_stddev_);
	NORM_DIST y_dist(init_pose.points[0].y, dist_stddev_);
	NORM_DIST r_dist(init_pose.channels[2].values[0], r_noise_);
	NORM_DIST v_dist(0.0, v_noise_);
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
    mean_x_ = init_pose.points[0].x;
    mean_y_ = init_pose.points[0].y;
    mean_r_ = init_pose.channels[2].values[0];
    mean_v_ = 0;

    time_pre_ = ros::Time::now().toSec();
}

//TODO: Improve motion model
void ObstPF::updateMotion(){
   //a naive constant acceleration model, to be upgrade to more general one
   if(mean_v_ > current_speed_)
	   current_speed_ += 0.1;
   else
	   current_speed_ -= 0.1;
   if(current_speed_ < 0)
	   current_speed_ = 0;
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
}

//perform observation, sampling and injection
void ObstPF::updateObservation(sensor_msgs::PointCloud &pc){
	cout << "update observation"<<endl;
    if(pc.points.size() == 0)
    	return;
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
    		double dist_temp = fmutil::mag(pc.points[j].x-particles_[i].x, pc.points[j].y-particles_[i].y);
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

    mean_v_ = sqrt(diff_y*diff_y+diff_x*diff_x);//(time_now - time_pre_);
    cout << "diff_x" << diff_x << "diff_y" << diff_y <<endl;
    cout << "time_now" << time_now << "pre_time "<<time_pre_<<"velocity"<<mean_v_<<endl;
    time_pre_ = time_now;
    cout << mean_v_<<endl;
    //estimate rotation from mean position
    if (mean_v_ > 0.2)
    	mean_r_ = atan2(mean_y-mean_y_, mean_x-mean_x_);
    else
    	mean_r_ = M_PI/2;
    mean_x_ = mean_x;
    mean_y_ = mean_y;
    //cout<<"x,y,r,v = "<<mean_x_<<","<<mean_y_<<","<<mean_r_<<","<<mean_v_
    //<<","<<var_x_<<","<<var_y_<<","<<pc.points.size()<<endl;
    //insert random particles using mean
    NORM_DIST x_dist(mean_x, dist_stddev_);
    NORM_DIST y_dist(mean_y, dist_stddev_);
    NORM_DIST r_dist(mean_r_, r_noise_);
    NORM_DIST v_dist(mean_v_, v_noise_);

    NORM_GEN gen_x(eng,x_dist);
    NORM_GEN gen_y(eng, y_dist);
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

//Update particles' weight based on distance
bool ObstPF::updateParticleWeight(sensor_msgs::PointCloud &pc, VehicleParticle &particle){
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
    	//TODO:Check the performance of this weight function
    	particle.weight = a_t*exp(-b_t*dist);
    return true;
}

ParticlesStat ObstPF::getParticlesStat(vector<VehicleParticle> &new_particles){
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

int ObstPF::roll_weighted_die(vector<double> &probabilities) {
    std::vector<double> cumulative;

    std::partial_sum(probabilities.begin(), probabilities.end(), std::back_inserter(cumulative));
	boost::uniform_real<> dist(0, cumulative.back());
	boost::variate_generator<boost::mt19937&, boost::uniform_real<> > die(eng, dist);

	return (std::lower_bound(cumulative.begin(), cumulative.end(), die()) - cumulative.begin());
}
