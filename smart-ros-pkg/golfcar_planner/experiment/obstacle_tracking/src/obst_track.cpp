/*
 * obst_track.cpp
 *
 *  Created on: Jul 10, 2013
 *      Author: liuwlz
 */

#include "obst_track.h"

bool debug = false;

ObstTrack::ObstTrack():first_call_(true) {
    pc_sub_ = nh_.subscribe("obst_pose_measure", 10, &ObstTrack::pcCallback, this);
    particles_pub_ = nh_.advertise<geometry_msgs::PoseArray>("pose_particles", 10);
    mean_particles_pub_ = nh_.advertise<sensor_msgs::PointCloud>("obst_pose_track", 10);
    object_id_ = 0;
	ros::spin();
	pre_size = 0;
}

ObstTrack::~ObstTrack(){

}

void ObstTrack::pcCallback(sensor_msgs::PointCloud::ConstPtr pc){
	//TODO: Multiple obstacle tracking:1) Particle filter identification to distinguish different obstacle, 2) Segmentation fault fixing

	sensor_msgs::PointCloud veh_pts;
	if(first_call_) {
		time_pre_ = pc->header.stamp.toNSec();
		first_call_ = false;
		object_id_++;
		distance_cur_ = 0.;
	}
	else {
		if(pc->header.stamp.toNSec() - time_pre_ <= 0) {
			filters_.clear();
			first_call_ = true;
			return;
		}
		if (debug)
			cout<<pc->header.stamp.toNSec()<<"time pre "<< time_pre_<<"now"<<ros::Time::now().toSec()<<endl;
		time_pre_ = pc->header.stamp.toNSec();
	}
	sensor_msgs::PointCloud pc_copy = *pc;

	if(filters_.size() == 0 && pc->points.size() > 0){
		if(pc_copy.points.size() == 0)
			return;
			ObstPF filter(200, 1.0, 0.9, pc_copy);
			filters_.push_back(filter);
		return;
	}
	if(filters_.size() > 0){
		cout << "Filter to be tracked" << pc_copy.points.size()<<endl;
		//for (int i = 0; i < filters_.size(); i++){
			/*
			sensor_msgs::PointCloud temp_pts;
			temp_pts.points.push_back(pc_copy.points[i]);
			temp_pts.header = pc_copy.header;
			*/
			filters_[0].updateObservation(pc_copy);
			if(filters_[0].var_x_ > 5 || filters_[0].var_y_ > 5){
				cout<<"fisrt   "<<"Variance too big, resetting filter"<<filters_[0].var_x_<<" "<<filters_[0].var_y_<<endl;
				filters_.clear();
				first_call_ = true;
				return;
			}
			publishParticles(filters_[0].particles_);
			sensor_msgs::PointCloud pose;
			sensor_msgs::ChannelFloat32 _orien;
			sensor_msgs::ChannelFloat32 _vel;
			pose.points.resize(filters_.size());
			pose.points[0].x = filters_[0].mean_x_;
			pose.points[0].y = filters_[0].mean_y_;
			_orien.name = "orientation";
			_vel.name = "velocity";
			_orien.values.push_back(filters_[0].mean_r_);
			_vel.values.push_back(filters_[0].mean_v_);
			pose.channels.push_back(_orien);
			pose.channels.push_back(_vel);
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
			if(distance_cur_<pose.points[0].x) {
				distance_cur_ = pose.points[0].x;
				cout<<object_id_<<": "<<distance_cur_<<"\xd"<<flush;
			}
		//}
	}
	pre_size = filters_.size();
}

void ObstTrack::filterPoints(geometry_msgs::Point32 init_pt, sensor_msgs::PointCloud &pc){
	ROS_DEBUG("Filter points");
	for(size_t i=0; i<pc.points.size(); ){
		if(fmutil::distance<geometry_msgs::Point32>(pc.points[i], init_pt)>15)
			pc.points.erase(pc.points.begin()+i);
		else
			i++;
	}
}

void ObstTrack::publishParticles(vector<VehicleParticle> &particles){
	geometry_msgs::PoseArray pa;
	pa.header.stamp = ros::Time::now();
	pa.header.frame_id = "local_map";
	for(size_t i=0; i<particles.size(); i++){
		geometry_msgs::Pose pt;
		pt.position.x = particles[i].x;
		pt.position.y = particles[i].y;
		pt.position.z = 0;
		tf::Quaternion quad = tf::createQuaternionFromYaw(particles[i].r);
		tf::quaternionTFToMsg(quad, pt.orientation);
		pa.poses.push_back(pt);
	}
	particles_pub_.publish(pa);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "obst_tracking");
	ObstTrack pose_tracking;
	return 0;
}
