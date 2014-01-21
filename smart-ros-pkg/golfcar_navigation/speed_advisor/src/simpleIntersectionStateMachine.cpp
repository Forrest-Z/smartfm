#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <pnc_msgs/poi.h>
#include <std_msgs/Bool.h>
#include <speed_advisor/intSM.h>
#include <nav_msgs/Odometry.h>

enum IntersectionState{
  NORMAL,
  APPROACHING,
  STOPPING,
  CHECKING
};

ros::Publisher *state_pub_, *automoton_pub_;
std_msgs::Bool go_msg_, stop_msg_;
int state_ = NORMAL;
double cur_speed_ = 0.0;
double intersection_movement_ = 1.0;
int intersection_node_idx_;

void speedCallback(nav_msgs::Odometry odo){
  cur_speed_ = odo.twist.twist.linear.x;
}

void environmentCallback(geometry_msgs::PointStamped p){
  intersection_movement_ = p.point.x;
}

void poiCallback(pnc_msgs::poi poi){
    size_t i=0;
    switch (state_){
      case NORMAL:
	for(; i<poi.int_pts.size(); i++){
	  if(poi.int_pts[i] - poi.cur_node > 0 && poi.cur_node>0){
	    state_ = APPROACHING;
	    intersection_node_idx_ = poi.int_pts[i];
	    break;
	  }
	}
	break;
      case APPROACHING:
	if(poi.cur_node > intersection_node_idx_){
	  state_ = STOPPING;
	  automoton_pub_->publish(stop_msg_);
	}
	break;
      case STOPPING:
	if(fabs(cur_speed_) < 0.01)
	  state_ = CHECKING;
	break;
      case CHECKING:
	if(intersection_movement_ < 0.5){
	  state_ = NORMAL;
	  automoton_pub_->publish(go_msg_);
	}
	break;
    }
    speed_advisor::intSM state_msg;
    state_msg.state = state_;
    state_msg.inter_node = intersection_node_idx_;
    state_msg.cur_node = poi.cur_node;
    state_pub_->publish(state_msg);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_intersection_sm");
  ros::NodeHandle nh;
  ros::Publisher automoton_pub = nh.advertise<std_msgs::Bool>("inter_trigger", 1, true);
  ros::Publisher state_pub = nh.advertise<speed_advisor::intSM>("inter_state", 1);
  automoton_pub_ = &automoton_pub;
  state_pub_ = &state_pub;
  ros::Subscriber poi_sub = nh.subscribe("poi", 1, poiCallback);
  ros::Subscriber speed_sub = nh.subscribe("odom", 1, speedCallback);
  ros::Subscriber environment_sub = nh.subscribe("environment_status", 1, environmentCallback);
  go_msg_.data = false;
  stop_msg_.data = true;
  ros::spin();
}