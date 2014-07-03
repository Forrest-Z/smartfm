/*
 * momdp.h
 *
 *  Created on: Mar 4, 2012
 *      Author: golfcar
 */

#ifndef MOMDP_H_
#define MOMDP_H_


#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
//#include <dataAssoc_experimental/PedDataAssoc_vector.h>
//#include <dataAssoc_experimental/PedDataAssoc.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ped_momdp_sarsop/peds_believes.h>
#include <ped_momdp_sarsop/ped_local_frame.h>
#include <ped_momdp_sarsop/ped_local_frame_vector.h>
#include <pnc_msgs/speed_contribute.h>
//#include "executer.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
//#include "pedestrian_changelane.h"
//#include "mcts.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "param.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//#include <pomdp_path_planner/GetPomdpPath.h>
//#include <pomdp_path_planner/PomdpPath.h>
#include <nav_msgs/GetPlan.h>
#include "WorldModel.h"
#include "ped_pomdp.h"
#include "belief.h"
#include "despotstar.h"
#include <ped_pathplan/StartGoal.h>

/*
#include "belief_update/belief_update_exact.h"
#include "belief_update/belief_update_particle.h"
#include "globals.h"
#include "lower_bound_policy_mode.h"
#include "lower_bound_policy_random.h"
#include "lower_bound_policy_suffix.h"
#include "model.h"


#include "solver.h"
#include "util_uniform.h"
#include "upper_bound/upper_bound_nonstochastic.h"
#include "upper_bound/upper_bound_stochastic.h"
#include "world.h"
*/

using namespace std;



class ped_momdp
{
public:
    ped_momdp(ros::NodeHandle& nh);
    
    ~ped_momdp();
    
    void updateSteerAnglePublishSpeed(geometry_msgs::Twist speed);
	void publishSpeed(const ros::TimerEvent &e);
	//void simLoop();
	void publishROSState();
	void publishAction(int);
	void publishBelief();
	void publishMarker(int , PedBelief & ped);
	bool getObjectPose(string target_frame, tf::Stamped<tf::Pose>& in_pose, tf::Stamped<tf::Pose>& out_pose) const;
	
	//for despot
	
	
	//void initSimulator();
	void initSimulator();
	void momdpInit();
	void RetrievePaths(const tf::Stamped<tf::Pose>& carpose);
	void RetrievePathCallBack(const nav_msgs::Path::ConstPtr path); 
	//void updatePedPoses();
	//void updateObsStates();
	//void clean_momdp_problem_sim();

	ros::Publisher window_pub, goal_pub;
	ros::Publisher car_pub;
	ros::Publisher pa_pub;
	ros::Publisher pathPub_;
	ros::Subscriber pathSub_;
	ros::Publisher start_goal_pub;
	//ros::Publisher markers_pubs[ModelParams::N_PED_IN];
	ros::Publisher markers_pub;
	ros::ServiceClient path_client;

    tf::TransformListener tf_;
    WorldStateTracker worldStateTracker;
    WorldModel worldModel;
	WorldBeliefTracker worldBeliefTracker;
	PedPomdp * despot;
	DESPOTSTAR* solver;
	double momdp_speed_,real_speed_;
	string global_frame_id;
	visualization_msgs::MarkerArray markers;

private:
	double control_freq;
	bool goal_reached;
	int safeAction;
    int X_SIZE, Y_SIZE;
    double dX, dY;
    double momdp_problem_timeout;
    bool robot_pose_available;
    double robotx_, roboty_, robotspeedx_;
    bool use_sim_time_, stationary_;
    ros::Timer timer_,timer_speed;
	//SharedPointer<AlphaVectorPolicy> policy;
	//SharedPointer<AlphaVectorPolicy> qmdp_policy;
    ros::Publisher believesPub_, cmdPub_,actionPub_;

    
    void controlLoop(const ros::TimerEvent &e);

    





};
#endif /* MOMDP_H_ */
