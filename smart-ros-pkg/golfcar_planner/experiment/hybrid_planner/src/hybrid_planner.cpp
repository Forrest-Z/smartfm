/*
 * hybrid_planner.cpp
 *
 *  Created on: Apr 1, 2013
 *      Author: liuwlz
 *
 *Using and modifying the golfcar_traj_gen package code for planner switching.
 *
 */
#include "hybrid_planner.h"

HybridPlanner::HybridPlanner(const int start, const int end)
{
	ROS_INFO("Initialize Hybrid planner");
    g_plan_pub_ = n.advertise<nav_msgs::Path>("pnc_globalplan", 1, true);
    nextpose_pub_ = n.advertise<geometry_msgs::PoseStamped>("pnc_nextpose",1);
    norminal_lane_pub_ = n.advertise<std_msgs::Bool>("norminal_lane", 1, true);
    hybrid_path_pub_ = n.advertise<nav_msgs::Path>("hybrid_plan", 1);
    move_status_pub_ = n.advertise<pnc_msgs::move_status>("move_status_hybrid",1);

    goal_in_collision_sub_ = n.subscribe("rrts_status",1, &HybridPlanner::rrts_status, this);
    move_status_sub_ = n.subscribe("move_status_repub",1, &HybridPlanner::movestatusCallBack, this);
    rrts_path_sub_ = n.subscribe("pncview_trajectory", 1, &HybridPlanner::rrtspathCallBack, this);

    ros::Rate loop_rate(3);
    initDest(start, end);
    initialized_ = false;
    goal_collision_ = false;
    is_first_goal = true;
    is_first_replan = true;
    is_replan_end = false;

    while(!getRobotGlobalPose())
    {
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("Waiting for Robot pose");
    }
    //search for the path nearest to the car current pose
    initialized_ = true;

    rrts_is_replaning_ = false;

    goal_collision_ = false;
    goal_infeasible_ = false;
    root_in_goal_ = false;
    robot_near_root_ = false;
    switched_root_ = false;
    trajectory_found_ = false;

    ros::spin();
}

void HybridPlanner::getNearestWaypoints(){
	ROS_INFO("Check Nearest Waypoint");
    double min_dist = std::numeric_limits<double>::max();
    for( unsigned i=0; i<path_.size(); i++ ){
        double dist = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), path_[i].x_, path_[i].y_);
        if(dist<min_dist){
            waypointNo_ = i;
            min_dist = dist;
        }
    }
    //Increment it to make sure the waypoint is in front
    double dist=0.0;
    do{
        waypointNo_++;
        dist = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), path_[waypointNo_].x_, path_[waypointNo_].y_);
    }while(dist<10);
    ROS_INFO("Initialized waypoint %d", waypointNo_);
    goToDest();
}

void HybridPlanner::rrts_status(rrts_exp::rrts_status rrts_status)
{
    if(initialized_)
    {
        goal_collision_ = rrts_status.goal_in_collision;
        goal_infeasible_ = rrts_status.goal_infeasible;
        root_in_goal_ = rrts_status.root_in_goal;
        robot_near_root_ = rrts_status.robot_near_root;
        switched_root_ = rrts_status.switched_root;
        trajectory_found_ = rrts_status.trajectory_found;
    }
}

void HybridPlanner::movestatusCallBack(pnc_msgs::move_status move_status){
	move_status_ = move_status;
	plannerReasonning();
}

void HybridPlanner::rrtspathCallBack(const nav_msgs::Path rrts_path){
	if(rrts_path.poses.size() != 0){
		ROS_INFO("Received rrts path");
		local_path = rrts_path;
	}
}

void HybridPlanner::initDest(const int start, const int end)
{
	ROS_INFO("Initialize svg path");
    //Default to true which is towards McD
    bool norminal_lane = true;
    //All paths starting from 0 and 1 are moving away from McD
    if(start == 0 || start == 1) norminal_lane = false;
    //Except when moving from 0 to 1 (DCC to McD)
    if(start == 0 && end == 1) norminal_lane = true;
    //All paths starting from 2 and 3 are moving towards McD
    //Except when moving from 2 to 3 (EA to E3A)
    if(start == 2 && end == 3) norminal_lane = false;
    destination_ = sp_.knownStations()(end);
    path_ = sp_.getPath(sp_.knownStations()(start),sp_.knownStations()(end));
    waypointNo_ = 0;
    getGlobalPlan();
    std_msgs::Bool temp; temp.data = norminal_lane;
    norminal_lane_pub_.publish(temp);
}

void HybridPlanner::getGlobalPlan(){

	ROS_INFO("Publish global path for view");
    global_path.poses.resize(path_.size());
    for( unsigned i=0; i<path_.size(); i++ )
    {
    	global_path.poses[i].pose.position.x = path_[i].x_;
    	global_path.poses[i].pose.position.y = path_[i].y_;
    	global_path.poses[i].pose.orientation.w = 1.0;
    	global_path.poses[i].header.frame_id = "/map";
    	global_path.poses[i].header.stamp = ros::Time::now();
    }
    global_path.header.frame_id = "/map";
    global_path.header.stamp = ros::Time::now();
}

bool HybridPlanner::goToDest()
{
	ROS_INFO("Publish waypoint for rrts");
    getRobotGlobalPose();

    ROS_INFO_THROTTLE(3, "Going to %s. Distance=%.0f.", destination_.c_str(), distanceToGoal());
    cout<<"goToDest() waypointNo="<<waypointNo_<<endl;
    map_pose.pose.position.x = path_[waypointNo_].x_;
    map_pose.pose.position.y = path_[waypointNo_].y_;
    double map_yaw = 0;
    if( waypointNo_ < path_.size()-1 ) {
        map_yaw = atan2(path_[waypointNo_+1].y_ - path_[waypointNo_].y_, path_[waypointNo_+1].x_ - path_[waypointNo_].x_);
    }
    else {
        assert(waypointNo_!=0);
        map_yaw = atan2(path_[waypointNo_].y_ - path_[waypointNo_-1].y_, path_[waypointNo_].x_ - path_[waypointNo_-1].x_);
    }

    map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(map_yaw);


    if(goal_collision_ || goal_infeasible_)
    {
        double dist = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), path_[waypointNo_].x_, path_[waypointNo_].y_);
        if(dist < 15)
        {
            if(waypointNo_ < path_.size())
            {
                waypointNo_++;
                cout<<"Goal in collision/infeasible reported, increment waypoint"<<endl;
            }
        }
    }
    map_pose.header.frame_id="/map";
    map_pose.header.stamp=ros::Time::now();
    nextpose_pub_.publish(map_pose);

    //transform from pose to point, planner expect point z as yaw
    //publish the first waypoint in map frame then continue to send the points until the last one
    return true;
}

double HybridPlanner::distanceToGoal()
{
    double d = 0;

    for( unsigned i = waypointNo_+1; i<path_.size(); i++ )
        d += PathPoint::distance(path_[i-1], path_[i]);

    PathPoint p(global_pose_.getOrigin().x(), global_pose_.getOrigin().y());
    d += PathPoint::distance(p, path_[waypointNo_]);

    return d;
}

bool HybridPlanner::getRobotGlobalPose()
{
    global_pose_.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "/base_link";
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    try {
        tf_.transformPose("/map", robot_pose, global_pose_);
    }
    catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        return false;
    }
    return true;
}

//TODO:Improve the reasonning.
void HybridPlanner::plannerReasonning(){
	hybrid_path.header.stamp = ros::Time();
	ROS_INFO("Reasonning about robot state");
    while(!getRobotGlobalPose()){
        ros::spinOnce();
        ROS_INFO("Waiting for Robot pose");
    }
	if(move_status_.emergency || rrts_is_replaning_){
		ROS_INFO("RRTS is planning");
		rrts_is_replaning_ = true;
		rrtsReplanning();
		double check_dist = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), map_pose.pose.position.x, map_pose.pose.position.y);
		ROS_INFO("Distance to sub_goal: %f" , check_dist);
		if (check_dist > 2.0){
			if (trajectory_found_){
				ROS_INFO("Robot heading to the sub_goal");
				hybrid_path_pub_.publish(local_path);
				move_status_.emergency = false;
				move_status_.path_exist = true;
				move_status_pub_.publish(move_status_);
			}
			else{
				ROS_INFO("Robot waiting for the path to sub_goal");
				move_status_.emergency = true;
				move_status_.path_exist = false;
				move_status_pub_.publish(move_status_);
			}
		}
		else{
			ROS_INFO("Robot reached the temporal sub_goal, shift to normal planning");
			if (global_path.poses.size() != 0){
				hybrid_path_pub_.publish(global_path);
			}
			move_status_pub_.publish(move_status_);
			rrts_is_replaning_ = false;
			is_first_goal = true;
		}
	}
	if (!rrts_is_replaning_){
		ROS_INFO("Normal planning");
		if (global_path.poses.size() != 0){
			hybrid_path_pub_.publish(global_path);
		}
		//rrts_callback.disable();
		planner->planner_timer.stop();
		move_status_pub_.publish(move_status_);
	}
}

void HybridPlanner::rrtsReplanning(){
	if (is_first_replan){
		planner = new PlannerExp;
	    //planner->nh.setCallbackQueue(&rrts_callback);
		is_first_replan = false;
	}
	if (is_first_goal){
		getNearestWaypoints();
		is_first_goal =false;
		planner->planner_timer.start();
	}
	//rrts_callback.callAvailable(ros::WallDuration());
}

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "hybrid_planner");
    if(argc<3)
        std::cout<<"Usage: route_planner start end"<<std::endl;
    else
        HybridPlanner rp(atoi(argcv[1]), atoi(argcv[2]));
    return 0;

}
