/*
 * demo_rrts.cpp
 *
 *  Created on: Oct 14, 2013
 *      Author: liuwlz
 */

#include "demo_rrts.h"

RePlanner::RePlanner(){
	ROS_INFO("Initialise re-planner");

    sub_goal_pub_ = n.advertise<geometry_msgs::PoseStamped>("pnc_nextpose",1);
    move_status_pub_ = n.advertise<pnc_msgs::move_status>("move_status_hybrid",1);
    hybrid_path_pub_ = n.advertise<nav_msgs::Path>("global_plan_repub", 1);
    replan_poly_pub_ = n.advertise<geometry_msgs::PolygonStamped>("replan_poly",1);

    global_plan_sub_ = n.subscribe("global_plan", 1,&RePlanner::globalPathCallBack, this);
    rrts_path_sub_ = n.subscribe("pnc_trajectory", 1, &RePlanner::rrtsPathCallBack, this);

    rrts_status_sub_ = n.subscribe("rrts_status",1, &RePlanner::rrtsstatusCallBack, this);
    move_status_sub_ = n.subscribe("move_status_repub",1, &RePlanner::movestatusCallBack, this);

    obst_pts_sub_ = n.subscribe("obst_interest",1,&RePlanner::obstCallBack,this);

    ros::Rate loop_rate(3);
    initialized_ = false;
    is_first_goal = true;
    is_first_replan = true;
    global_path_found_ = false;

    while(!getRobotGlobalPose()){
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("Waiting for Robot pose");
    }
    initialized_ = true;

    planner = new PlannerExp;
    planner->planner_timer.stop();

    empty_path.poses.resize(0);

    rrts_is_replaning_ = false;
    goal_collision_ = false;
    goal_infeasible_ = false;
    root_in_goal_ = false;
    robot_near_root_ = false;
    trajectory_found_ = false;
    ros::spin();
}

RePlanner::~RePlanner(){

}

bool RePlanner::getRobotGlobalPose(){
    global_pose_.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "base_link";
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

bool RePlanner::replanZone(){
	geometry_msgs::Point32 temp_pts_1, temp_pts_2, temp_pts_3, temp_pts_4;
	geometry_msgs::PolygonStamped replan_region_poly;
		//For first obst
	temp_pts_1.x = 100.0; temp_pts_1.y = 106.2; temp_pts_1.z = 0;
	temp_pts_2.x = 100.0; temp_pts_2.y = 128.2; temp_pts_2.z = 0;
	temp_pts_3.x = 115.0; temp_pts_3.y = 128.2; temp_pts_3.z = 0;
	temp_pts_4.x = 115.0; temp_pts_4.y = 106.2; temp_pts_4.z = 0;

	replan_region_poly.header.stamp = ros::Time::now();
	replan_region_poly.header.frame_id = "map";
	replan_region_poly.polygon.points.push_back(temp_pts_1);
	replan_region_poly.polygon.points.push_back(temp_pts_2);
	replan_region_poly.polygon.points.push_back(temp_pts_3);
	replan_region_poly.polygon.points.push_back(temp_pts_4);
	replan_poly_pub_.publish(replan_region_poly);
	if (global_pose_.getOrigin().x()<115 && global_pose_.getOrigin().x()>100){
		if (global_pose_.getOrigin().y()>106.2 && global_pose_.getOrigin().y()<128.2){
			return true;
		}
	}
	return false;
}

void RePlanner::getNearestWaypoints(){
	ROS_INFO("Check Nearest Waypoint");
    double min_dist = DBL_MAX;
    for( unsigned i=0; i<global_path.poses.size(); i++ ){
        double dist = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), global_path.poses[i].pose.position.x, global_path.poses[i].pose.position.y);
        if(dist<min_dist){
            waypointNo_ = i;
            min_dist = dist;
        }
    }
    double dist=0.0;
    do{
        waypointNo_++;
        dist = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), global_path.poses[waypointNo_].pose.position.x, global_path.poses[waypointNo_].pose.position.y);
    }while(dist<10.5);

    ROS_INFO("Initialized waypoint %d", waypointNo_);
    goToDest();
}

void RePlanner::obstCallBack(const sensor_msgs::PointCloud obst_pts){
	obst_interest = obst_pts;
}

void RePlanner::movestatusCallBack(pnc_msgs::move_status move_status){
	ROS_INFO("Got move status");
	move_status_ = move_status;
	plannerReasonning();
}

bool RePlanner::goToDest(){
	ROS_INFO("Publish waypoint for rrts");
    getRobotGlobalPose();

    if(goal_collision_ || goal_infeasible_){
        double dist = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), global_path.poses[waypointNo_].pose.position.x, global_path.poses[waypointNo_].pose.position.y);
        if(dist < 12){
            if(waypointNo_ < global_path.poses.size()){
                waypointNo_ ++;
                cout<<"Goal in collision/infeasible reported, increment waypoint"<<endl;
            }
        }
    }

    sub_goal.pose.position.x = global_path.poses[waypointNo_].pose.position.x;
    sub_goal.pose.position.y = global_path.poses[waypointNo_].pose.position.y;
    double map_yaw = 0;
    if( waypointNo_ < global_path.poses.size()-1 ) {
        map_yaw = atan2(global_path.poses[waypointNo_+1].pose.position.y - global_path.poses[waypointNo_].pose.position.y, global_path.poses[waypointNo_+1].pose.position.x - global_path.poses[waypointNo_].pose.position.x);
    }
    else {
        assert(waypointNo_!=0);
        map_yaw = atan2(global_path.poses[waypointNo_].pose.position.y - global_path.poses[waypointNo_-1].pose.position.y, global_path.poses[waypointNo_].pose.position.x - global_path.poses[waypointNo_-1].pose.position.x);
    }

    sub_goal.pose.orientation = tf::createQuaternionMsgFromYaw(map_yaw);

    sub_goal.header.frame_id="/map";
    sub_goal.header.stamp=ros::Time::now();
    sub_goal_pub_.publish(sub_goal);
    return true;
}

void RePlanner::rrtsstatusCallBack(rrts_exp::rrts_status rrts_status){
    if(initialized_){
        goal_collision_ = rrts_status.goal_in_collision;
        goal_infeasible_ = rrts_status.goal_infeasible;
        root_in_goal_ = rrts_status.root_in_goal;
        robot_near_root_ = rrts_status.robot_near_root;
        trajectory_found_ = rrts_status.trajectory_found;
    }
}

void RePlanner::globalPathCallBack(nav_msgs::Path global_plan){
	if (global_plan.poses.size() != 0 && !global_path_found_){
		ROS_INFO("Received global path");
		global_path = global_plan;
		hybrid_path = global_path;
		global_path_found_ = true;
	}
}

void RePlanner::rrtsPathCallBack(const nav_msgs::Path rrts_path){
	if(rrts_path.poses.size() != 0){
		ROS_INFO("Received rrts path");
		local_path = rrts_path;
	}
}

#if 1
void RePlanner::plannerReasonning(){
	hybrid_path.header.stamp = ros::Time();
	ROS_INFO("Reasonning about robot state");
    while(!getRobotGlobalPose()){
        ros::spinOnce();
        ROS_INFO("Waiting for Robot pose");
    }

    empty_path.header.frame_id = global_path.header.frame_id;
    empty_path.header.stamp = ros::Time::now();

    double dist_to_dest = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), global_path.poses[global_path.poses.size()-1].pose.position.x, global_path.poses[global_path.poses.size()-1].pose.position.y);
    if (dist_to_dest < 1.0){
    	move_status_.path_exist = false;
    	move_status_.emergency = true;
    	move_status_pub_.publish(move_status_);
    	exit(0);
    }
	if((move_status_.emergency || rrts_is_replaning_) && replanZone()){
		ROS_INFO("RRTS is planning");
		rrts_is_replaning_ = true;
		rrtsReplanning();
		if (goal_collision_){
			getNearestWaypoints();
		}
		/*
		if (goal_infeasible_){
			ROS_INFO("Infeasible goal, shift back to normal plan");
			hybrid_path_pub_.publish(empty_path);
			move_status_pub_.publish(move_status_);
			rrts_is_replaning_ = false;
			is_first_goal = true;
			planner->planner_timer.stop();
			return;
		}
		*/
		double check_dist = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), sub_goal.pose.position.x, sub_goal.pose.position.y);
		ROS_INFO("Distance to sub_goal: %f" , check_dist);
		if (check_dist > 2.5){
			if (trajectory_found_){
				ROS_INFO("Robot heading to the sub_goal");
				/*
				int temp_no = waypointNo_;
				while (temp_no < global_path.poses.size()-5){
					temp_no++;
					local_path.poses.push_back(global_path.poses[temp_no]);
				}
				hybrid_path_pub_.publish(local_path);
				*/
				combineHybridPlan();
				hybrid_path_pub_.publish(hybrid_path);
				move_status_.emergency = false;
				move_status_.path_exist = true;
				move_status_pub_.publish(move_status_);
			}
			else{
				ROS_INFO("Robot waiting for the path to sub_goal");
				move_status_.emergency = true;
				move_status_.path_exist = false;
				//Extend the sub_goal if the current sub_goal is infeasible
				move_status_pub_.publish(move_status_);
			}
		}
		else{
			ROS_INFO("Robot reached the temporal sub_goal, shift to normal planning");
			/*
			if (global_path.poses.size() != 0){
				hybrid_path_pub_.publish(global_path);
			}
			*/
			hybrid_path_pub_.publish(empty_path);
			move_status_pub_.publish(move_status_);
			rrts_is_replaning_ = false;
			is_first_goal = true;
		}
	}

	if (!rrts_is_replaning_ || !replanZone()){
		ROS_INFO("Normal planning");
		/*
		if (global_path.poses.size() != 0){
			hybrid_path_pub_.publish(global_path);
		}
		*/
		hybrid_path_pub_.publish(empty_path);
		planner->planner_timer.stop();
		move_status_pub_.publish(move_status_);
	}
}

#else

void RePlanner::plannerReasonning(){
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
		double check_dist = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), sub_goal.pose.position.x, sub_goal.pose.position.y);
		ROS_INFO("Distance to sub_goal: %f" , check_dist);
		if (check_dist > 1.5){
			if (trajectory_found_){
				ROS_INFO("Robot heading to the sub_goal");
				combineHybridPlan();
				hybrid_path_pub_.publish(hybrid_path);
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
			ROS_INFO("Robot reached the temporal sub_goal");
			hybrid_path_pub_.publish(hybrid_path);
			move_status_pub_.publish(move_status_);
			rrts_is_replaning_ = false;
			is_first_goal = true;
		}
	}

	if (!rrts_is_replaning_){
		ROS_INFO("Normal planning");
		hybrid_path_pub_.publish(hybrid_path);
		planner->planner_timer.stop();
		move_status_pub_.publish(move_status_);
	}
}
#endif

void RePlanner::combineHybridPlan(){
	int local_temp_no = 0;
	int global_temp_no = waypointNo_ + 1;
	hybrid_path.poses.clear();
	while (local_temp_no < local_path.poses.size()){
		hybrid_path.poses.push_back(local_path.poses[local_temp_no]);
		local_temp_no ++;
	}
	while (global_temp_no < global_path.poses.size()){
		hybrid_path.poses.push_back(global_path.poses[global_temp_no]);
		global_temp_no ++;
	}
}



void RePlanner::rrtsReplanning(){
	/*
	if (is_first_replan){
		planner = new PlannerExp;
		is_first_replan = false;
	}
	*/
	if (is_first_goal){
		getNearestWaypoints();
		is_first_goal =false;
		planner->planner_timer.start();
	}
}

int main(int argc, char** argcv){
    ros::init(argc, argcv, "ped_utown");
    RePlanner rp;
    ros::spin();
    return 0;
}



