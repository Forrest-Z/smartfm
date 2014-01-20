/*
 * local_map.cpp
 *
 *  Created on: Nov 13, 2013
 *      Author: liuwlz
 */

#include <Metric_Map/local_map.h>

namespace MPAV{

	LocalMap::LocalMap():
	private_nh_("~"){

		obsMetric = new DistMetric();
		useMetric = true;
		priorInit = false;
		updateMapInit = false;
		firstStationGoal = true;

		if (useMetric)
			obstValue = 127;
		else
			obstValue = 0;

		const string default_global = "map";
		const string default_local = "local_map";
		const string default_base = "base_link";
		const int default_track_dist = 5;
		const double default_width = 16;
		const double default_height = 20;
		const double default_resolution = 0.2;
		const double default_extend = 6.0;

		private_nh_.param("global_frame", global_frame_, default_global);
		private_nh_.param("local_frame", local_frame_, default_local);
		private_nh_.param("base_frame", base_frame_, default_base);
		private_nh_.param("track_dist", track_dist_, default_track_dist);
		private_nh_.param("width", width_, default_width);
		private_nh_.param("height",height_,default_height);
		private_nh_.param("resolution", resolution_, default_resolution);
		private_nh_.param("extend_dist", extend_dist, default_extend);
		private_nh_.param("coop_trigger", CoopTrigger, false);

		local_map_.info.resolution = resolution_;
		local_map_.info.height = height_/resolution_; //x axis in map frame
		local_map_.info.width = width_/resolution_; //-y axis in map frame

		updateMapSkipMax = 5;
		updateMapSkip = 0;

		tf_sleeper_ = new ros::Duration(1/20.0);

		pointcloud_sub_.subscribe(nh_, "sickldmrs/cloud", 10);
		pointcloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pointcloud_sub_, tf_, local_frame_, 10);
		pointcloud_filter_->registerCallback(boost::bind(&LocalMap::pointcloudCallback, this, _1));

		laser_sub_.subscribe(nh_, "sick_scan2", 10);
		laser_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, local_frame_, 10);
		laser_filter_->registerCallback(boost::bind(&LocalMap::laserCallback, this, _1));

		laser2_sub_.subscribe(nh_, "scan_in2", 10);
		laser2_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser2_sub_, tf_, local_frame_, 10);
		laser2_filter_->registerCallback(boost::bind(&LocalMap::laser2Callback, this, _1));

		laser3_sub_.subscribe(nh_, "scan_in3", 10);
		laser3_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser3_sub_, tf_, local_frame_, 10);
		laser3_filter_->registerCallback(boost::bind(&LocalMap::laser3Callback, this, _1));

		prior_pts_pub_ = nh_.advertise<sensor_msgs::PointCloud>("prior_pts", 10, true);
		map_pub_ = nh_.advertise<pnc_msgs::local_map>("local_map", 10);
		map_pts_pub_ = nh_.advertise<sensor_msgs::PointCloud>("local_map_pts", 10);

		clear_space_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("local_clear_space",1);
		obst_dist_pub_ = nh_.advertise<pnc_msgs::move_status>("move_status_repub",1);
		obst_view_pub_ = nh_.advertise<sensor_msgs::PointCloud>("obst_view",1);
		prior_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("prior_map",1);

		move_status_sub_ = nh_.subscribe("move_status", 1, &LocalMap::movestatusCallBack, this);
		reference_path_sub_ = nh_.subscribe("global_plan", 1, &LocalMap::referencePathCallBack, this);
		station_goal_sub_ = nh_.subscribe("sm_station_goal", 1, &LocalMap::StationGoalCannback, this);
		nav_msgs::GetMap::Request  req;
		nav_msgs::GetMap::Response resp;
		ROS_INFO("Requesting the prior map...");
		while(!ros::service::call("static_map", req, resp)){
			ROS_WARN("Request for map failed; trying again...");
			ros::Duration d(0.5);
			d.sleep();
			ros::spinOnce();
		}
		prior_map_ = resp.map;
		ROS_INFO("Prior map retrieved...");

		tf::Quaternion q;
		q.setRPY(0.0, 0.0,-M_PI/2.0);
		transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(-height_/4.0,width_/2.0,0.0)), ros::Time::now()+*tf_sleeper_, base_frame_, local_frame_ );
		ros::Timer tf_timer = nh_.createTimer(ros::Duration(*tf_sleeper_), &LocalMap::tfTimerCallback, this);

		ros::spin();
	};

	void LocalMap::referencePathCallBack(const nav_msgs::Path ref_path){
		if (!priorInit){
			ROS_INFO("Reference path retrived...");
			reference_path = ref_path;
			extendReferencePath();
			priorInit = true;
		}
		else
			return;
	}

	#define SQ(x) x * x

	void LocalMap::extendReferencePath(){
		ROS_INFO("Initialise the Distance Map for Reference Path, it Will TAKE A WHILE");
		//Initialise all the space being free;
		assert(prior_map_.data.size() > 0);

		for (size_t i = 0 ; i < prior_map_.data.size(); i++){
			prior_map_.data[i] = 100;
		}

		for (size_t i = 0 ; i < reference_path.poses.size(); i++){
			geometry_msgs::PoseStamped pose_curr;
			pose_curr = reference_path.poses[i];
			addPointToPriorMap(pose_curr.pose.position);
		}

		refPathDist = new DistMetric();
		refPathDist->initObstacle(prior_map_);
		refPathDist->setMaxDist(1.5*extend_dist/prior_map_.info.resolution);
		refPathDist->updateDistMap();
		refPathDist->getDistMap(prior_map_);
		initPriorPoints();
		delete refPathDist;
	}

	void LocalMap::initPriorPoints(){
		ROS_INFO("Initialise prior points");
		prior_pts_.header = prior_map_.header;
		for(unsigned int i=0; i<prior_map_.info.width; i++){
			for(unsigned int j=0; j<prior_map_.info.height; j++){
				int map_index = (j * prior_map_.info.width + i);

				geometry_msgs::Point32 map_p;
				map_p.x = i*prior_map_.info.resolution + prior_map_.info.origin.position.x;
				map_p.y = j*prior_map_.info.resolution + prior_map_.info.origin.position.y;

				double min_dist = ((double)prior_map_.data[map_index])/100.0* 1.5 * extend_dist;
				if(min_dist < extend_dist){
					prior_pts_.points.push_back(map_p);
					prior_obs_.push_back(prior_map_.data[map_index]+1);
				}
			}
		}
		prior_pts_pub_.publish(prior_pts_);
		ROS_INFO("Prior Points Initialised");
	}

	void LocalMap::StationGoalCannback(const geometry_msgs::PoseStamped goalIn){
		if (firstStationGoal){
			station_goal = goalIn;
			firstStationGoal = false;
		}
		else{
			if (station_goal.pose.position.x != goalIn.pose.position.x){
				priorInit= false;
				firstStationGoal = true;
			}
		}
	}

	void LocalMap::tfTimerCallback(const ros::TimerEvent &event){
		transform_.stamp_ = ros::Time::now() + *tf_sleeper_;
		broadcaster_.sendTransform(transform_);
	};

	void LocalMap::pointcloudCallback(sensor_msgs::PointCloud2ConstPtr pc){
		sensor_msgs::PointCloud obs_pts;
		sensor_msgs::convertPointCloud2ToPointCloud(*pc, obs_pts);

		try{
			tf_.transformPointCloud(local_frame_, obs_pts, obs_pts);
		}
		catch (tf::TransformException &ex){
			ROS_DEBUG("Failure %s\n", ex.what()); //Print exception which was caught
			return;
		}
		if (priorInit)
			updateMap(obs_pts);
	};

	void LocalMap::laserCallback(sensor_msgs::LaserScanConstPtr scan_in){
		sensor_msgs::LaserScan scan_copy = *scan_in;
		sensor_msgs::PointCloud laser_cloud;
		try{
			projector_.transformLaserScanToPointCloud(local_frame_, scan_copy,
					laser_cloud, tf_);
		}
		catch (tf::TransformException& e){
			ROS_ERROR("%s",e.what());
			return;
		}
		laser1_pts = laser_cloud;
		if (priorInit)
			updateMap(laser_cloud);
	};

	void LocalMap::laser2Callback(sensor_msgs::LaserScanConstPtr scan_in){
		sensor_msgs::LaserScan scan_copy = *scan_in;
		sensor_msgs::PointCloud laser_cloud;
		try{
			projector_.transformLaserScanToPointCloud(local_frame_, scan_copy,
					laser_cloud, tf_);
		}
		catch (tf::TransformException& e){
			ROS_ERROR("%s",e.what());
			return;
		}
		laser2_pts_ = laser_cloud;
	};

	void LocalMap::laser3Callback(sensor_msgs::LaserScanConstPtr scan_in){
		sensor_msgs::LaserScan scan_copy = *scan_in;
		sensor_msgs::PointCloud laser_cloud;
		try{
			projector_.transformLaserScanToPointCloud(local_frame_, scan_copy,
					laser_cloud, tf_);
		}
		catch (tf::TransformException& e){
			ROS_ERROR("%s",e.what());
			return;
		}
		laser3_pts_ = laser_cloud;
	};

	void LocalMap::addPointToPriorMap(geometry_msgs::Point map_p){
		int map_px = (map_p.x)/prior_map_.info.resolution;
		int map_py = (map_p.y)/prior_map_.info.resolution;
		unsigned int map_index = (map_py * prior_map_.info.width + map_px);
		if(map_index < prior_map_.data.size()){
			prior_map_.data[map_index] = 0;
		}
	}

	void LocalMap::addPointToMap(geometry_msgs::Point32 map_p){
		addPointToMap(map_p, 0);
	}

	void LocalMap::addPointToMap(geometry_msgs::Point32 map_p, int occ){
		if ( (map_p.x < 0) || (map_p.x > width_) || (map_p.y < 0) || (map_p.y > height_))
			return;
		int map_px = (map_p.x)/local_map_.info.resolution;
		int map_py = (map_p.y)/local_map_.info.resolution;
		unsigned int map_index = (map_py * local_map_.info.width + map_px);
		if(map_index < local_map_.data.size()){
			local_map_.data[map_index] = occ;
			map_p.z = occ/100.0;
		}
	}

	void LocalMap::updateMap(sensor_msgs::PointCloud& pc)
	{
		if (!updateMapInit)
			ROS_INFO("Wait for LaserScan");
		if(updateMapSkip < updateMapSkipMax)
		{
			updateMapSkip++;
			return;
		}
		else
			updateMapSkip = 0;



		tf::Stamped<tf::Pose> robot_pose;
		sensor_msgs::PointCloud prior_pts_local;
		replan_check_pts.points.clear();
		replan_check_pts.header.frame_id =local_frame_;

		local_map_.data.clear();
		local_map_.data.resize(local_map_.info.width*local_map_.info.height);
		local_map_pts_.points.clear();
		prior_pts_.header.stamp = pc.header.stamp;
		bool getPose = getRobotPose(robot_pose);
		if(!getPose) ROS_WARN("getRobotPose failed");
		if(getPose){
			try{
				tf_.transformPointCloud(local_frame_, prior_pts_, prior_pts_local);
				local_map_pts_.header = prior_pts_local.header;
				for(size_t i=0; i<prior_pts_local.points.size(); i++){
					addPointToMap(prior_pts_local.points[i], 107);
				}
			}
			catch (tf::TransformException &ex){
				ROS_DEBUG("Failure %s\n", ex.what());
				return;
			}
			for(size_t i=0; i<pc.points.size(); i++){
				addPointToMap(pc.points[i]);
				if (!CoopTrigger)
					replan_check_pts.points.push_back(pc.points[i]);
			}
			for(size_t i=0; i<laser2_pts_.points.size(); i++){
				addPointToMap(laser2_pts_.points[i]);
				replan_check_pts.points.push_back(laser2_pts_.points[i]);
			}
			for(size_t i=0; i<laser3_pts_.points.size(); i++){
				addPointToMap(laser3_pts_.points[i]);
				replan_check_pts.points.push_back(laser3_pts_.points[i]);
			}

			if (useMetric)
				getGradientCost(local_map_);

			geometry_msgs::PoseStamped origin;
			tf::poseStampedTFToMsg(robot_pose, origin);
			local_map_.info.origin =origin.pose;
			local_map_.header.frame_id = global_frame_;
			local_map_.header.stamp = pc.header.stamp;
			pnc_msgs::local_map lm;
			lm.occupancy = local_map_;
			vector<int> free_cells;
			publishLocalMapPts(free_cells);
			lm.free_cells = free_cells;
			map_pub_.publish(lm);
		}
		updateMapInit = true;
	}

	void LocalMap::publishLocalMapPts(vector<int> &free_cells){
		local_map_pts_.header.stamp = ros::Time::now();
		local_map_pts_.points.clear();
		for(unsigned int i=0; i<local_map_.info.width; i++){
			for(unsigned int j=0; j<local_map_.info.height; j++){
				geometry_msgs::Point32 map_p;
				map_p.x = i*local_map_.info.resolution;
				map_p.y = j*local_map_.info.resolution;
				map_p.z = (local_map_.data[j * local_map_.info.width +i])/100.0;
				unsigned int map_index = (j * local_map_.info.width + i);
				//ROS_INFO("local_map value ,%d", local_map_.data[j * local_map_.info.width +i]);
				if(local_map_.data[j * local_map_.info.width +i] != obstValue)
					free_cells.push_back(map_index);
				local_map_pts_.points.push_back(map_p);
			}
		}
		map_pts_pub_.publish(local_map_pts_);
	}

	bool LocalMap::getRobotPose(tf::Stamped<tf::Pose> &odom_pose) const{

		odom_pose.setIdentity();
		tf::Stamped<tf::Pose> robot_pose;
		robot_pose.setIdentity();
		robot_pose.frame_id_ = local_frame_;
		robot_pose.stamp_ = ros::Time();
		ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

		try {
			tf_.transformPose(global_frame_, robot_pose, odom_pose);
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
		// check odom_pose timeout
		if (current_time.toSec() - odom_pose.stamp_.toSec() > 0.3) {
			ROS_WARN("Get robot pose transform timeout. Current time: %.4f, odom_pose stamp: %.4f, tolerance: %.4f",
					current_time.toSec(), odom_pose.stamp_.toSec(), 0.1);
			return false;
		}
		return true;
	};

	void LocalMap::movestatusCallBack(const pnc_msgs::move_status move_status){
		move_status_ = move_status;
		if (priorInit && updateMapInit)
			getClearSpace();
	}

	void LocalMap::getClearSpace(){
		vector<geometry_msgs::Point32> lookahead_points;
		geometry_msgs::Point32 point;
		//a rectangle footprint is assumed starting from lower left
		double steer_angle = move_status_.steer_angle;
		//double steer_angle = 0;

		geometry_msgs::Point32 robot_FP_UL;
		geometry_msgs::Point32 robot_FP_UR;

		robot_FP_UL.x = 1.8;//3/4*GOLFCAR_HEIGHT;
		robot_FP_UL.y = -0.6;//-1/2*GOLFCAR_WIDTH;

		robot_FP_UR.x = 1.8;//3/4*GOLFCAR_HEIGHT;
		robot_FP_UR.y = 0.6;//1/2*GOLFCAR_WIDTH;

		//collision detection area
		point = robot_FP_UL;
		lookahead_points.push_back(point);

		point.x = robot_FP_UL.x+cos(steer_angle)*track_dist_;
		point.y = robot_FP_UL.y+sin(steer_angle)*track_dist_;
		lookahead_points.push_back(point);

		point.x = robot_FP_UR.x+cos(steer_angle)*track_dist_;
		point.y = robot_FP_UR.y+sin(steer_angle)*track_dist_;
		lookahead_points.push_back(point);

		point = robot_FP_UR;
		lookahead_points.push_back(point);

		clear_space.header.stamp = ros::Time::now();
		clear_space.header.frame_id = base_frame_;
		clear_space.polygon.points = lookahead_points;
		clear_space_pub_.publish(clear_space);
		getNearestObst();
	}

#if 0
	void LocalMap::getNearestObst(){
		//Transform polygon points from /base_link to /local_map
		geometry_msgs::Point32 obst_view;
		sensor_msgs::PointCloud obst_detected;
		obst_detected.header.frame_id = local_frame_;
		//ROS_INFO("Get Nearest Obstacle_380");
		vector<geometry_msgs::PointStamped> pointst(clear_space.polygon.points.size());
		geometry_msgs::PointStamped temp_point;
		bool tf_success = false;
		temp_point.header.stamp = ros::Time();
		temp_point.header.frame_id = clear_space.header.frame_id;
		for (size_t i = 0; i < clear_space.polygon.points.size(); i++){
			temp_point.point.x = clear_space.polygon.points[i].x;
			temp_point.point.y = clear_space.polygon.points[i].y;
			while (!tf_success){
				try{
					tf_.transformPoint(local_frame_, temp_point, pointst[i]);
					tf_success = true;
					//ROS_INFO("Get Nearest Obstacle_393");
				}
				catch(tf::LookupException& ex) {
					ROS_ERROR("No Transform available Error: %s\n", ex.what());
				}
			}
			tf_success = false;
		}
		//Simplly expand the checking area to save the computation cost of getting filling cells
		double xmin = 9999;
		double xmax = 0;
		double ymin = 9999;
		double ymax = 0;
		for (size_t i = 0; i<pointst.size();i++){
			//if (pointst[i].point.x > xmax)
			xmax = pointst[i].point.x > xmax ? pointst[i].point.x : xmax;
			//if (pointst[i].point.x < xmin)
			xmin = pointst[i].point.x < xmin ? pointst[i].point.x : xmin;
			//if (pointst[i].point.y > ymax)
			ymax = pointst[i].point.y > ymax ? pointst[i].point.y : ymax;
			//if (pointst[i].point.y < ymin)
			ymin = pointst[i].point.y < ymin ? pointst[i].point.y : ymin;
		}

		int xminid = (int) (xmin/local_map_.info.resolution) ;
		int xmaxid = (int) (xmax/local_map_.info.resolution) +1;
		int yminid = (int) (ymin/local_map_.info.resolution) +3;
		int ymaxid = (int) (ymax/local_map_.info.resolution) +3;

		for (int i = xminid; i < xmaxid; i++){
			for (int j = yminid ; j < ymaxid; j++){
				int index = j*local_map_.info.width + i;
				if (local_map_.data[index] == obstValue){
					move_status_.emergency = true;
					//ROS_INFO("Obst Detected");
					obst_view.x = i*local_map_.info.resolution;
					obst_view.y = j*local_map_.info.resolution;
					obst_detected.points.push_back(obst_view);
				}
			}
		}
		obst_view_pub_ .publish(obst_detected);
		obst_dist_pub_.publish(move_status_);
	}

#else
	void LocalMap::getNearestObst(){
		sensor_msgs::PointCloud obst_detected;
		obst_detected.header.frame_id = local_frame_;
		vector<geometry_msgs::PointStamped> pointst(clear_space.polygon.points.size());
		geometry_msgs::PointStamped temp_point;
		bool tf_success = false;
		temp_point.header.stamp = ros::Time();
		temp_point.header.frame_id = clear_space.header.frame_id;
		for (size_t i = 0; i < clear_space.polygon.points.size(); i++){
			temp_point.point.x = clear_space.polygon.points[i].x;
			temp_point.point.y = clear_space.polygon.points[i].y;
			while (!tf_success){
				try{
					tf_.transformPoint(local_frame_, temp_point, pointst[i]);
					tf_success = true;
				}
				catch(tf::LookupException& ex) {
					ROS_ERROR("No Transform available Error: %s\n", ex.what());
				}
			}
			tf_success = false;
		}
		//Simplly expand the checking area to save the computation cost of getting filling cells
		double xmin = 9999;
		double xmax = 0;
		double ymin = 9999;
		double ymax = 0;
		for (size_t i = 0; i<pointst.size();i++){
			xmax = pointst[i].point.x > xmax ? pointst[i].point.x : xmax;
			xmin = pointst[i].point.x < xmin ? pointst[i].point.x : xmin;
			ymax = pointst[i].point.y > ymax ? pointst[i].point.y : ymax;
			ymin = pointst[i].point.y < ymin ? pointst[i].point.y : ymin;
		}
		//ROS_INFO("Dist, xmin: %f, xmax: %f", xmin, xmax);
		for (size_t i = 0 ; i < replan_check_pts.points.size(); i++){
			geometry_msgs::Point32 temp_pts = replan_check_pts.points[i];
			if(temp_pts.x > xmin && temp_pts.x < xmax && temp_pts.y > ymin && temp_pts.y < ymax){
				move_status_.emergency = true;
				obst_detected.points.push_back(temp_pts);
				ROS_INFO("X: %f, Y: %f",temp_pts.x, temp_pts.y);
				break;
			}
		}
		replan_check_pts.header.stamp = ros::Time::now();
		obst_view_pub_ .publish(obst_detected);
		obst_dist_pub_.publish(move_status_);
	}
#endif

	void LocalMap::getGradientCost(nav_msgs::OccupancyGrid &gradient_local_map_){
		obsMetric->setMaxDist(2.0*extend_dist/prior_map_.info.resolution);
		obsMetric-> initObstacle(gradient_local_map_);
		obsMetric-> updateDistMap();
		obsMetric->updateCost(gradient_local_map_);
	}
}
