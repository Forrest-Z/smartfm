/*
 * local_map_ext.cpp
 *
 *  Created on: Aug 16, 2012
 *      Author: demian
 */

#include "local_map_ext.h"


LocalMap::LocalMap(double height, double width, double res):height_(height), width_(width),
private_nh_("~")
{

	obsMetric = new obsDistMetric();
	useMetric = true;
	obstAnalyze = true;

	if (useMetric)
		obstValue = 127;
	else
		obstValue = 0;

    local_map_.info.resolution = res;
    local_map_.info.height = height/res; //x axis in map frame
    local_map_.info.width = width/res; //-y axis in map frame

    dist_map_.info.resolution = res;
    dist_map_.info.height = height/res; //x axis in map frame
    dist_map_.info.width = width/res; //-y axis in map frame

    updateMapSkipMax = 5;
    updateMapSkip = 0;

    const string default_global = "/map";
    const string default_local = "/local_map";
    const string default_base = "/base_link";

    private_nh_.param("global_frame", global_frame_, default_global);
    private_nh_.param("local_frame", local_frame_, default_local);
    private_nh_.param("base_frame", base_frame_, default_base);
    //global_frame_ = "/map";
    //local_frame_ = "/local_map";
    //base_frame_ = "/base_link";
    tf_sleeper_ = new ros::Duration(1/20.0);

    pointcloud_sub_.subscribe(nh_, "sickldmrs/cloud", 10);
    pointcloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pointcloud_sub_, tf_, local_frame_, 10);
    pointcloud_filter_->registerCallback(boost::bind(&LocalMap::pointcloudCallback, this, _1));

    laser_sub_.subscribe(nh_, "scan_in", 10);
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

    move_status_sub_ = nh_.subscribe("move_status", 1, &LocalMap::movestatusCallBack, this);
    clear_space_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("clear_space",1);
    obst_dist_pub_ = nh_.advertise<pnc_msgs::move_status>("move_status_repub",1);
    obst_view_pub_ = nh_.advertise<sensor_msgs::PointCloud>("obst_view",1);

    nav_msgs::GetMap::Request  req;
    nav_msgs::GetMap::Response resp;
    ROS_INFO("Requesting the prior map...");
    while(!ros::service::call("static_map", req, resp))
    {
        ROS_WARN("Request for map failed; trying again...");
        ros::Duration d(0.5);
        d.sleep();
    }

    prior_pts_.header = resp.map.header;

    unsigned int left_lane = 0, right_lane = 0;
    for(unsigned int i=0; i<resp.map.info.width; i++)
    {
        for(unsigned int j=0; j<resp.map.info.height; j++)
        {
            int map_index = (j * resp.map.info.width + i);
            geometry_msgs::Point32 map_p;

            //todo: take care of different type of obstacles
            //map_p.z = resp.map.data[map_index];

            map_p.x = i*resp.map.info.resolution + resp.map.info.origin.position.x;
            map_p.y = j*resp.map.info.resolution + resp.map.info.origin.position.y;
            if(resp.map.data[map_index] > 0)
            {
                prior_pts_.points.push_back(map_p);
                prior_obs_master_.push_back(resp.map.data[map_index]);
                //initialize the prior_obs the same as the master

                if(resp.map.data[map_index] == 107) left_lane++;
                if(resp.map.data[map_index] == 87) right_lane++;;
            }
        }
    }
    prior_obs_ = prior_obs_master_;
    cout<<"left lane = "<<left_lane<<" right_lane = "<<right_lane<<endl;
    prior_pts_pub_.publish(prior_pts_);

    prior_map_ = resp.map;
    ROS_INFO("Prior map retrieved...");

    tf::Quaternion q;
    q.setRPY(0.0, 0.0,-M_PI/2.0);
    transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(-height/4.0,width/2.0,0.0)), ros::Time::now()+*tf_sleeper_, base_frame_, local_frame_ );
    ros::Timer timer = nh_.createTimer(ros::Duration(*tf_sleeper_), &LocalMap::timerCallback, this);
    norminal_lane_sub_ = nh_.subscribe("norminal_lane", 1, &LocalMap::norminalLane, this);
    ros::spin();
};


void LocalMap::updatePriorObsWithLane(bool norminal)
{
	if(norminal)
	{
		prior_obs_ = prior_obs_master_;
	}
	else
	{
		for(size_t i = 0; i < prior_obs_master_.size(); i++)
		{
			if(prior_obs_master_[i] == 107) prior_obs_[i] = 87;
			if(prior_obs_master_[i] == 87) prior_obs_[i] = 107;
		}
	}
}

void LocalMap::norminalLane(std_msgs::Bool norminal_lane)
{
	//update the prior points
	if(norminal_lane.data)
		ROS_WARN("We are now moving towards McD");
	else
		ROS_WARN("We are now moving away from McD");
	updatePriorObsWithLane(norminal_lane.data);
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
    //ROS_INFO("Size of freecells, %d", free_cells.size());
}

void LocalMap::timerCallback(const ros::TimerEvent &event)
{
    transform_.stamp_ = ros::Time::now() + *tf_sleeper_;
    broadcaster_.sendTransform(transform_);
};

void LocalMap::pointcloudCallback(sensor_msgs::PointCloud2ConstPtr pc)
{
    sensor_msgs::PointCloud obs_pts;
    sensor_msgs::convertPointCloud2ToPointCloud(*pc, obs_pts);

    try
    {
        tf_.transformPointCloud(local_frame_, obs_pts, obs_pts);
    }
    catch (tf::TransformException &ex)
    {
        ROS_DEBUG("Failure %s\n", ex.what()); //Print exception which was caught
        return;
    }
    updateMap(obs_pts);
};


void LocalMap::laserCallback(sensor_msgs::LaserScanConstPtr scan_in)
{
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
    updateMap(laser_cloud);
};


void LocalMap::laser2Callback(sensor_msgs::LaserScanConstPtr scan_in)
{
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
    laser2_coop_pts_ = laser_cloud;
    cout<<"laser2"<<endl;
};

void LocalMap::laser3Callback(sensor_msgs::LaserScanConstPtr scan_in)
{
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
    laser3_coop_pts_ = laser_cloud;
    cout<<"laser3"<<endl;
};

void LocalMap::addPointToMap(geometry_msgs::Point32 map_p){
    addPointToMap(map_p, 0);
}

void LocalMap::addPointToMap(geometry_msgs::Point32 map_p, int occ)
{
    if(!fmutil::isWithin(map_p.x, 0.0, width_)) return;
    if(!fmutil::isWithin(map_p.y, 0.0, height_)) return;
    int map_px = (map_p.x)/local_map_.info.resolution;
    int map_py = (map_p.y)/local_map_.info.resolution;
    unsigned int map_index = (map_py * local_map_.info.width + map_px);
    if(map_index < local_map_.data.size())
    {
        local_map_.data[map_index] = occ;
        map_p.z = occ/100.0;
    }
}

void LocalMap::updateMap(sensor_msgs::PointCloud& pc)
{
    //cout<<"Map update call"<<endl;
    if(updateMapSkip < updateMapSkipMax)
    {
        updateMapSkip++;
        return;
    }
    else
        updateMapSkip = 0;

    tf::Stamped<tf::Pose> robot_pose;
    sensor_msgs::PointCloud prior_pts_local;
    //a naive implementation where current sensor will just reset the map to prior map with latest obs
    local_map_.data.clear();
    local_map_.data.resize(local_map_.info.width*local_map_.info.height);
    dist_map_.data.clear();
    dist_map_.data.resize(dist_map_.info.width*dist_map_.info.height);
    local_map_pts_.points.clear();
    prior_pts_.header.stamp = pc.header.stamp;
    bool getPose = getRobotPose(robot_pose);
    if(!getPose) ROS_WARN("getRobotPose failed");
    if(getPose){
        try{
            tf_.transformPointCloud(local_frame_, prior_pts_, prior_pts_local);
            local_map_pts_.header = prior_pts_local.header;
            for(size_t i=0; i<prior_pts_local.points.size(); i++){
                addPointToMap(prior_pts_local.points[i], prior_obs_[i]);
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_DEBUG("Failure %s\n", ex.what());
            return;
        }
        for(size_t i=0; i<pc.points.size(); i++)
            addPointToMap(pc.points[i]);

        //adding obstacles from coop
        for(size_t i=0; i<laser2_coop_pts_.points.size(); i++)
            addPointToMap(laser2_coop_pts_.points[i]);
        for(size_t i=0; i<laser3_coop_pts_.points.size(); i++)
            addPointToMap(laser3_coop_pts_.points[i]);

        if (useMetric)
        	getGradientCost(local_map_, dist_map_);

        geometry_msgs::PoseStamped origin;
        tf::poseStampedTFToMsg(robot_pose, origin);
        local_map_.info.origin =origin.pose;
        local_map_.header.frame_id = global_frame_;
        local_map_.header.stamp = pc.header.stamp;
        dist_map_.info.origin =origin.pose;
        dist_map_.header.frame_id = global_frame_;
        dist_map_.header.stamp = pc.header.stamp;
        pnc_msgs::local_map lm;
        lm.occupancy = local_map_;
        lm.dist = dist_map_;
        vector<int> free_cells;
        publishLocalMapPts(free_cells);
        lm.free_cells = free_cells;
        map_pub_.publish(lm);
    }
};

bool LocalMap::getRobotPose(tf::Stamped<tf::Pose> &odom_pose) const
{
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
	robot_FP_UL.y = -0.3;//-1/2*GOLFCAR_WIDTH;

	robot_FP_UR.x = 1.8;//3/4*GOLFCAR_HEIGHT;
	robot_FP_UR.y = 0.3;//1/2*GOLFCAR_WIDTH;

	//collision detection area
	point = robot_FP_UL;
	lookahead_points.push_back(point);

	point.x = robot_FP_UL.x+cos(steer_angle)*TRACKING_DIST;
	point.y = robot_FP_UL.y+sin(steer_angle)*TRACKING_DIST;
	lookahead_points.push_back(point);

	point.x = robot_FP_UR.x+cos(steer_angle)*TRACKING_DIST;
	point.y = robot_FP_UR.y+sin(steer_angle)*TRACKING_DIST;
	lookahead_points.push_back(point);

	point = robot_FP_UR;
	lookahead_points.push_back(point);

	clear_space.header.stamp = ros::Time::now();
	clear_space.header.frame_id = base_frame_;
	clear_space.polygon.points = lookahead_points;
	clear_space_pub_.publish(clear_space);
	getNearestObst();
}

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
	for (int i = 0; i < clear_space.polygon.points.size(); i++){
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
	for (int i = 0; i<pointst.size();i++){
		if (pointst[i].point.x > xmax)
			xmax = pointst[i].point.x;
		if (pointst[i].point.x < xmin)
			xmin = pointst[i].point.x;
		if (pointst[i].point.y > ymax)
			ymax = pointst[i].point.y;
		if (pointst[i].point.y < ymin)
			ymin = pointst[i].point.y;
	}
	//ROS_INFO("xmin: %f,xmax: %f,ymin: %f,ymax: %f", xmin, xmax, ymin, ymax);

	int xminid = (int) (xmin/local_map_.info.resolution) ;
	int xmaxid = (int) (xmax/local_map_.info.resolution) +1;
	int yminid = (int) (ymin/local_map_.info.resolution) +3;
	int ymaxid = (int) (ymax/local_map_.info.resolution) +3;
	//ROS_INFO("xmin: %d,xmax: %d,ymin: %d,ymax: %d", xminid, xmaxid, yminid, ymaxid);

	unsigned int obst_count = 0;
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

void LocalMap::getGradientCost(nav_msgs::OccupancyGrid &gradient_local_map_, nav_msgs::OccupancyGrid &dist_map_){
	obsMetric -> initObstMap(gradient_local_map_);
	obsMetric -> updateDistMap();
	obsMetric->updateCost(gradient_local_map_, dist_map_);
}

int main(int argc, char** argcv){
    ros::init(argc, argcv, "local_map");
    LocalMap lm(30.0, 30.0, 0.2);

    return 0;
}
