/*
 * obst_detect.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: liuwlz
 */

#include "obst_detect.h"
ObstDetect::ObstDetect(){

	local_frame_ = "/local_map";

    pointcloud_sub_.subscribe(nh_, "sickldmrs/cloud", 10);
    pointcloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pointcloud_sub_, tf_, local_frame_ , 10);
    pointcloud_filter_->registerCallback(boost::bind(&ObstDetect::PointcloudCB, this, _1));

    laser1_sub_.subscribe(nh_, "base_scan", 10);
    laser1_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser1_sub_, tf_, local_frame_ , 10);
    laser1_filter_->registerCallback(boost::bind(&ObstDetect::Laser1CB, this, _1));

    laser2_sub_.subscribe(nh_, "scan_in2", 10);
    laser2_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser2_sub_, tf_, local_frame_ , 10);
    laser2_filter_->registerCallback(boost::bind(&ObstDetect::Laser2CB, this, _1));

    laser3_sub_.subscribe(nh_, "scan_in3", 10);
    laser3_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser3_sub_, tf_, local_frame_ , 10);
    laser3_filter_->registerCallback(boost::bind(&ObstDetect::Laser3CB, this, _1));

	processed_pts_pub = nh_.advertise<sensor_msgs::PointCloud> ("processed_obst_pts", 10);
	unprocessed_pts_pub = nh_.advertise<sensor_msgs::PointCloud>("unprocessed_obst_pts", 10);
	interest_pts_pub = nh_.advertise<sensor_msgs::PointCloud>("interest_pts", 10);
	obst_polys_pub = nh_.advertise<POLY>("obstacles_polygon", 1);
	obst_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_makers", 1);
	obst_pose_pub = nh_.advertise<sensor_msgs::PointCloud>("obst_pose_measure",1);
	obst_info_pub = nh_.advertise<obstacle_tracking::obst_info>("obst_info", 1);

	//Dynamic reconfigure of cluster parameters
	srv = new dynamic_reconfigure::Server<obstacle_tracking::clusterCTRConfig> (ros::NodeHandle("~"));
	dynamic_reconfigure::Server<obstacle_tracking::clusterCTRConfig>::CallbackType f = boost::bind(&ObstDetect::ParamReconfigure, this, _1, _2);
	srv->setCallback(f);

	obst_channel.name = "obst_type";
	mesh = visualization_msgs::Marker::MESH_RESOURCE;
	cylinder = visualization_msgs::Marker::CYLINDER;
	cube = visualization_msgs::Marker::CUBE;
}

ObstDetect::~ObstDetect(){

}

//Transfer ros pointcloud to pcl pointcloud: pointcloud->pointcloud2->PCL
inline pcl::PointCloud<pcl::PointXYZ> pointcloudToPCL(sensor_msgs::PointCloud &pc){
    sensor_msgs::PointCloud2 pc2;
    pcl::PointCloud<pcl::PointXYZ> pcl;
    sensor_msgs::convertPointCloudToPointCloud2(pc, pc2);
    pcl::fromROSMsg(pc2, pcl);
    return pcl;
}

inline sensor_msgs::PointCloud PCLTopointcloud(pcl::PointCloud<pcl::PointXYZ> &pcl){
    sensor_msgs::PointCloud pc;
	sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(pcl, pc2);
    sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc);
    return pc;
}

inline double PointDistCal(POSE first_pts, POSE second_pts){
	double pts_dist;
	double x_diff = first_pts.x - second_pts.x;
	double y_diff = first_pts.y - second_pts.y;
	pts_dist = sqrt(x_diff*x_diff + y_diff*y_diff);
	return pts_dist;
}

inline double SlopeCal(POSE first_pts, POSE second_pts){
	double slope;
	double x_diff = first_pts.x - second_pts.x;
	double y_diff = first_pts.y - second_pts.y;
	if (x_diff != 0)
		slope = y_diff/x_diff;
	else
		slope = DBL_MAX;
	return slope;
}

inline POSE PendicularPointCal(POSE first_pts, POSE second_pts, double dist){
	POSE pend_point;
	double slope = SlopeCal(first_pts, second_pts);
	pend_point.y = first_pts.y + sqrt(dist*dist / (1+slope*slope));
	pend_point.x = first_pts.x - slope*sqrt(dist*dist / (1+slope*slope));
	pend_point.z = first_pts.z;
	return pend_point;
}

inline POSE RectanglePointCal(POSE first_pts, POSE second_pts, POSE third_pts){
	POSE rect_point;
	double slope_1 = SlopeCal(first_pts, second_pts);
	double slope_3 = SlopeCal(second_pts, third_pts);
	double temp_1 = slope_1*first_pts.y - slope_3*third_pts.y;
	double temp_2 = first_pts.x - third_pts.x;
	rect_point.y = (temp_1+temp_2)/(slope_1-slope_3);
	rect_point.x = first_pts.x - slope_1*(rect_point.y - first_pts.y);
	rect_point.z = first_pts.z;
	return rect_point;
}

inline double RectangleOrienCal(POLY poly){
	double angle;
	double dist_pre = PointDistCal(poly.polygon.points[0], poly.polygon.points[1]);
	double dist_post = PointDistCal(poly.polygon.points[1], poly.polygon.points[2]);
	if (dist_pre >= dist_post)
		angle = atan(SlopeCal(poly.polygon.points[0], poly.polygon.points[1]));
	else
		angle = atan(SlopeCal(poly.polygon.points[1], poly.polygon.points[2]));
	if (angle < 0)
		angle += M_PI;
	return angle;
}

inline double InnerAngleCal(POSE point_pre, POSE point_curr, POSE point_post){
	double inner_angle;
	double dx_pre = point_curr.x - point_pre.x;
	double dy_pre = point_curr.y - point_pre.y;
	double dx_post = point_curr.x - point_post.x;
	double dy_post = point_curr.y - point_post.y;
	double m_pre = sqrt(dx_pre*dx_pre + dy_pre*dy_pre);
	double m_post = sqrt(dx_post*dx_post + dy_post*dy_post);
	inner_angle = acos((dx_pre*dx_post + dy_pre*dy_post)/(m_pre*m_post));
	return inner_angle*180/M_PI;
}

void ObstDetect::ParamReconfigure(obstacle_tracking::clusterCTRConfig &config, uint32_t level){
	cluster_tolerance = config.tolerate_value;
	min_cluster_size = config.min_cluster;
	max_cluster_size = config.max_cluster;
	moving_avg_size = config.moving_avg_size;
}

void ObstDetect::PointcloudCB(sensor_msgs::PointCloud2ConstPtr pc){
    sensor_msgs::PointCloud obs_pts;
    sensor_msgs::convertPointCloud2ToPointCloud(*pc, obs_pts);
    try{
        tf_.transformPointCloud(local_frame_, obs_pts, obs_pts);
    }
    catch (tf::TransformException &ex){
        ROS_DEBUG("Failure %s\n", ex.what()); //Print exception which was caught
        return;
    }
}

void ObstDetect::Laser1CB(sensor_msgs::LaserScanConstPtr scan_in){
	ROS_INFO("LaserScan");
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
    UpdateObst(laser1_pts);
}

void ObstDetect::Laser2CB(sensor_msgs::LaserScanConstPtr scan_in){
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
    laser2_pts = laser_cloud;
}

void ObstDetect::Laser3CB(sensor_msgs::LaserScanConstPtr scan_in){
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
    laser3_pts = laser_cloud;
}

//Update the obstacle pts from multiple sensors or vehicles
void ObstDetect::UpdateObst(sensor_msgs::PointCloud pc_in){
	//TODO:Implement single scan first, later integrate other scan source for cooperative perception
	obst_pts = pc_in;
	unprocessed_pts_pub.publish(obst_pts);
	ObstClustering(obst_pts);
}

//Extract the interesting pts for analysis
void ObstDetect::ObstClustering(sensor_msgs::PointCloud obst_pts){

	ROS_INFO("Obstacle Clustering");

	//Clear the data of previous laser scan
	obst_channel.values.clear();
	filtered_interest_pts.points.clear();
	filtered_interest_pts.channels.clear();
	veh_polys.clear();
	ped_poses.clear();
	markerarray.markers.clear();

	/******************************Using KDTree for Euclidean clusting*******************/
	pcl::search::KdTree<pcl::PointXYZ>::Ptr Kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr obst_pcl (new pcl::PointCloud<pcl::PointXYZ>);
	*obst_pcl = pointcloudToPCL(obst_pts);

	Kdtree->setInputCloud(obst_pcl);
	cout << "Obstacle points number"<<obst_pcl->points.size()<<endl;
	vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(cluster_tolerance);
	ec.setMinClusterSize(min_cluster_size);
	ec.setMaxClusterSize(max_cluster_size);
	ec.setSearchMethod(Kdtree);
	ec.setInputCloud(obst_pcl);
	ec.extract(cluster_indices);
	/*******************************************End****************************************/

	cout << cluster_indices.size()<<endl;
	ROS_INFO("Extract Clustering");
	int index = 0;
	sensor_msgs::PointCloud processed_pcl;
    sensor_msgs::ChannelFloat32 _channel;
    _channel.name = "distance";

	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++){
			cloud_cluster->points.push_back(obst_pcl->points[*pit]);
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
	    cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

	    sensor_msgs::PointCloud cluster_pts;
	    cluster_pts = PCLTopointcloud(*cloud_cluster);
	    ObstExtraction(cluster_pts);

	    for (int i = 0; i < cluster_pts.points.size(); i++){
	    	processed_pcl.points.push_back(cluster_pts.points[i]);
	    	_channel.values.push_back(5*index);
	    }
	    index++;
	}
    processed_pcl.channels.push_back(_channel);
	processed_pcl.header.frame_id = local_frame_;
	processed_pcl.header.stamp = ros::Time::now();
	processed_pts_pub.publish(processed_pcl);

	filtered_interest_pts.header.stamp = ros::Time::now();
	filtered_interest_pts.header.frame_id = local_frame_;
	filtered_interest_pts.channels.push_back(obst_channel);
	interest_pts_pub.publish(filtered_interest_pts);

	ObstPosePub(veh_polys, ped_poses);
}

void ObstDetect::ObstExtraction(sensor_msgs::PointCloud clustered_pcl){
	//Extract the interest point for analysis: End points & discontinuous point
	ROS_INFO("Analyze interest pts & Extract obst");

	/******************Extract interest points: Initial, end point and conners**************************/
	sensor_msgs::PointCloud interest_pts;

	interest_pts.points.push_back(clustered_pcl.points.front());
	for (int i = moving_avg_size; i < (clustered_pcl.points.size()-moving_avg_size); i++ ){
		POSE current_pts, pre_avg_pts, post_avg_pts;
		current_pts = clustered_pcl.points[i];

		//Calculate the average points to address the noise;
		for (int j = 1; j <= moving_avg_size; j++){
			pre_avg_pts.x += clustered_pcl.points[i-j].x;
			pre_avg_pts.y += clustered_pcl.points[i-j].y ;
			post_avg_pts.x += clustered_pcl.points[i+j].x;
			post_avg_pts.y += clustered_pcl.points[i+j].y;
		}
		pre_avg_pts.x = pre_avg_pts.x/moving_avg_size;
		pre_avg_pts.y = pre_avg_pts.y/moving_avg_size;
		post_avg_pts.x = post_avg_pts.x/moving_avg_size;
		post_avg_pts.y = post_avg_pts.y/moving_avg_size;

		double inner_angle = InnerAngleCal(pre_avg_pts, current_pts, post_avg_pts);
		if (inner_angle < 105 && inner_angle > 75 )
			interest_pts.points.push_back(current_pts);
	}
	interest_pts.points.push_back(clustered_pcl.points.back());
	/**********************************End**********************************/

	/***************Classification of Obstacles using interest points****************/
	//If the obst has three interest point(one corner), it will be high likehood to be vehicle

	if (interest_pts.points.size() == 3){
		double angle = InnerAngleCal(interest_pts.points[0], interest_pts.points[1], interest_pts.points[2] );
		double pre_dist = PointDistCal(interest_pts.points[0], interest_pts.points[1]);
		double post_dist = PointDistCal(interest_pts.points[1], interest_pts.points[2]);
		double min_dist = (pre_dist < post_dist) ? pre_dist : post_dist;
		double max_dist = (pre_dist > post_dist) ? pre_dist : post_dist;
		bool angle_crit = (angle < 105 && angle > 75);
		bool dist_crite = (1.5< max_dist && max_dist < 2.5) && (0.5< min_dist && min_dist <1.5);
		if (angle_crit && dist_crite){
		//Store the obstalce type channel for further interpretate, type 3 for obstacle with 3 interest points
			for (int i = 0; i < interest_pts.points.size(); i++){
				filtered_interest_pts.points.push_back(interest_pts.points[i]);
			obst_channel.values.push_back(3);
			}
			ObstInterpretate(interest_pts);
		}
	}

	//If the obst has two interest point, it will be more complicate to decide its type
	if (interest_pts.points.size() == 2){
		POSE first_pts = interest_pts.points[0];
		POSE second_pts = interest_pts.points[1];
		double pts_dist = PointDistCal(first_pts, second_pts);
		if (abs(SlopeCal(second_pts, first_pts)) < 0.6){
			//Front Vehicle feature, two keypoints and processed as retangle
			if (pts_dist < 2 && pts_dist > 0.8){
				for (int i = 0; i < interest_pts.points.size(); i++){
					obst_channel.values.push_back(2);
					filtered_interest_pts.points.push_back(interest_pts.points[i]);
				}
				ObstInterpretate(interest_pts);
			}
			//Pedestrian features, no need to interpret, processed as point obstacle
			if (pts_dist < 0.8){
				for (int i = 0; i < interest_pts.points.size(); i++){
					obst_channel.values.push_back(1);
					filtered_interest_pts.points.push_back(interest_pts.points[i]);
				}
				POSE ped_pose_;
				ped_pose_.x = (interest_pts.points[0].x+interest_pts.points[1].x)/2;
				ped_pose_.y = (interest_pts.points[0].y+interest_pts.points[1].y)/2;
				ped_pose_.z = 0;
				ped_poses.push_back(ped_pose_);
			}
		}
	}
	/********************************************End********************************/
}

//TODO: Interpret the interest points to "refill" the obstacle region
void ObstDetect::ObstInterpretate(sensor_msgs::PointCloud keypoints){
	//"Line Vehicle": Fit two more points
	interpreted_ploy.polygon.points.clear();
	interpreted_ploy.header.stamp = ros::Time::now();
	interpreted_ploy.header.frame_id = local_frame_;

	if (keypoints.points.size() == 2){
		interpreted_ploy.polygon.points.push_back(keypoints.points[0]);
		interpreted_ploy.polygon.points.push_back(keypoints.points[1]);
		double width = PointDistCal(keypoints.points[0], keypoints.points[1]);
		double height = 2.0 * width;
		POSE third_point = PendicularPointCal(keypoints.points[1], keypoints.points[0], height);
		interpreted_ploy.polygon.points.push_back(third_point);
		POSE fourth_point = PendicularPointCal(keypoints.points[0], keypoints.points[1], height);
		interpreted_ploy.polygon.points.push_back(fourth_point);
	}

	//"Triple points vehicle": Fit the fourth point
	if (keypoints.points.size()==3){
		for (int i = 0; i <3; i++)
			interpreted_ploy.polygon.points.push_back(keypoints.points[i]);
		POSE fourth_point = RectanglePointCal(keypoints.points[0], keypoints.points[1], keypoints.points[2]);
		interpreted_ploy.polygon.points.push_back(fourth_point);
	}
	veh_polys.push_back(interpreted_ploy);
	obst_polys_pub.publish(interpreted_ploy);
}

//For visualization only
void ObstDetect::ObstMarkerVisualize(sensor_msgs::PointCloud obst_pts){
	for (int i = 0; i < obst_pts.points.size(); i++){
		visualization_msgs::Marker marker;
		marker.id = i;
		marker.action = visualization_msgs::Marker::ADD;

		if (obst_pts.channels[0].values[i] == 0){
			marker.ns = "veh_obsts";

			//The modification of coords here make no sense, just to match the origin of the mesh model for visualization
			marker.pose.position.x =obst_pts.points[i].x ;
			marker.pose.position.y =obst_pts.points[i].y - 0.8 ;
			marker.pose.position.z = 0.25;

			double angle = obst_pts.channels[2].values[i] + M_PI;

			marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0.0,angle);

#if (0)
			marker.type = cube;
			marker.scale.x = 2.0; marker.scale.y = 1.0; marker.scale.z = 1.0;
#else if
			marker.type = mesh;
			marker.mesh_resource = "package://local_map/urdf/cool_car.STL";
			marker.scale.x = 0.011; marker.scale.y = 0.012; marker.scale.z = 0.012;
#endif
		}

		if (obst_pts.channels[0].values[i] == 1){
			marker.ns = "ped_obsts";

			marker.pose.position.x =obst_pts.points[i].x ;
			marker.pose.position.y =obst_pts.points[i].y ;
			marker.pose.position.z =0;
			double angle = obst_pts.channels[2].values[i] + M_PI;
			marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0.0,angle);

#if (0)
			marker.type = cube;
			marker.scale.x = 2.0; marker.scale.y = 1.0; marker.scale.z = 1.0;
#else if
			marker.type = mesh;
			marker.mesh_resource = "package://local_map/urdf/android.stl";
			marker.scale.x = 0.02; marker.scale.y = 0.02; marker.scale.z = 0.02;
#endif
		}
		marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f; marker.color.a = 1.0;
		marker.lifetime = ros::Duration();
		marker.header.frame_id = local_frame_;
		marker.header.stamp = ros::Time::now();
		markerarray.markers.push_back(marker);
	}
	obst_marker_pub.publish(markerarray);
}

void ObstDetect::ObstPosePub(vector<POLY> polys, vector<POSE> poses){
	sensor_msgs::PointCloud obst_pose;
	POSE pose_temp;
	sensor_msgs::ChannelFloat32 _type, _id, _orien;
	_type.name = "obst_type";
	_id.name = "obst_id";
	_orien.name = "obst_orien";
	for (int i =0; i < polys.size(); i++){
		pose_temp.x = (polys[i].polygon.points[0].x+polys[i].polygon.points[2].x)/2;
		pose_temp.y = (polys[i].polygon.points[0].y+polys[i].polygon.points[2].y)/2;
		pose_temp.z = 0;
		obst_pose.points.push_back(pose_temp);
		//Can not decide the exact orientation unless the heading avaiable.
		double orien = RectangleOrienCal(polys[i]);
		_type.values.push_back(0);
		_id.values.push_back(i);
		_orien.values.push_back(orien);
	}
	for (int i = 0; i < poses.size(); i++){
		obst_pose.points.push_back(poses[i]);
		_type.values.push_back(1);
		_id.values.push_back(i);
		_orien.values.push_back(0);
	}
	obst_pose.header.frame_id = local_frame_;
	obst_pose.header.stamp = ros::Time::now();
	obst_pose.channels.push_back(_type);
	obst_pose.channels.push_back(_id);
	obst_pose.channels.push_back(_orien);
	obst_pose_pub.publish(obst_pose);

	obstacles.veh_polys = polys;
	obstacles.ped_pts = poses;
	obst_info_pub.publish(obstacles);

	ObstMarkerVisualize(obst_pose);
}

int main(int argc, char**argv){
	ros::init(argc, argv,"obst_detect");
	ObstDetect obstdetect;
	ros::spin();
}
