#include "rolling_window.h"

namespace golfcar_pcl{
	
	rolling_window::rolling_window():
    private_nh_("~")
    {
		target_frame_ = "odom";
		base_frame_ = "base_link";
		
		tf_ = new tf::TransformListener();
		
		odom_init_				= 	false;
		scan_init_				= 	false;
		cloud_init_				= 	false;
		
		front_bound_  			= 	11.0;//was 12
		back_bound_	  			= 	-3.0;//was -4
		odom_trigger_thresh_ 	= 	0.30;
		scan_in_thresh_			=	0.02;
		cloud_in_thresh_		=	0.02;
		
		laser_scan_sub_.subscribe(nh_, "scan_in", 10);
		laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, target_frame_, 10);
		laser_scan_filter_->registerCallback(boost::bind(&rolling_window::scanCallback, this, _1));
		laser_scan_filter_->setTolerance(ros::Duration(0.05));

		cloud_scan_sub_.subscribe(nh_, "cloud_in", 10);
		cloud_scan_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_scan_sub_, *tf_, target_frame_, 10);
		cloud_scan_filter_->registerCallback(boost::bind(&rolling_window::cloudCallback, this, _1));
		cloud_scan_filter_->setTolerance(ros::Duration(0.05));
		
		//be careful to the frame_id of "odom";
		odom_sub_.subscribe(nh_, "odom", 10);
		odom_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, *tf_, base_frame_, 10);
		odom_filter_ ->registerCallback(boost::bind(&rolling_window::odomCallback, this, _1));
		odom_filter_->setTolerance(ros::Duration(0.05));

		rolling_window_pub_ = nh_.advertise<PointCloud>("rolling_window_pcl", 10);
		
		//new_function1(NF1);
		road_surface_pub_   = nh_.advertise<PointCloud>("road_surface_plane", 10);
		
		//new_function2(NF2);
		viewpoint_td_sick_.x = 1.70;
		viewpoint_td_sick_.y = 0.00;
		viewpoint_td_sick_.z = 1.53;
		normals_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("normals_array", 100);
		
		window_counts_ = 0;
	}
	
	rolling_window::~rolling_window(){}
	

    void rolling_window::scanCallback(const sensor_msgs::LaserScanConstPtr scan_in)
    {
		bool use_laser_input = false;
		tf::StampedTransform laserOdomTemp;
		ros::Time laserTime = scan_in->header.stamp;
		string laserFrame 	= scan_in->header.frame_id;
		try
		{
			tf_->lookupTransform(target_frame_, laserFrame, laserTime, laserOdomTemp);
		}
		catch(tf::TransformException e)
		{
			ROS_WARN("scan Failed to get fresh tf between target_frame and scan_source, (%s)", e.what()); return;
		}
		
		//only after odom is initialized, the input will be processed;
		if(odom_init_ && !scan_init_)
		{
			scan_init_ = true;
			use_laser_input = true;
			scan_OdomMeas_ = laserOdomTemp;
		}
		else if(odom_init_ && scan_init_)
		{
			bool move_flag = rolling_window::checkDistance(scan_OdomMeas_, laserOdomTemp, scan_in_thresh_);
			if(move_flag)
			{
				use_laser_input = true;
				//to record last updating position;
				scan_OdomMeas_ = laserOdomTemp;
			}
		}
		
		if(use_laser_input)
		{
			sensor_msgs::PointCloud2 laser_cloud;
			PointCloud cloud_tmp;
			//"projector_" support "pointcloud2";
			try{projector_.transformLaserScanToPointCloud(target_frame_, *scan_in, laser_cloud, *tf_);}
			catch (tf::TransformException& e){ ROS_ERROR("%s",e.what());return;}
			//this function may be deprecated and removed in the future version; remember this;
			pcl::fromROSMsg(laser_cloud, cloud_tmp);
			rolling_window_odom_ = rolling_window_odom_ + cloud_tmp;
		}
	}
	
    void rolling_window::cloudCallback(const sensor_msgs::PointCloud2ConstPtr cloud_in)
    {
		bool use_cloud_input = false;
		tf::StampedTransform cloudOdomTemp;
		ros::Time cloudTime = cloud_in->header.stamp;
		string cloudFrame 	= cloud_in->header.frame_id;
		try
		{
			tf_->lookupTransform(target_frame_, cloudFrame, cloudTime, cloudOdomTemp);
		}
		catch(tf::TransformException e)
		{
			ROS_WARN("cloud Failed to get fresh tf between target_frame and scan_source, (%s)", e.what()); return;
		}
		
		//only after odom is initialized, the input will be processed;
		if(odom_init_ && !cloud_init_)
		{
			cloud_init_ = true;
			use_cloud_input = true;
			cloud_OdomMeas_ = cloudOdomTemp;
		}
		else if(odom_init_ && cloud_init_)
		{
			bool move_flag = rolling_window::checkDistance(cloud_OdomMeas_, cloudOdomTemp, cloud_in_thresh_);
			if(move_flag)
			{
				use_cloud_input = true;
				cloud_OdomMeas_ = cloudOdomTemp;
			}
		}
		
		if(use_cloud_input)
		{
			//to record last updating position;
			sensor_msgs::PointCloud cloud_cloud;
			sensor_msgs::PointCloud2 cloud_cloud2;
			PointCloud cloud_tmp;
			
			sensor_msgs::convertPointCloud2ToPointCloud(*cloud_in, cloud_cloud);
			try{tf_->transformPointCloud(target_frame_, cloud_cloud, cloud_cloud);}
			catch (tf::TransformException& e){ ROS_ERROR("%s",e.what());return;}
			sensor_msgs::convertPointCloudToPointCloud2(cloud_cloud, cloud_cloud2);
			pcl::fromROSMsg(cloud_cloud2, cloud_tmp);
			rolling_window_odom_ = rolling_window_odom_ + cloud_tmp;
		}
	}
	
	bool rolling_window::checkDistance(const tf::StampedTransform& oldTf, const tf::StampedTransform& newTf, float Dis_thresh)
	{
		//"odom_old_new" denotes the tf of  "baselink_new" coordinate inside the "baselink_old" coordinate;
		tf::Transform odom_old_new  = oldTf.inverse() * newTf;
		float tx, ty;
		tx = -odom_old_new.getOrigin().y();
		ty =  odom_old_new.getOrigin().x();
		float mov_dis = sqrtf(tx*tx + ty*ty);
		//double yaw_dis, ttemp;
		//odom_old_new.getBasis().getEulerYPR(yaw_dis, ttemp, ttemp);
		if(mov_dis > Dis_thresh) return true;
		else return false;
	}
	
	void rolling_window::odomCallback(const OdomConstPtr& odom)
	{
		ros::Time odom_time = odom->header.stamp;
		tf::StampedTransform baseOdomTemp;
		try
		{
			//pay attention to the "ros::Time(0)", which use the latest available tf;
			//if use "odom_time", some warn information will appear saying "cache empty", maybe due to the fast frequency of this odom callback;
			tf_->lookupTransform(target_frame_, base_frame_, ros::Time(0), baseOdomTemp);
		}
		catch(tf::TransformException e)
		{
			ROS_WARN("odom Failed to get fresh tf between target_frame and scan_source, (%s)", e.what()); return;
		}
		
		if(!odom_init_)
		{
			odom_init_ = true;
			odom_OdomMeas_ = baseOdomTemp;
		}
		else
		{
			bool move_flag = rolling_window::checkDistance(odom_OdomMeas_, baseOdomTemp, odom_trigger_thresh_);
			if(move_flag) 
			{
				rolling_window::windowProcessing(odom_time);
				odom_OdomMeas_ = baseOdomTemp;
			}
		}
	}
	
	void rolling_window::windowProcessing(ros::Time current_time)
	{
		rolling_window_odom_.header.stamp 		=	current_time;
		rolling_window_odom_.header.frame_id 	=	target_frame_;
		
		//from "odom" frame to current "base_link" frame;
		sensor_msgs::PointCloud  window_tmp;
		sensor_msgs::PointCloud2 window_tmp2;
		PointCloud windowXYZ_tmp;
		PointCloud patch_ROI;
		
		pcl::toROSMsg(rolling_window_odom_, window_tmp2);
		sensor_msgs::convertPointCloud2ToPointCloud(window_tmp2, window_tmp);
		//rolling_window_odom_.clear();
		
		try
		{
			tf_->transformPointCloud(base_frame_, window_tmp, window_tmp);
			sensor_msgs::convertPointCloudToPointCloud2(window_tmp, window_tmp2);
			pcl::fromROSMsg(window_tmp2, windowXYZ_tmp);
		}
		catch (tf::TransformException &ex)
		{
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
			return;
		}

		rolling_window_baselink_.clear();
		rolling_window_baselink_.height = 1;
		rolling_window_baselink_.header = windowXYZ_tmp.header;

		patch_ROI.clear();
		patch_ROI.height = 1;
		patch_ROI.header.stamp 	=	current_time;
		patch_ROI.header.frame_id 	=	base_frame_;
		
		for(size_t i=0; i<windowXYZ_tmp.points.size();i++)
		{
          if(windowXYZ_tmp.points[i].x<front_bound_ && windowXYZ_tmp.points[i].x>back_bound_)
          {
              rolling_window_baselink_.points.push_back(windowXYZ_tmp.points[i]);
			     rolling_window_baselink_.width ++;
          }
          
          if(windowXYZ_tmp.points[i].x<10.0 && windowXYZ_tmp.points[i].x>6.0)
          {
              patch_ROI.points.push_back(windowXYZ_tmp.points[i]);
			     patch_ROI.width ++;
          }
		}
		
		//remember to also transfer the pcl back to "odom" frame, for accumulation in the next round;
		pcl::toROSMsg(rolling_window_baselink_, window_tmp2);
		sensor_msgs::convertPointCloud2ToPointCloud(window_tmp2, window_tmp);
		try
		{
			tf_->transformPointCloud(target_frame_, window_tmp, window_tmp);
			sensor_msgs::convertPointCloudToPointCloud2(window_tmp, window_tmp2);
			pcl::fromROSMsg(window_tmp2, rolling_window_odom_);
		}
		catch (tf::TransformException &ex)
		{
			printf ("Failure %s\n", ex.what());
			return;
		}
		
		//simple extension that will ensure only a single scan is used for each windows
		//Initial test failed. There are too little overlap for gmapping to work properly
		//window_counts_++;
		//if(window_counts_< 10) return;
		//rolling_window_odom_.clear();

		pcl::VoxelGrid<pcl::PointXYZ> sor;
		// always good not to use in place filtering as stated in
		// http://www.pcl-users.org/strange-effect-of-the-downsampling-td3857829.html
		PointCloud::Ptr input_msg_filtered (new PointCloud ());
		float downsample_size_ = 0.05;
		sor.setInputCloud(rolling_window_baselink_.makeShared());
		sor.setLeafSize (downsample_size_, downsample_size_, downsample_size_);
		sor.filter (*input_msg_filtered);

		rolling_window_pub_.publish(*input_msg_filtered);
		window_counts_ = 0;

		//----------NF3--------to focus on a fraction of the road surface-------------------------
		//PointCloud patch_ROI;
		
		
		//----------NF1--------just to test the plane estimation;---------------------------------
		//http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
		
	   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.05);
		seg.setInputCloud (patch_ROI.makeShared ());
		seg.segment (*inliers, *coefficients);
		
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.header = rolling_window_baselink_.header;
		// Fill in the cloud data
		cloud.width  = inliers->indices.size ();
		cloud.height = 1;
		cloud.points.resize (cloud.width * cloud.height);
		size_t j=0;
		for (size_t i = 0; i < inliers->indices.size (); ++i)
		{
			cloud.points[j]=patch_ROI.points[inliers->indices[i]];
			j++;
		}
		road_surface_pub_.publish(cloud);
		
		
		/*
		//----------NF2----------Calculate the norm including curvature of surfaces;--------------
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (rolling_window_baselink_.makeShared());

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		ne.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal> cloud_normals;

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch (0.3);
		ne.setViewPoint(viewpoint_td_sick_.x,viewpoint_td_sick_.y,viewpoint_td_sick_.z);
		// Compute the features
		ne.compute (cloud_normals);
		
		// concatentate the fileds
		pcl::PointCloud<pcl::PointNormal> point_normals;
		pcl::concatenateFields(rolling_window_baselink_, cloud_normals, point_normals);
		//cout<<"Point normal "<<point_normals.points[0].normal_x<<' '<<point_normals.points[0].normal_y<<' '<<point_normals.points[0].normal_z<<endl;
		
		// publish normal using visualization marker
		publishNormal(point_normals);
		*/
		 
	}
	
	
	
	void rolling_window::publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud)
	{
		pcl::PointCloud<pcl::PointNormal> edge_points;
		edge_points.header = pcl_cloud.header;
		edge_points.width = 1;

		for(size_t i=0; i<pcl_cloud.points.size();i++)
		{	
			if(fabs(pcl_cloud.points[i].curvature)>0.01)
			{
				edge_points.points.push_back(pcl_cloud.points[i]);
			}
			//cout<<"Point normal "<<pcl_cloud.points[i].curvature<<endl;
		}
		edge_points.height = edge_points.points.size();

		// publish normal as posearray for visualization
		bool publish_normals = true;
		if(publish_normals)
		{
			geometry_msgs::PoseArray normals_poses;
			normals_poses.header = edge_points.header;
			for(unsigned int i=0; i<edge_points.points.size(); i++)
			{
				geometry_msgs::Pose normals_pose;
				geometry_msgs::Point pos;
				pos.x = edge_points.points[i].x; pos.y = edge_points.points[i].y; pos.z = edge_points.points[i].z;
				normals_pose.position = pos;
				btVector3 axis(edge_points.points[i].normal[0],edge_points.points[i].normal[1],edge_points.points[i].normal[2]);
				if(isnan(edge_points.points[i].normal[0])||isnan(edge_points.points[i].normal[1])||isnan(edge_points.points[i].normal[2])) continue;
				btVector3 marker_axis(1, 0, 0);
				btQuaternion qt(marker_axis.cross(axis.normalize()), marker_axis.angle(axis.normalize()));
				double yaw, pitch, roll;
				btMatrix3x3(qt).getEulerYPR(yaw, pitch, roll);
				geometry_msgs::Quaternion quat_msg;
				tf::quaternionTFToMsg(qt, quat_msg);
				if(isnan(qt.x())||isnan(qt.y())||isnan(qt.z())||isnan(qt.w())) continue;
				normals_pose.orientation.x = qt.x();// = quat_msg;
				normals_pose.orientation.y = qt.y();
				normals_pose.orientation.z = qt.z();
				normals_pose.orientation.w = qt.w();

				normals_poses.poses.push_back(normals_pose);
			}
			normals_poses_pub_.publish(normals_poses);
		}
	}
	
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "rolling_window");
	 ros::NodeHandle n;
	 golfcar_pcl::rolling_window rolling_window_node;
     ros::spin();
     return 0;
}
