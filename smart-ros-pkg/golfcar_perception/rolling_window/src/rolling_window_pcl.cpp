#include "rolling_window_pcl.h"

namespace golfcar_pcl{
	
	rolling_window_pcl::rolling_window_pcl():
    private_nh_("~")
    {
		target_frame_ = "odom";
		base_frame_ = "base_link";
		
		tf_ = new tf::TransformListener();

		odom_init_				= 	false;
		scan_init_				= 	false;
		cloud_init_				= 	false;
		
		scan_in_thresh_		=	0.02;
		cloud_in_thresh_		=	0.02;
		
		private_nh_.param("front_bound", front_bound_, 11.0);
		private_nh_.param("back_bound", back_bound_, -3.0);
		private_nh_.param("odom_trigger_thresh", odom_trigger_thresh_, 1.0);
		
		laser_scan_sub_.subscribe(nh_, "scan_in", 10);
		laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, target_frame_, 10);
		laser_scan_filter_->registerCallback(boost::bind(&rolling_window_pcl::scanCallback, this, _1));
		laser_scan_filter_->setTolerance(ros::Duration(0.05));

		cloud_scan_sub_.subscribe(nh_, "cloud_in", 10);
		cloud_scan_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_scan_sub_, *tf_, target_frame_, 10);
		cloud_scan_filter_->registerCallback(boost::bind(&rolling_window_pcl::cloudCallback, this, _1));
		cloud_scan_filter_->setTolerance(ros::Duration(0.05));
		
		//be careful to the frame_id of "odom";
		odom_sub_.subscribe(nh_, "odom", 10);
		odom_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, *tf_, base_frame_, 10);
		odom_filter_ ->registerCallback(boost::bind(&rolling_window_pcl::odomCallback, this, _1));
		odom_filter_->setTolerance(ros::Duration(0.05));

		rolling_window_pub_ = nh_.advertise<PointCloud>("rolling_window_pcl", 10);
	}
	
	rolling_window_pcl::~rolling_window_pcl()
	{
		delete tf_;
	}
	

    void rolling_window_pcl::scanCallback(const sensor_msgs::LaserScanConstPtr scan_in)
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
			bool move_flag = rolling_window_pcl::checkDistance(scan_OdomMeas_, laserOdomTemp, scan_in_thresh_);
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
	
    void rolling_window_pcl::cloudCallback(const sensor_msgs::PointCloud2ConstPtr cloud_in)
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
			bool move_flag = rolling_window_pcl::checkDistance(cloud_OdomMeas_, cloudOdomTemp, cloud_in_thresh_);
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
	
	bool rolling_window_pcl::checkDistance(const tf::StampedTransform& oldTf, const tf::StampedTransform& newTf, float Dis_thresh)
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
	
	void rolling_window_pcl::odomCallback(const OdomConstPtr& odom)
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
			bool move_flag = rolling_window_pcl::checkDistance(odom_OdomMeas_, baseOdomTemp, odom_trigger_thresh_);
			if(move_flag) 
			{
				rolling_window_pcl::windowProcessing(odom_time);
				odom_OdomMeas_ = baseOdomTemp;
			}
		}
	}
	
	void rolling_window_pcl::windowProcessing(ros::Time current_time)
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
	}
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "rolling_window_node");
	 ros::NodeHandle n;
	 golfcar_pcl::rolling_window_pcl rolling_window_node;
     ros::spin();
     return 0;
}
