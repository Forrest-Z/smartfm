#include "rolling_window_pcl.h"

namespace golfcar_pcl{
	
	rolling_window_pcl::rolling_window_pcl():
    private_nh_("~")
    {
		odom_frame_ = "odom";
		base_frame_ = "base_link";
		
		tf_ = new tf::TransformListener();

		odom_init_				= 	false;
		scan_init_				= 	false;
		cloud_init_				= 	false;
		
		process_fraction_exist_ =  false;
		
		scan_in_thresh_		=	0.02;
		cloud_in_thresh_		=	0.02;
		
		rolling_window_baselink_.height  			= 	1;
		rolling_window_baselink_.header.frame_id	=	base_frame_;
		rolling_window_odom_.height  					=  1;
		rolling_window_odom_.header.frame_id		=	odom_frame_;
		process_fraction_odom_.height 				=  1;
		process_fraction_odom_.header.frame_id		=	odom_frame_;
		front_buffer_odom_.height 		   			=  1;
		front_buffer_odom_.header.frame_id			=	odom_frame_;
		
		private_nh_.param("front_bound", front_bound_, 11.0);
		private_nh_.param("back_bound", back_bound_, -3.0);
		private_nh_.param("odom_trigger_thresh", odom_trigger_thresh_, 1.0);
		
		laser_scan_sub_.subscribe(nh_, "scan_in", 10);
		laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, odom_frame_, 10);
		laser_scan_filter_->registerCallback(boost::bind(&rolling_window_pcl::scanCallback, this, _1));
		laser_scan_filter_->setTolerance(ros::Duration(0.05));

		//be careful to the frame_id of "odom";
		odom_sub_.subscribe(nh_, "odom", 10);
		odom_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, *tf_, base_frame_, 10);
		odom_filter_ ->registerCallback(boost::bind(&rolling_window_pcl::odomCallback, this, _1));
		odom_filter_->setTolerance(ros::Duration(0.05));

		rolling_window_pub_   = nh_.advertise<RollingPointCloud>("rolling_window_pcl", 10);
		process_fraction_pub_ = nh_.advertise<rolling_window::pcl_indices>("process_fraction_indices", 10);

		float pitch_para1 = 0.1;
		float pitch_para2 = 0.01;
		float roll_para   = 0.02;
		ms_imu_.initialize(pitch_para1, pitch_para2, roll_para);

		float laser_x = 1.5;
		float laser_z = 1.7;
		float laser_pitch = 0.21;
		golfcar_pcl::imu_info *imu_tmp = &ms_imu_;
		rolling_analyzer_.initialize(imu_tmp, laser_x, laser_z, laser_pitch);

		imu_sub_ = nh_.subscribe("/ms/imu/data", 100, &rolling_window_pcl::imuCallback, this);
		//laser_scan_serial_ = 0;
	}
	
	rolling_window_pcl::~rolling_window_pcl()
	{
		delete tf_;
	}

	void rolling_window_pcl::imuCallback(const sensor_msgs::Imu::ConstPtr msg)
	{
	    btScalar pitch, roll, yaw;
	    geometry_msgs::Quaternion orientation = msg->orientation;
	    btQuaternion btq(orientation.x, orientation.y, orientation.z, orientation.w);
	    btMatrix3x3(btq).getEulerYPR(yaw, pitch, roll);
	    ms_imu_.update(msg->angular_velocity.y, pitch, roll);
	}
	
    void rolling_window_pcl::scanCallback(const sensor_msgs::LaserScanConstPtr scan_in)
    {
		bool use_laser_input = false;
		tf::StampedTransform laserOdomTemp;
		ros::Time laserTime = scan_in->header.stamp;
		string laserFrame 	= scan_in->header.frame_id;

		try
		{
			tf_->lookupTransform(odom_frame_, laserFrame, laserTime, laserOdomTemp);
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
			RollingPointXYZ point_tmp;
			RollingPointCloud cloud_tmp;
			cloud_tmp.header = scan_in->header;

			for(size_t i=0; i< scan_in->ranges.size(); i++)
			{
				float angle_tmp = scan_in->angle_min + scan_in->angle_increment*i;
				if(scan_in->ranges[i] > scan_in->range_max)
				{
					continue;
				}
				else
				{
					point_tmp.x = scan_in->ranges[i]*cos(angle_tmp);
					point_tmp.y = scan_in->ranges[i]*sin(angle_tmp);
					point_tmp.z = 0.0;

					point_tmp.laser_serial = scan_in->header.seq;
					point_tmp.pitch_speed = rolling_analyzer_.imu_pointer->pitch_speed;
					point_tmp.pitch = rolling_analyzer_.imu_pointer->pitch;
					point_tmp.roll = rolling_analyzer_.imu_pointer->roll;
					point_tmp.beam_angle = angle_tmp;
					//these variance are calculated in the "vertical_baselink" frame;
					point_tmp.x_var      = 0.0;
					point_tmp.y_var      = 0.0;
					point_tmp.z_var      = rolling_analyzer_.z_var(scan_in->ranges[i], angle_tmp);
					cloud_tmp.points.push_back(point_tmp);
				}
			}
			pcl_ros::transformPointCloud(odom_frame_, cloud_tmp, cloud_tmp, *tf_ );
			front_buffer_odom_ = front_buffer_odom_ + cloud_tmp;
		}
		//laser_scan_serial_++;
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
			//if use "odom_time", some warn information will appear saying "cache empty", 
			//maybe due to the fast frequency of this odom callback;
			
			tf_->lookupTransform(odom_frame_, base_frame_, odom_time, baseOdomTemp);
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
	
	//brief documentation:
	//1. "rolling_window_odom_" maintains the pointcloud in the odom frame, as the history;
	//    when odom_trigger, it erases the obsolete points outside the bounding box;
	//2. "front_buffer_odom_" accumulates the newest input until the odom_trigger is on;
	//3. "process_fraction_odom_" is added to "rolling_window_odom" (new) with its pcl indices recorded; 
	//	   then it takes the value of "front_buffer_odom_", waiting for another trigger;
	//4. "rolling_window_baselink" just publish all the data, including "rolling_window_odom" (new) and "front_buffer_odom_";
	
	void rolling_window_pcl::windowProcessing(ros::Time current_time)
	{
		rolling_window_baselink_.clear();
		//every each clear() operation,the height will be randomly set; must be set back to "1";
		rolling_window_baselink_.height  			= 	1;		
		rolling_window_baselink_.header.stamp 		=	current_time;
		rolling_window_odom_.header.stamp 			=	current_time;
		process_fraction_odom_.header.stamp 		=	current_time;
		front_buffer_odom_.header.stamp 			=	current_time;
		
		// downsample the raw pcl;
		//pcl_downsample(front_buffer_odom_);

		RollingPointCloud windowXYZ_tmp;
		pcl_ros::transformPointCloud(base_frame_, rolling_window_odom_, windowXYZ_tmp, *tf_ );

		// to erase obsolete points outside of the bounding box;
		for(size_t i=0; i<windowXYZ_tmp.points.size();i++)
		{
          if(windowXYZ_tmp.points[i].x<front_bound_ && windowXYZ_tmp.points[i].x>back_bound_)
          {
              rolling_window_baselink_.points.push_back(windowXYZ_tmp.points[i]);
			     rolling_window_baselink_.width ++;
          }
		}
		//record the remaining pcl in "odom" frame;
		pcl_ros::transformPointCloud(odom_frame_, rolling_window_baselink_, rolling_window_odom_, *tf_ );
		
		if(process_fraction_exist_)
		{
			rolling_window::pcl_indices proc_indices;
			proc_indices.header.stamp    =	current_time;
			proc_indices.header.frame_id =	base_frame_;
			size_t proc_begin_index  = rolling_window_baselink_.points.size();
			size_t proc_indices_size = process_fraction_odom_.points.size();
			for(size_t i = 0; i < proc_indices_size ; i++) proc_indices.indices.push_back(i+proc_begin_index);
			process_fraction_pub_.publish(proc_indices);
			
			RollingPointCloud windowXYZ_proc_tmp, windowXYZ_buff_tmp;
			pcl_ros::transformPointCloud(base_frame_, process_fraction_odom_, windowXYZ_proc_tmp, *tf_ );
			pcl_ros::transformPointCloud(base_frame_, front_buffer_odom_, windowXYZ_buff_tmp, *tf_ );
			rolling_window_baselink_ = rolling_window_baselink_  + windowXYZ_proc_tmp;
			rolling_window_baselink_ = rolling_window_baselink_  + windowXYZ_buff_tmp;
			
			rolling_window_pub_.publish(rolling_window_baselink_);
			
			rolling_window_odom_ 	= rolling_window_odom_ + process_fraction_odom_;
			process_fraction_odom_ 	= front_buffer_odom_;
			front_buffer_odom_.points.clear();	
		}
		else
		{
			process_fraction_exist_ = true;
			//process_fraction_odom_  = front_buffer_odom_;
			return;
		} 
	}
	
	//pay attention to this down-sampling process: it may lead to unexpected sparsity;
	void rolling_window_pcl::pcl_downsample(RollingPointCloud &point_cloud)
	{
		pcl::VoxelGrid<RollingPointXYZ> sor;

		// always good not to use in place filtering as stated in
		// http://www.pcl-users.org/strange-effect-of-the-downsampling-td3857829.html
		RollingPointCloud::Ptr input_msg_filtered (new RollingPointCloud ());
		float downsample_size_ = 0.05;
		sor.setInputCloud(point_cloud.makeShared());
		sor.setLeafSize (downsample_size_, downsample_size_, downsample_size_);
		sor.filter (*input_msg_filtered);
		point_cloud = * input_msg_filtered;
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
