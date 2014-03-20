#include "grid_utils_mapper.h"

namespace golfcar_perception{

	grid_utils_mapper::grid_utils_mapper():private_nh_("~")
	{
		private_nh_.param("laser_frame_id",     laser_frame_id_,    std::string("golfcart/front_bottom_lidar"));
		private_nh_.param("map_frame_id",      	map_frame_id_,      std::string("golfcart/map"));

		laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "front_bottom_scan", 100);
		tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_, map_frame_id_, 100);
		tf_filter_->registerCallback(boost::bind(&grid_utils_mapper::scanCallback, this, _1));
		tf_filter_->setTolerance(ros::Duration(0.1));

		//specify the map details: the obstacle layer should choose the same origin and orientation as the localization layer;
		double 	origin_x_, origin_y_, origin_yaw_;
		int		sizeX, sizeY;
		double 	resolution_;
		private_nh_.param("sizeX",      	sizeX,     		5000);
		private_nh_.param("sizeY",      	sizeY,     		5000);
		private_nh_.param("resolution",		resolution_,	0.1);

		double min_occ, max_range, min_pass_through;
		private_nh_.param("min_occ",      		min_occ,     		0.1);
		private_nh_.param("max_range",      	max_range,     		40.0);
		private_nh_.param("min_pass_through",   min_pass_through,   2.0);

		origin_x_ = 0.0; origin_y_ = 0.0; origin_yaw_ = 0.0;
		info_.origin.position.x = origin_x_;
		info_.origin.position.y = origin_y_;
		info_.origin.orientation = tf::createQuaternionMsgFromYaw(origin_yaw_);
		info_.resolution = resolution_;
		info_.width = sizeX;
		info_.height = sizeY;

	    nm::OccupancyGrid fake_grid;
	    fake_grid.info = info_;
		overlay_ = gu::createCloudOverlay(fake_grid, map_frame_id_, min_occ, max_range, min_pass_through);
		grid_pub_ = nh_.advertise<nm::OccupancyGrid>("local_grid", 1);
	}

	void grid_utils_mapper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		CloudBuffer clouds;
		// Figure out current sensor position
	    tf::Pose identity(tf::createIdentityQuaternion(), btVector3(0, 0, 0));
	    tf::Stamped<tf::Pose> map_pose;
	    tf_.transformPose(map_frame_id_, tf::Stamped<tf::Pose> (identity, ros::Time(), laser_frame_id_), map_pose);

	    // Project scan from sensor frame (which varies over time) to odom (which doesn't)
	    sm::PointCloud fixed_frame_cloud;
		try{projector_.transformLaserScanToPointCloud(map_frame_id_, *scan_in, fixed_frame_cloud, tf_);}
		catch (tf::TransformException& e){ROS_INFO("Cannot Transform Into Global Frame"); std::cout << e.what(); return;}

        CloudPtr localized_cloud(new occupancy_grid_utils::LocalizedCloud());
        localized_cloud->cloud.points = fixed_frame_cloud.points;
        tf::poseTFToMsg(map_pose, localized_cloud->sensor_pose);
        localized_cloud->header.frame_id = map_frame_id_;
        last_cloud_ = localized_cloud;
        clouds.push_back(last_cloud_);
        last_cloud_.reset();

        BOOST_FOREACH  (CloudConstPtr cloud, clouds) gu::addCloud(&overlay_, cloud);

        nm::OccupancyGrid::ConstPtr grid = gu::getGrid(overlay_);
        ROS_INFO ("Done building grid");
        grid_pub_.publish(grid);
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "grid_utils_mapper");
	ros::NodeHandle n;
	golfcar_perception::grid_utils_mapper grid_utils_mapper_node;
	ros::spin();
	return 0;
}
