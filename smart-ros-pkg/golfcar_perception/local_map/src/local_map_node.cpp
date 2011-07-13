#include <local_map.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>

using namespace std;
struct Point
{
    float x[3];
};
class local_map_node{
		
		public:
		
		local_map_node();
		~local_map_node();
		
		private:
		local_map l_map_;
		ros::NodeHandle n_;
		ros::Subscriber odom_sub_;
		int odom_skip_;
		int odom_count_;
		Point odom_pre_;
		void odomCallBack(const nav_msgs::Odometry::ConstPtr &odom_in);
		void on_sick(const sensor_msgs::LaserScan::ConstPtr &msg);
		tf::TransformListener tf_;
		tf::MessageFilter<sensor_msgs::LaserScan> *tf_filter_;
		message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
		
};

// vehicle location is always (0,0)
local_map_node::local_map_node()
{

	//l_map_.map_init();
	odom_sub_ = n_.subscribe("odom",1, &local_map_node::odomCallBack, this);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, tf_, "base_link", 100);
	tf_filter_->registerCallback(boost::bind(&local_map_node::on_sick, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.05));
	odom_skip_ = 10;
	odom_count_ = 0;
}

local_map_node::~local_map_node()
{
	l_map_.map_free();
}
void local_map_node::odomCallBack(const nav_msgs::Odometry::ConstPtr &odom_in)
{

	if(odom_count_%odom_skip_)
	{
		Point odom_now;

		odom_now.x[0] = odom_in->pose.pose.position.x;
		odom_now.x[1] = odom_in->pose.pose.position.y;
		odom_now.x[2] = tf::getYaw(odom_in->pose.pose.orientation);

		//call function to update the local map


		//then
		odom_pre_ = odom_now;
	}
	odom_count_++;

}

void local_map_node::on_sick(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
    cout<<"got message with: " << scan_in->header.frame_id << endl;

    sensor_msgs::PointCloud cloud;
	laser_geometry::LaserProjection projector;

	try
	{
		projector.transformLaserScanToPointCloud("odom",*scan_in, cloud,tf_);
	}
	catch (tf::TransformException& e)
	{
		std::cout << e.what();
		return;

	}
	vector<Point> points;
	points.resize(cloud.points.size());
	for(unsigned int i=0; i<cloud.points.size(); i++)
	{
		points[i].x[0]=cloud.points[i].x;
		points[i].x[1]=cloud.points[i].y;
		points[i].x[2]=cloud.points[i].z;
	}
	l_map_.process_points(points);
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_map");
	ros::NodeHandle n;
	local_map_node lmn;

    ros::spin();

}

