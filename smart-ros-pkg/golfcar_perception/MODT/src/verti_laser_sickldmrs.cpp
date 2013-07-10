#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <laser_geometry/laser_geometry.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <cmath>

using namespace std;
using namespace ros;

class verti_laser_sickldmrs
{

public:
	verti_laser_sickldmrs();

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	std::string												verti_laser_frame_id_;
	laser_geometry::LaserProjection                         projector_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub0_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub1_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub2_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub3_;

	message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan> *sync_;
	void scanProcess (const sensor_msgs::LaserScan::ConstPtr& scan_in0, const sensor_msgs::LaserScan::ConstPtr& scan_in1,
					  const sensor_msgs::LaserScan::ConstPtr& scan_in2, const sensor_msgs::LaserScan::ConstPtr& scan_in3);

	double 													verti_angle_threshold_;
	ros::Publisher 											vertical_laser_pub_;
};

verti_laser_sickldmrs::verti_laser_sickldmrs()
: private_nh_("~")
{
	private_nh_.param("verti_laser_frame_id",      verti_laser_frame_id_,     std::string("/ldmrs"));
	private_nh_.param("verti_angle_threshold",     verti_angle_threshold_,    M_PI/4.0);

	vertical_laser_pub_  =   nh_.advertise<sensor_msgs::LaserScan>("/sickldmrs/verti_laser", 2);
	laser_sub0_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/sickldmrs/scan0", 100);
	laser_sub1_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/sickldmrs/scan1", 100);
	laser_sub2_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/sickldmrs/scan2", 100);
	laser_sub3_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/sickldmrs/scan3", 100);
	sync_	= new message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan>(*laser_sub0_, *laser_sub1_, *laser_sub2_, *laser_sub3_, 10);
	sync_->registerCallback(boost::bind(&verti_laser_sickldmrs::scanProcess, this, _1, _2, _3, _4));
}


void verti_laser_sickldmrs::scanProcess (const sensor_msgs::LaserScan::ConstPtr& scan_in0, const sensor_msgs::LaserScan::ConstPtr& scan_in1,  const sensor_msgs::LaserScan::ConstPtr& scan_in2,  const sensor_msgs::LaserScan::ConstPtr& scan_in3)
{
	sensor_msgs::LaserScan vertical_laser;
	vertical_laser.header.frame_id 	= verti_laser_frame_id_;
	vertical_laser.header.stamp 	= scan_in0->header.stamp;
	vertical_laser.header.seq 		= scan_in0->header.seq;
	vertical_laser.angle_increment 	= -scan_in0->angle_increment;
	vertical_laser.angle_min 		= scan_in0->angle_max;
	vertical_laser.angle_max 		= scan_in0->angle_min;
	vertical_laser.time_increment 	= scan_in0->time_increment;
	vertical_laser.scan_time 		= scan_in0->scan_time;
	vertical_laser.range_min 		= scan_in0->range_min;
	vertical_laser.range_max 		= scan_in0->range_max;
	vertical_laser.ranges.resize(scan_in0->ranges.size(), 0.0);

	//the vertical angle interval between two scans of sick-ldmrs;
	float thetha = 0.8*M_PI/180.0;
	for(size_t i=0; i<vertical_laser.ranges.size(); i++)
	{
		float scan_range0 = scan_in0->ranges[i];
		float scan_range1 = scan_in1->ranges[i];
		float scan_range2 = scan_in2->ranges[i];
		float scan_range3 = scan_in3->ranges[i];

		float longer_range, shorter_range;
		if(scan_range1 > scan_in1->range_min)
		{
			longer_range = scan_range0 > scan_range1 ? scan_range0:scan_range1;
			shorter_range = scan_range0 <= scan_range1 ? scan_range0:scan_range1;
		}
		else if(scan_range2 > scan_in2->range_min)
		{
			longer_range = scan_range2 > scan_range3 ? scan_range2:scan_range3;
			shorter_range = scan_range2 <= scan_range3 ? scan_range2:scan_range3;
		}

		if(shorter_range<scan_in0->range_min)vertical_laser.ranges[i] = 0.0;
		else
		{
			float angle = atan2(longer_range*thetha, longer_range-shorter_range)+thetha/2.0;
			if(angle<M_PI/18.0) vertical_laser.ranges[i] = 0.0;
			else
			{
				if(scan_range1 > scan_in1->range_min) vertical_laser.ranges[i] = scan_in1->ranges[i]*cos(thetha);
				if(scan_range2 > scan_in2->range_min) vertical_laser.ranges[i] = scan_in2->ranges[i]*cos(thetha);
			}
		}
	}
	vector<float> ranges_sorted;
	for(size_t i=0; i<vertical_laser.ranges.size(); i++)ranges_sorted.push_back(vertical_laser.ranges[vertical_laser.ranges.size()-i-1]);
	vertical_laser.ranges = ranges_sorted;
	vertical_laser_pub_.publish(vertical_laser);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "verti_laser_sickldmrs_node");
	verti_laser_sickldmrs verti_laser_sickldmrs_node;
	ros::spin();
	return (0);
}
