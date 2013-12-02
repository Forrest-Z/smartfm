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
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <cmath>

using namespace std;
using namespace ros;
using namespace cv;

class DATMO_TrainingScan
{
	public:
	sensor_msgs::LaserScan laser_scan;
	geometry_msgs::PoseStamped poseInOdom;

	std::vector < std::pair<std::vector<size_t>, size_t> > movingObjectClusters;
};

class DATMO_labelledScan
{
	public:
	sensor_msgs::LaserScan laser_scan;
	geometry_msgs::PoseStamped poseInOdom;

	//0-nothing, 1-vehicle, 2-motorbike, 3-pedestrian, 4-other;
	std::vector<int> type_mask;
};
