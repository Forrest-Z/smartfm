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

	//the first element of the pair is a cluster of point serial in the LIDAR, the 2nd element is the corresponding id of the contour;
	std::vector < std::pair<std::vector<int>, int> > movingObjectClusters;
};

class DATMO_labelledScan
{
	public:
	sensor_msgs::LaserScan laser_scan;
	geometry_msgs::PoseStamped poseInOdom;

	//0-nothing, 1-vehicle, 2-motorbike, 3-pedestrian, 4-under-determined;
	std::vector<int> type_mask;
};

class DATMO_abstractSummary
{
	public:
	int labelled_scan_startSerial;
	int labelled_scan_endSerial;
	std::vector<std::vector<int> > type_masks;
};

class compressed_scan_segment
{

	public:
	//raw information;
	vector<geometry_msgs::Point32> rawPoints;
	vector<float> rawIntensities;

	//compressed information;
	geometry_msgs::Point32 KeyPoint[3];
	float intensities[3];
	int m, n;	//"m" is the point number between the first two key points; similar to "n";
	float sigmaM, sigmaN;	//"sigmaM" is the largest distance of the points in the first line segment to it; similar to "sigmaN";

	void compress_scan()
	{
		assert(rawPoints.size()!=0);
		assert(rawPoints.size() == rawIntensities.size());
		if(rawPoints.size()==1)
		{
		}
		else if(rawPoints.size()==2)
		{
		}
		//in cases where at least 3 points;
		else
		{
		}
	}
};

class object_cluster_segments
{
	public:
	int object_type;
	vector <geometry_msgs::Pose> pose_InLatestCoord_vector;
	vector <compressed_scan_segment> scan_segment_batch;
};
