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
	vector<int> serial_in_scan;
	vector<geometry_msgs::Point32> rawPoints;
	vector<geometry_msgs::Point32> odomPoints;
	vector<float> rawIntensities;

	//compressed information;
	geometry_msgs::Point32 KeyPoint[3];
	float intensities[3];
	int m, n;	//"m" is the point number between the first two key points; similar to "n";
	float sigmaM, sigmaN;	//"sigmaM" is the largest distance of the points in the first line segment to it; similar to "sigmaN";

	compressed_scan_segment()
	{
		m=0; n=0; sigmaM=0.0; sigmaN=0.0;
	}

	void compress_scan()
	{
		assert(rawPoints.size() == rawIntensities.size());
		if(rawPoints.size()==0)
		{
			//cout<<"rawPoints size 0"<<endl;
			geometry_msgs::Point32 virtual_deputy;
			virtual_deputy.x = 0.0;
			virtual_deputy.y = 0.0;
			virtual_deputy.z = 0.0;
			KeyPoint[0] = virtual_deputy;
			KeyPoint[1] = virtual_deputy;
			KeyPoint[2] = virtual_deputy;
			intensities[0] = 0.0;
			intensities[1] = 0.0;
			intensities[2] = 0.0;
			m = 0;
			n = 0;
			sigmaM = 0.0;
			sigmaN = 0.0;
		}

		else if(rawPoints.size()==1)
		{
			KeyPoint[0] = rawPoints.front();
			KeyPoint[1] = rawPoints.front();
			KeyPoint[2] = rawPoints.front();

			intensities[0] = rawIntensities.front();
			intensities[1] = rawIntensities.front();
			intensities[2] = rawIntensities.front();

			m = 0;
			n = 0;
			sigmaM = 0.0;
			sigmaN = 0.0;
		}
		else if(rawPoints.size()==2)
		{
			KeyPoint[0] = rawPoints.front();
			KeyPoint[1] = rawPoints.front();
			KeyPoint[2] = rawPoints.back();
			intensities[0] = rawIntensities.front();
			intensities[1] = rawIntensities.front();
			intensities[2] = rawIntensities.back();
			m = 0;
			n = 1;
			sigmaM = 0.0;
			sigmaN = 0.0;
		}
		//in cases where at least 3 points;
		else
		{
			KeyPoint[0] = rawPoints.front();
			KeyPoint[2] = rawPoints.back();
			intensities[0] = rawIntensities.front();
			intensities[2] = rawIntensities.back();
			//cout<<rawPoints.front().x<< ""<<rawPoints.front().y;
			size_t longest_serial = 1;
			float longest_distance = 0.0;
			for(size_t i=1; i<rawPoints.size(); i++)
			{
				float pt2line_dist = DistPoint2Line(KeyPoint[0], KeyPoint[2], rawPoints[i]);
				if(pt2line_dist>longest_distance)
				{
					longest_distance = pt2line_dist;
					longest_serial = i;
				}
			}
			KeyPoint[1] = rawPoints[longest_serial];
			intensities[1] = rawIntensities[longest_serial];

			m=1;n=1;
			sigmaM = 0.0; sigmaN = 0.0;

			for(size_t i=1; i<longest_serial; i++)
			{
				float pt2line_dist = DistPoint2Line(KeyPoint[0], KeyPoint[1], rawPoints[i]);
				if(pt2line_dist>sigmaM) sigmaM = pt2line_dist;
				m++;
			}
			for(size_t i=longest_serial+1; i<rawPoints.size()-1; i++)
			{
				float pt2line_dist = DistPoint2Line(KeyPoint[1], KeyPoint[2], rawPoints[i]);
				if(pt2line_dist>sigmaN) sigmaN = pt2line_dist;
				n++;
			}
		}
	}

	float DistPoint2Line(geometry_msgs::Point32 line_start_pt, geometry_msgs::Point32 line_end_pt, geometry_msgs::Point32 query_pt)
	{
		//construct the line based on the two end points;
		//ax+by+c=0;
		float a = line_start_pt.y-line_end_pt.y;
		float b = line_end_pt.x-line_start_pt.x;
		float c = line_start_pt.x*line_end_pt.y-line_end_pt.x*line_start_pt.y;
		//distance formular;
		return (fabsf(a*query_pt.x+b*query_pt.y+c)/sqrt(a*a+b*b));
	}
};

class object_cluster_segments
{
	public:
	int object_type;
	int contour_serial;
	vector <geometry_msgs::Pose> pose_InLatestCoord_vector;
	vector <compressed_scan_segment> scan_segment_batch;

	float x, y, thetha, v, omega;
};
