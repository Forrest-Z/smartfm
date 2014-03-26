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
	std::vector<int> type_mask;
};

class DATMO_abstractSummary
{
	public:
	int labelled_scan_startSerial;
	int labelled_scan_endSerial;

	int process_scan_startSerial;
	int process_scan_endSerial;

	std::vector<std::vector<int> > masks_under_processing;

	//0-nothing, 1-vehicle, 2-motorbike, 3-pedestrian;
	std::vector<std::vector<int> > type_masks;
};

class compressed_scan_segment
{
	public:
	//raw information;
	vector<int> serial_in_scan;
	vector<geometry_msgs::Point32> rawPoints;
	vector<geometry_msgs::Point32> odomPoints;
	//points in the latest LIDAR frame;
	vector<geometry_msgs::Point32> lidarPoints;
	vector<float> rawIntensities;

	//negative value means object is the foreground, which has shorter distance than background from LIDAR;
	//positive value means the opposite;
	//near 0.0 value means out of FOV, out-of-range background readings, etc;
	double front_dist2background, back_dist2background;

	//compressed information;
	geometry_msgs::Point32 KeyPoint[3];
	float intensities[3];
	int m, n;	//"m" is the point number between the first two key points; similar to "n";
	float sigmaM, sigmaN;	//"sigmaM" is the largest distance of the points in the first line segment to it; similar to "sigmaN";

	compressed_scan_segment()
	{
		m=0; n=0; sigmaM=0.0; sigmaN=0.0;
		front_dist2background = 0.0;
		back_dist2background  = 0.0;
	}

	void compress_scan()
	{
		//for(size_t i=0; i<serial_in_scan.size(); i++)cout<<serial_in_scan[i]<<"\t"; cout<<endl;
		//for(size_t i=0; i<rawPoints.size(); i++)cout<<rawPoints[i].x<<","<<rawPoints[i].y<<"\t"; cout<<endl;
		//cout<<front_dist2background<<","<<back_dist2background<<"\t"<<endl;

		assert(odomPoints.size() == rawIntensities.size());
		if(odomPoints.size()==0)
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

		else if(odomPoints.size()==1)
		{
			KeyPoint[0] = odomPoints.front();
			KeyPoint[1] = odomPoints.front();
			KeyPoint[2] = odomPoints.front();

			intensities[0] = rawIntensities.front();
			intensities[1] = rawIntensities.front();
			intensities[2] = rawIntensities.front();

			m = 0;
			n = 0;
			sigmaM = 0.0;
			sigmaN = 0.0;
		}
		else if(odomPoints.size()==2)
		{
			KeyPoint[0] = odomPoints.front();
			KeyPoint[1] = odomPoints.front();
			KeyPoint[2] = odomPoints.back();
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
			KeyPoint[0] = odomPoints.front();
			KeyPoint[2] = odomPoints.back();
			intensities[0] = rawIntensities.front();
			intensities[2] = rawIntensities.back();
			//cout<<rawPoints.front().x<< ""<<rawPoints.front().y;
			size_t longest_serial = 1;
			float longest_distance = 0.0;
			for(size_t i=1; i<odomPoints.size(); i++)
			{
				float pt2line_dist = DistPoint2Line(KeyPoint[0], KeyPoint[2], odomPoints[i]);
				if(pt2line_dist>longest_distance)
				{
					longest_distance = pt2line_dist;
					longest_serial = i;
				}
			}
			KeyPoint[1] = odomPoints[longest_serial];
			intensities[1] = rawIntensities[longest_serial];

			m=1;n=1;
			sigmaM = 0.0; sigmaN = 0.0;

			for(size_t i=1; i<longest_serial; i++)
			{
				float pt2line_dist = DistPoint2Line(KeyPoint[0], KeyPoint[1], odomPoints[i]);
				if(pt2line_dist>sigmaM) sigmaM = pt2line_dist;
				m++;
			}

			for(size_t i=longest_serial+1; i<odomPoints.size()-1; i++)
			{
				float pt2line_dist = DistPoint2Line(KeyPoint[1], KeyPoint[2], odomPoints[i]);
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
	vector <geometry_msgs::Pose> pose_InOdom_vector;
	vector <compressed_scan_segment> scan_segment_batch;
	float x, y, thetha, v, omega;

	cv::Moments ST_moments;
	double  ST_Humoment[7];

	//for new feature set;
	geometry_msgs::Point32 latest_centroid_odom;
	double rough_direction_odom;

	void GenPosInvariantCompressedSegment()
	{
		//1st: calculate cluster's rough moving direction, and determine the local coordinate;
		std::vector<geometry_msgs::Point32> centroid_position;
		for(size_t j=0; j<scan_segment_batch.size(); j=j+scan_segment_batch.size()-1)
		{
			geometry_msgs::Point32 centroid_tmp;
			centroid_tmp.x = 0.0;
			centroid_tmp.y = 0.0;
			centroid_tmp.z = 0.0;
			for(size_t k=0; k<scan_segment_batch[j].odomPoints.size(); k++)
			{
				centroid_tmp.x = centroid_tmp.x + scan_segment_batch[j].odomPoints[k].x;
				centroid_tmp.y = centroid_tmp.y + scan_segment_batch[j].odomPoints[k].y;
				centroid_tmp.z = centroid_tmp.z + scan_segment_batch[j].odomPoints[k].z;
			}
			centroid_tmp.x = centroid_tmp.x/(float)scan_segment_batch[j].odomPoints.size();
			centroid_tmp.y = centroid_tmp.y/(float)scan_segment_batch[j].odomPoints.size();
			centroid_tmp.z = centroid_tmp.z/(float)scan_segment_batch[j].odomPoints.size();
			centroid_position.push_back(centroid_tmp);
		}
		latest_centroid_odom = centroid_position.back();
		rough_direction_odom = atan2(centroid_position.back().y-centroid_position.front().y, centroid_position.back().x-centroid_position.front().x);
		tf::Pose localcoord_inOdom(tf::Matrix3x3(tf::createQuaternionFromYaw(rough_direction_odom)), tf::Vector3(tfScalar(centroid_position.front().x), tfScalar(centroid_position.front().y), tfScalar(centroid_position.front().z)));
		//ROS_INFO("localcoord_inOdom: %f, %f", localcoord_inOdom.getOrigin().getX(), localcoord_inOdom.getOrigin().getY());

		//2nd: transform the compressed scan segment into the local coordinate;
		for(size_t j=0; j<scan_segment_batch.size(); j++)
		{
			for(size_t k=0; k<3; k++)
			{
				tf::Pose keypoint_odom(tf::Matrix3x3(tf::createQuaternionFromYaw(0.0)), tf::Vector3(tfScalar(scan_segment_batch[j].KeyPoint[k].x), tfScalar(scan_segment_batch[j].KeyPoint[k].y), tfScalar(scan_segment_batch[j].KeyPoint[k].z)));
				tf::Pose keypoint_local = localcoord_inOdom.inverse()*keypoint_odom;
				scan_segment_batch[j].KeyPoint[k].x = (float)keypoint_local.getOrigin().getX();
				scan_segment_batch[j].KeyPoint[k].y = (float)keypoint_local.getOrigin().getY();
				scan_segment_batch[j].KeyPoint[k].z = (float)keypoint_local.getOrigin().getZ();
			}
		}
	}
};

class DATMO_RawTrainingData
{
	public:

	std::vector <int> scan_serials;
	//here "-1" is for background noise, and object cluster labels begin with "0";
	std::vector <std::vector<int> > scan_ClusterLabel_vector;
	std::vector <sensor_msgs::PointCloud> clusters_odom;
	std::vector <sensor_msgs::PointCloud> clusters_baselink;


	//0-nothing, 1-vehicle, 2-motorbike, 3-pedestrian;
	std::vector <std::vector<int> > scan_ClassType_vector;
};



