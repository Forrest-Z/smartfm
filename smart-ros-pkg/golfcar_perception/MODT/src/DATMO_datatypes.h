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
			n = 0;
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

			m=0;n=0;
			sigmaM = 0.0; sigmaN = 0.0;

			std::vector<float> distance_vecM, distance_vecN;
			float total_distM=0.0, total_distN = 0.0;
			for(size_t i=1; i<longest_serial; i++)
			{
				float pt2line_dist = DistPoint2Line(KeyPoint[0], KeyPoint[1], rawPoints[i]);
				total_distM = total_distM + pt2line_dist;
				distance_vecM.push_back(pt2line_dist);
				m++;
			}
			float avgM = total_distM/((float)m + 0.000000001);
			for(size_t i=0; i<distance_vecM.size(); i++)sigmaM = sigmaM + (distance_vecM[i]-avgM)*(distance_vecM[i]-avgM)/(float(m)+0.000000001);

			for(size_t i=longest_serial+1; i<rawPoints.size()-1; i++)
			{
				float pt2line_dist = DistPoint2Line(KeyPoint[1], KeyPoint[2], rawPoints[i]);
				total_distN = total_distN + pt2line_dist;
				distance_vecN.push_back(pt2line_dist);
				n++;
			}
			float avgN = total_distN/((float)n + 0.000000001);
			for(size_t i=0; i<distance_vecN.size(); i++) sigmaN = sigmaN + (distance_vecN[i]-avgN)*(distance_vecN[i]-avgN)/(float(n)+0.000000001);
		}
	}

	void compress_scan_latestLIDAR()
	{
		//for(size_t i=0; i<serial_in_scan.size(); i++)cout<<serial_in_scan[i]<<"\t"; cout<<endl;
		//for(size_t i=0; i<lidarPoints.size(); i++)cout<<lidarPoints[i].x<<","<<lidarPoints[i].y<<"\t"; cout<<endl;
		//cout<<front_dist2background<<","<<back_dist2background<<"\t"<<endl;

		assert(lidarPoints.size() == rawIntensities.size());
		if(lidarPoints.size()==0)
		{
			//cout<<"lidarPoints size 0"<<endl;
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

		else if(lidarPoints.size()==1)
		{
			KeyPoint[0] = lidarPoints.front();
			KeyPoint[1] = lidarPoints.front();
			KeyPoint[2] = lidarPoints.front();

			intensities[0] = rawIntensities.front();
			intensities[1] = rawIntensities.front();
			intensities[2] = rawIntensities.front();

			m = 0;
			n = 0;
			sigmaM = 0.0;
			sigmaN = 0.0;
		}
		else if(lidarPoints.size()==2)
		{
			KeyPoint[0] = lidarPoints.front();
			KeyPoint[1] = lidarPoints.front();
			KeyPoint[2] = lidarPoints.back();
			intensities[0] = rawIntensities.front();
			intensities[1] = rawIntensities.front();
			intensities[2] = rawIntensities.back();
			m = 0;
			n = 0;
			sigmaM = 0.0;
			sigmaN = 0.0;
		}
		//in cases where at least 3 points;
		else
		{
			KeyPoint[0] = lidarPoints.front();
			KeyPoint[2] = lidarPoints.back();
			intensities[0] = rawIntensities.front();
			intensities[2] = rawIntensities.back();
			//cout<<lidarPoints.front().x<< ""<<lidarPoints.front().y;
			size_t longest_serial = 1;
			float longest_distance = 0.0;
			for(size_t i=1; i<lidarPoints.size(); i++)
			{
				float pt2line_dist = DistPoint2Line(KeyPoint[0], KeyPoint[2], lidarPoints[i]);
				if(pt2line_dist>longest_distance)
				{
					longest_distance = pt2line_dist;
					longest_serial = i;
				}
			}
			KeyPoint[1] = lidarPoints[longest_serial];
			intensities[1] = rawIntensities[longest_serial];

			m=0;n=0;
			sigmaM = 0.0; sigmaN = 0.0;

			std::vector<float> distance_vecM, distance_vecN;
			float total_distM=0.0, total_distN = 0.0;
			for(size_t i=1; i<longest_serial; i++)
			{
				float pt2line_dist = DistPoint2Line(KeyPoint[0], KeyPoint[1], lidarPoints[i]);
				total_distM = total_distM + pt2line_dist;
				distance_vecM.push_back(pt2line_dist);
				m++;
			}
			float avgM = total_distM/((float)m + 0.000000001);
			for(size_t i=0; i<distance_vecM.size(); i++)sigmaM = sigmaM + (distance_vecM[i]-avgM)*(distance_vecM[i]-avgM)/(float(m)+0.000000001);

			for(size_t i=longest_serial+1; i<lidarPoints.size()-1; i++)
			{
				float pt2line_dist = DistPoint2Line(KeyPoint[1], KeyPoint[2], lidarPoints[i]);
				total_distN = total_distN + pt2line_dist;
				distance_vecN.push_back(pt2line_dist);
				n++;
			}
			float avgN = total_distN/((float)n + 0.000000001);
			for(size_t i=0; i<distance_vecN.size(); i++) sigmaN = sigmaN + (distance_vecN[i]-avgN)*(distance_vecN[i]-avgN)/(float(n)+0.000000001);
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

	//in latest-Baselink frame;
	double rough_direction_latestLIDAR;
	vector <geometry_msgs::Point32> rawpoints_centroids, centroid_position, cluster_attached_centroids;

	void GetCentroid_rawPoints()
	{
		for(size_t j=0; j<scan_segment_batch.size(); j++)
		{
			geometry_msgs::Point32 centroid_tmp;
			centroid_tmp.x = 0.0;
			centroid_tmp.y = 0.0;
			centroid_tmp.z = 0.0;
			for(size_t k=0; k<scan_segment_batch[j].rawPoints.size(); k++)
			{
				centroid_tmp.x = centroid_tmp.x + scan_segment_batch[j].rawPoints[k].x;
				centroid_tmp.y = centroid_tmp.y + scan_segment_batch[j].rawPoints[k].y;
				centroid_tmp.z = centroid_tmp.z + scan_segment_batch[j].rawPoints[k].z;
			}

			if(scan_segment_batch[j].rawPoints.size()!=0)
			{
				centroid_tmp.x = centroid_tmp.x/(float)scan_segment_batch[j].rawPoints.size();
				centroid_tmp.y = centroid_tmp.y/(float)scan_segment_batch[j].rawPoints.size();
				centroid_tmp.z = centroid_tmp.z/(float)scan_segment_batch[j].rawPoints.size();
			}
			rawpoints_centroids.push_back(centroid_tmp);
		}
	}

	void GetCentroid_latestLIDAR()
	{
		for(size_t j=0; j<scan_segment_batch.size(); j++)
		{
			geometry_msgs::Point32 centroid_tmp;
			centroid_tmp.x = 0.0;
			centroid_tmp.y = 0.0;
			centroid_tmp.z = 0.0;
			for(size_t k=0; k<scan_segment_batch[j].lidarPoints.size(); k++)
			{
				centroid_tmp.x = centroid_tmp.x + scan_segment_batch[j].lidarPoints[k].x;
				centroid_tmp.y = centroid_tmp.y + scan_segment_batch[j].lidarPoints[k].y;
				centroid_tmp.z = centroid_tmp.z + scan_segment_batch[j].lidarPoints[k].z;
			}

			if(scan_segment_batch[j].lidarPoints.size()!=0)
			{
				centroid_tmp.x = centroid_tmp.x/(float)scan_segment_batch[j].lidarPoints.size();
				centroid_tmp.y = centroid_tmp.y/(float)scan_segment_batch[j].lidarPoints.size();
				centroid_tmp.z = centroid_tmp.z/(float)scan_segment_batch[j].lidarPoints.size();
			}
			centroid_position.push_back(centroid_tmp);
		}

		if(centroid_position.back().x-centroid_position.front().x!=0.0) rough_direction_latestLIDAR = atan2(centroid_position.back().y-centroid_position.front().y, centroid_position.back().x-centroid_position.front().x);
		else rough_direction_latestLIDAR = 0.0;
	}

	//convert the compressed scans into the cluster attached-coordinates, so that the features are pose-invariant, including the keypoints and segments centroids;
	void GenPosInvariantCompressedSegment()
	{
		tf::Pose localcoord_inLIDAR(tf::Matrix3x3(tf::createQuaternionFromYaw(rough_direction_latestLIDAR)), tf::Vector3(tfScalar(centroid_position.front().x), tfScalar(centroid_position.front().y), tfScalar(centroid_position.front().z)));
		//2nd: transform the compressed scan segment into the local coordinate;
		for(size_t j=0; j<scan_segment_batch.size(); j++)
		{
			for(size_t k=0; k<3; k++)
			{
				tf::Pose keypoint_LIDAR(tf::Matrix3x3(tf::createQuaternionFromYaw(0.0)), tf::Vector3(tfScalar(scan_segment_batch[j].KeyPoint[k].x), tfScalar(scan_segment_batch[j].KeyPoint[k].y), tfScalar(scan_segment_batch[j].KeyPoint[k].z)));
				tf::Pose keypoint_local = localcoord_inLIDAR.inverse()*keypoint_LIDAR;

				scan_segment_batch[j].KeyPoint[k].x = (float)keypoint_local.getOrigin().getX();
				scan_segment_batch[j].KeyPoint[k].y = (float)keypoint_local.getOrigin().getY();
				scan_segment_batch[j].KeyPoint[k].z = (float)keypoint_local.getOrigin().getZ();
			}
			tf::Pose centroid_LIDAR(tf::Matrix3x3(tf::createQuaternionFromYaw(0.0)), tf::Vector3(tfScalar(centroid_position[j].x), tfScalar(centroid_position[j].y), tfScalar(centroid_position[j].z)));
			tf::Pose keypoint_local = localcoord_inLIDAR.inverse()*centroid_LIDAR;
			geometry_msgs::Point32 cluster_attached_coord_centroid;
			cluster_attached_coord_centroid.x = (float)keypoint_local.getOrigin().getX();
			cluster_attached_coord_centroid.y = (float)keypoint_local.getOrigin().getY();
			cluster_attached_coord_centroid.z = (float)keypoint_local.getOrigin().getZ();
			cluster_attached_centroids.push_back(cluster_attached_coord_centroid);
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



