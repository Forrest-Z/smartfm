/*
 * correlative_matching.cpp
 *
 *  Created on: May 7, 2012
 *      Author: demian
 */

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud.h>

using namespace std;

class correlative_matching
{
public:
	correlative_matching(double map_size, double map_low_res, double map_high_res) :map_size_(map_size), map_low_res_(map_low_res), map_high_res_(map_high_res)
	{
		int high_res_no = map_size/map_high_res+1;
		high_res_map_.resize(high_res_no);
		for(int i=0; i<high_res_no; i++) high_res_map_[i].resize(high_res_no);
		int low_res_no =  map_size/map_low_res+1;
		low_res_map_.resize(low_res_no);
		for(int i=0; i<low_res_no; i++) low_res_map_[i].resize(low_res_no);
	}


	//obtain the best matching based on the last input data
	//possibly should include the estimated variance
	void getBestMatch(pcl::PointCloud<pcl::PointXYZ>& input)
	{
		generatePrior(input);
	}

	void getLowResMap(sensor_msgs::PointCloud& map)
	{
		map = low_res_map_pc_;
	}

private:

	vector<vector<double> > high_res_map_, low_res_map_;
	sensor_msgs::PointCloud low_res_map_pc_;


	//a square map as the underlying prior
	double map_size_;
	double map_low_res_;
	double map_high_res_;

	void generatePrior(pcl::PointCloud<pcl::PointXYZ>& input)
	{
		 pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		 //compress the cloud into 2D
		 for(size_t i=0; i<input.points.size(); i++)
		 {
			 input.points[i].z = 0;
		 }
		 kdtree.setInputCloud (input.makeShared());
		 low_res_map_pc_.points.clear();
		 int i=0, j=0;
		 for(double x = -(map_size_/2.0); x<map_size_/2.0; x+=map_low_res_)
		 {
			 j = 0;
			 for(double y = -(map_size_/2.0); y<map_size_/2.0; y+=map_low_res_)
			 {
				 pcl::PointXYZ searchPoint;
				 searchPoint.x = x;
				 searchPoint.y = y;
				 //only the nearest neighbor is considered, squared distance cost function is used
				 int K = 1;
				 std::vector<int> pointIdxNKNSearch(K);
				 std::vector<float> pointNKNSquaredDistance(K);
				 kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
				 low_res_map_[i][j] = (pointNKNSquaredDistance[0]);
				 geometry_msgs::Point32 p;
				 p.x = x; p.y = y; p.z = sqrt(pointNKNSquaredDistance[0]);
				 low_res_map_pc_.points.push_back(p);
				 j++;
			 }
			 i++;
		 }

	};

};



