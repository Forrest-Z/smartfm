/*
 * GridMap.cpp
 *
 *  Created on: Jul 8, 2012
 *      Author: demian
 */
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/ros/conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
//#include <pcl/common/impl/transforms.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <fmutil/fm_stopwatch.h>
#include <fmutil/fm_math.h>

using namespace std;

class GridMap
{
public:
	GridMap(double meter_per_pixel, double range_covariance):
		meter_per_pixel_(meter_per_pixel), range_covariance_(range_covariance),
		max_dist_ (sqrt(log(255)*range_covariance)),
		gausian_length_((int) (max_dist_ / meter_per_pixel_ + 1))
	{
		draw_circle_segments_ = gausian_length_*M_PI;
		gaussian_mapping_ = makeGaussianLinearMapping();
		precalculateCosSin();
		cout<<"Initialized grid map with: "<<endl;
		cout<<"MMP: "<<meter_per_pixel_<<endl;
		cout<<"Range Cov: "<<range_covariance_<<endl;
		cout<<"Max dist: "<<max_dist_<<endl;
		cout<<"Gausian length: "<<gausian_length_<<endl;
		cout<<"Draw circle segments: "<<draw_circle_segments_<<endl;
	}

	void getMap(sensor_msgs::PointCloud& grid_map_pc)
	{
		double mmp = meter_per_pixel_;
		for(size_t i = 0; i < data_.size(); i++)
		{
			for(size_t j=0; j < data_[i].size(); j++)
			{
				geometry_msgs::Point32 p32;
				p32.x = i*mmp + min_pt_.x; p32.y = j*mmp + min_pt_.y; p32.z = data_[i][j];

				grid_map_pc.points.push_back(p32);
			}
		}
	}

	void getMMP(double& mmp)
	{
		mmp = meter_per_pixel_;
	}

	void buildMapDirectDraw(sensor_msgs::PointCloud2ConstPtr input_pts)
	{
		pcl::PointCloud<pcl::PointXYZ> xy_pts = compressTo2D(input_pts);
		pcl::getMinMax3D(xy_pts, min_pt_, max_pt_);
		pcl::PointXYZ size;
		size.x = max_pt_.x - min_pt_.x;
		size.y = max_pt_.y - min_pt_.y;
		size.z = max_pt_.z - min_pt_.z;
		data_ = *(new vector< vector<int> >(size.x/meter_per_pixel_+1, vector<int>(size.y/meter_per_pixel_+1)));
		cout<<"Map size initialized with "<<data_.size()<<" "<<data_[0].size()<<endl;
		#pragma omp parallel for
		for(size_t i=0; i<xy_pts.points.size(); i++)
		{
			int grid_x = getGridX(xy_pts.points[i].x);
			int grid_y = getGridY(xy_pts.points[i].y);
			int length = (int) (max_dist_ / meter_per_pixel_ + 1);
			for(int j=0; j<=length; j++)
			{
				if(grid_x < 0 || grid_y < 0) cout<<grid_x<<" "<<grid_y<<endl;
				else drawCircle(grid_x, grid_y, j, length*M_PI);

			}
		}
	}

	inline int getGridX(double x)
	{
		return (x - min_pt_.x)/meter_per_pixel_;
	}

	inline int getGridY(double y)
	{
		return (y - min_pt_.y)/meter_per_pixel_;
	}

	int score2D(pcl::PointCloud<pcl::PointXYZ> pcl_to_match)
	{

		int score = 0;
		for(size_t i=0; i<pcl_to_match.points.size(); i++)
		{
			pcl::PointXYZ pt = pcl_to_match.points[i];
			int grid_x = getGridX(pt.x), grid_y = getGridY(pt.y);
			if(outsideMap(grid_x, grid_y)) continue;
			score += data_[grid_x][grid_y];
		}
		return score;
	}

	void buildMapKdTree(sensor_msgs::PointCloud2ConstPtr input_pts)
	{
		pcl::PointCloud<pcl::PointXYZ> xy_pts = compressTo2D(input_pts);

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud (xy_pts.makeShared());

		pcl::getMinMax3D(xy_pts, min_pt_, max_pt_);
		cout<<min_pt_<<" "<<max_pt_<<endl;
		pcl::PointXYZ size;
		size.x = max_pt_.x - min_pt_.x;
		size.y = max_pt_.y - min_pt_.y;
		size.z = max_pt_.z - min_pt_.z;
		data_ = *(new vector< vector<int> >(size.x/meter_per_pixel_, vector<int>(size.y/meter_per_pixel_)));
		cout<<"Map size initialized with "<<data_.size()<<" "<<data_[0].size()<<endl;
		#pragma omp parallel for
		for(size_t i=0; i< data_.size(); i++)
		{
			for(size_t j=0; j<data_[i].size(); j++)
			{
				pcl::PointXYZ searchPoint;

				searchPoint.x = i * meter_per_pixel_ + min_pt_.x;
				searchPoint.y = j * meter_per_pixel_ + min_pt_.y;

				int K = 1;

				std::vector<int> pointIdxNKNSearch(K);
				std::vector<float> pointNKNSquaredDistance(K);

				if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
				{
					data_[i][j] ;
					double dist = sqrt(pointNKNSquaredDistance[0]);
					if(dist >= max_dist_)
					{
						data_[i][j] = 0;
					}
					else
					{
						data_[i][j] = gaussian_mapping_[ dist/ meter_per_pixel_ + 1];
					}
				}
				else data_[i][j] = 0;
			}
		}
		cout<<"Done build map"<<endl;
	}

private:
	double meter_per_pixel_, map_size_, x_offset_, y_offset_, max_dist_, range_covariance_;
	int draw_circle_segments_;
	pcl::PointXYZ min_pt_, max_pt_;
	vector< vector<int> > data_;
	vector<int> gaussian_mapping_;
	vector<double> circle_segments_cos_, circle_segments_sin_;
	int gausian_length_;
	vector<int> makeGaussianLinearMapping()
	{
		//max_dist_ = sqrt(log(255)*range_covariance);

		vector<int> mapping;
		for(int i=0; i<gausian_length_; i++)
		{
			double d = meter_per_pixel_ * i;
			mapping.push_back((int) (255*exp(-d*d/range_covariance_)));
			cout<<d<<": "<<mapping[mapping.size()-1]<<" ";
		}
		cout<<endl;
		return mapping;
	}

	pcl::PointCloud<pcl::PointXYZ> compressTo2D(sensor_msgs::PointCloud2ConstPtr input_pts)
	{
		pcl::PointCloud<pcl::PointXYZ> xy_rawpts;
		pcl::PointCloud<pcl::PointXYZ> xy_pts;//(new pcl::PointCloud<pcl::PointXY>());
		pcl::fromROSMsg(*input_pts, xy_rawpts);
		for(size_t i=0; i<xy_rawpts.points.size(); i++)
		{
			xy_rawpts.points[i].z = 0.0;
		}
		//downsample since now it is 2D
		cout<<"Before voxel filter: "<<xy_rawpts.points.size()<<endl;
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(xy_rawpts.makeShared());
		sor.setLeafSize (0.05, 0.05, 0.05);
		sor.filter (xy_pts);
		cout<<"After voxel filter: "<<xy_pts.points.size()<<endl;
		return xy_pts;
	}

	void precalculateCosSin()
	{
		for(int i=0; i <= draw_circle_segments_; i++)
		{
			float const t = 2*M_PI*(float)i/(float)draw_circle_segments_;
			circle_segments_cos_.push_back(cos(t));
			circle_segments_sin_.push_back(sin(t));
		}
	}

	bool outsideMap(int grid_x, int grid_y)
	{
		return (grid_x < 0 || grid_y < 0 || grid_x >= data_.size() || grid_y >= data_[0].size());
	}
	void drawCircle(int x, int y, int r, int segments)
	{
		for( int n = 0; n <= segments; n++ )
		{
			int grid_x = x + circle_segments_cos_[n]*r, grid_y = y +circle_segments_sin_[n]*r;
			if(outsideMap(grid_x, grid_y)) continue;
			data_[grid_x][grid_y] = fmutil::max(gaussian_mapping_[r], data_[grid_x][grid_y]);
		}

	}
public:
	template <typename PointT> void
	transformPointCloud (const pcl::PointCloud<PointT> &cloud_in,
	                          pcl::PointCloud<PointT> &cloud_out,
	                          const Eigen::Affine3f &transform)
	{
	  if (&cloud_in != &cloud_out)
	  {
	    // Note: could be replaced by cloud_out = cloud_in
	    cloud_out.header   = cloud_in.header;
	    cloud_out.is_dense = cloud_in.is_dense;
	    cloud_out.width    = cloud_in.width;
	    cloud_out.height   = cloud_in.height;
	    cloud_out.points.reserve (cloud_out.points.size ());
	    cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
	  }

	  if (cloud_in.is_dense)
	  {
	    // If the dataset is dense, simply transform it!
	    for (size_t i = 0; i < cloud_out.points.size (); ++i)
	      cloud_out.points[i].getVector3fMap () = transform *
	                                              cloud_in.points[i].getVector3fMap ();
	  }
	  else
	  {
	    // Dataset might contain NaNs and Infs, so check for them first,
	    // otherwise we get errors during the multiplication (?)
	    for (size_t i = 0; i < cloud_out.points.size (); ++i)
	    {
	      if (!pcl_isfinite (cloud_in.points[i].x) ||
	          !pcl_isfinite (cloud_in.points[i].y) ||
	          !pcl_isfinite (cloud_in.points[i].z))
	        continue;
	      cloud_out.points[i].getVector3fMap () = transform *
	                                              cloud_in.points[i].getVector3fMap ();
	    }
	  }
	}
};

GridMap* gm;
ros::Publisher* pub_;
bool initialized=false;
void pcCallback(sensor_msgs::PointCloud2ConstPtr pc)
{
	fmutil::Stopwatch sw;
	sw.start("Build map");
	if(initialized)
	{
		for(double i = -2.0; i <= 2.0; i+=0.5)
		{
			for(double j = -2.0; j <= 2.0; j+=0.5)
			{
				//todo: find out why transformPointCloud cannot be used directly
				Eigen::Affine3f e_tf = pcl::getTransformation(j, i, 0, 0, 0, 0);
				pcl::PointCloud<pcl::PointXYZ> pcl, pcl_transformed;
				pcl::fromROSMsg(*pc, pcl);
				gm->transformPointCloud(pcl, pcl_transformed, e_tf);
				cout<<gm->score2D(pcl_transformed)<<" ";
			}
			cout<<endl;
		}
		cout<<endl;
	}
	gm->buildMapDirectDraw(pc);

	sensor_msgs::PointCloud grid_map_pc;
	grid_map_pc.header = pc->header;

	gm->getMap(grid_map_pc);
	pub_->publish(grid_map_pc);
	initialized = true;
	sw.end();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_map");
    gm = new GridMap(0.05, 0.1);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("pc_out", 10, &pcCallback);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("grid_map", 10);
    pub_ = &pub;
    ros::spin();
}
