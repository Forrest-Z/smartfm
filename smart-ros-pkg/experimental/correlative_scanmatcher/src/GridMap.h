/*
 * GridMap.cpp
 *
 *  Created on: Jul 8, 2012
 *      Author: demian
 */
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/ros/conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/registration/registration.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <math.h>
#include <fmutil/fm_stopwatch.h>
#include <fmutil/fm_math.h>

using namespace std;

struct simple_pose
{
	double x, y, t;
	simple_pose(){x=0.0; y=0.0; t=0.0;};
	simple_pose(double X, double Y, double T)
	{
		x=X; y=Y; t=T;
	}

};

namespace PointCloudHelper
{
pcl::PointCloud<pcl::PointXYZ> compressTo2D(const sensor_msgs::PointCloud2& input_pts)
		{
	pcl::PointCloud<pcl::PointXYZ>::Ptr xy_rawpts(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr xy_pts(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(input_pts, *xy_rawpts);
	for(size_t i=0; i<xy_rawpts->points.size(); i++)
	{
		xy_rawpts->points[i].z = 0.0;
	}
	//downsample since now it is 2D
	cout<<"Before voxel filter: "<<xy_rawpts->points.size()<<endl;
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(xy_rawpts);
	sor.setLeafSize (0.05, 0.05, 0.05);
	sor.filter (*xy_pts);
	cout<<"After voxel filter: "<<xy_pts->points.size()<<endl;
	return *xy_pts;
		}

pcl::PointCloud<pcl::PointXYZ> compressTo2D(const pcl::PointCloud<pcl::PointXYZ> &input)
		{
	pcl::PointCloud<pcl::PointXYZ> xy_pts, xy_rawpts = input;
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
}

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
		ros::NodeHandle nh;
		pt_transformed_first_ = nh.advertise<sensor_msgs::PointCloud2>("pt_transformed_first", 10);
		pt_transformed_sec_ = nh.advertise<sensor_msgs::PointCloud2>("pt_transformed_sec", 10);
		pt_transformed_th_ = nh.advertise<sensor_msgs::PointCloud2>("pt_transformed_th", 10);
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

	void buildMapDirectDraw(const pcl::PointCloud<pcl::PointXYZ> &xy_pts)
	{
		pcl::getMinMax3D(xy_pts, min_pt_, max_pt_);
		//cout<<min_pt_.x<<" "<<min_pt_.y<<endl;
		//cout<<max_pt_.x<<" "<<max_pt_.y<<endl;
		pcl::PointXYZ size;
		size.x = max_pt_.x - min_pt_.x;
		size.y = max_pt_.y - min_pt_.y;
		size.z = max_pt_.z - min_pt_.z;
		data_ = *(new vector< vector<int> >(size.x/meter_per_pixel_+1, vector<int>(size.y/meter_per_pixel_+1)));
		//cout<<"Map size initialized with "<<data_.size()<<" "<<data_[0].size()<<endl;
		#pragma omp parallel for
		for(size_t i=0; i<xy_pts.points.size(); i++)
		{
			int grid_x = getGridX(xy_pts.points[i].x);
			int grid_y = getGridY(xy_pts.points[i].y);
			for(int j=0; j<=gausian_length_; j++)
			{
				if(grid_x < 0 || grid_y < 0) cout<<grid_x<<" "<<grid_y<<endl;
				else drawCircle(grid_x, grid_y, j, gausian_length_*M_PI);

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

	int score2D(const pcl::PointCloud<pcl::PointXYZ>& pcl_to_match, double &percent_hit)
	{

		int score = 0;
		int count = 0;
		for(size_t i=0; i<pcl_to_match.points.size(); i++)
		{
			pcl::PointXYZ pt = pcl_to_match.points[i];
			int grid_x = getGridX(pt.x), grid_y = getGridY(pt.y);
			if(outsideMap(grid_x, grid_y)) continue;
			score += data_[grid_x][grid_y];
			count ++;
		}
		percent_hit = count/(double)pcl_to_match.points.size();
		return score;
	}

	void buildMapKdTree(const pcl::PointCloud<pcl::PointXYZ> &xy_pts)
	{
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




	void findBestMatchRotateFirst(const pcl::PointCloud<pcl::PointXYZ> &pcl, simple_pose& best_pose, int& max_score, double tran_range, double tran_res, double or_range, double or_res, bool publish = true, simple_pose cur_pose= simple_pose(0.0,0.0,0.0) )
	{
		//although float number is allowed for res and range, to avoid quantization issue
		//only use pair of number that is zero remainder of range/res
		cout<<pcl.points.size()<<" ";
		fmutil::Stopwatch sw("RotateFirst");
		sw.start();
		max_score = 0;
		double eva_xres = tran_res;
		double eva_xrange = tran_range;
		int x_iter = eva_xrange/eva_xres*2;

		double eva_yres = tran_res;
		double eva_yrange = tran_range;
		int y_iter = eva_yrange/eva_yres*2;

		double eva_tres = or_res/180*M_PI;
		double eva_trange = or_range/180*M_PI;
		int t_iter = eva_trange/eva_tres*2;


		double total_percent = 0.0;
		int total_count = 0;
#pragma omp parallel for
		for(int i=0; i<=t_iter; i++)
		{
			double theta = i*eva_tres-eva_trange+cur_pose.t;
			double ct = cos(theta), st = sin(theta);
			//always get a fresh copy of points, rinse and start
			pcl::PointCloud<pcl::PointXYZ> p_copy(pcl);
			for(size_t j =0 ; j<p_copy.points.size(); j++)
			{
				pcl::PointXYZ p = p_copy.points[j];
				p_copy.points[j].x = p.x*ct - p.y*st;
				p_copy.points[j].y = p.x*st + p.y*ct;
			}


			for(int j=0; j<=x_iter; j++)
			{
				for(int k=0; k<=y_iter; k++)
				{
					pcl::PointCloud<pcl::PointXYZ> p_copy2(p_copy);

					double offsetx = j*eva_xres-eva_xrange+cur_pose.x;
					double offsety = k*eva_yres-eva_yrange+cur_pose.y;


					for(size_t l =0 ; l<p_copy2.points.size(); l++)
					{
						p_copy2.points[l].x += offsetx;
						p_copy2.points[l].y += offsety;
					}
					if(publish)
					{
						bool pub_cond_1 = (j==0 && k==y_iter/2 && i==t_iter/2);
						bool pub_cond_2 = (j==x_iter/2 && k==y_iter/2 && i==t_iter/2);
						bool pub_cond_3 = (j==x_iter && k==y_iter/2 && i==t_iter/2);
						if( pub_cond_1 || pub_cond_2 ||pub_cond_3)
						{
							sensor_msgs::PointCloud2 msg;
							msg.header = pcl.header;
							pcl::toROSMsg(p_copy2, msg);
							if(pub_cond_1) {pt_transformed_first_.publish(msg), cout<<" 1: ";}
							else if(pub_cond_2) {pt_transformed_sec_.publish(msg), cout<<" 2: ";}
							else {pt_transformed_th_.publish(msg), cout<<" 3: ";}
							cout<<offsetx<<" "<<offsety<<" "<<theta<<endl;
						}
					}
					double percent_hit;
					int score = score2D(p_copy2, percent_hit);
					total_percent +=percent_hit;
					total_count++;
					//cout<<"Current score at "<<offsetx<<" "<<offsety<<" "<<theta<<": "<<score<<" "<<max_score<<endl;
					if(max_score < score)
					{
						best_pose.x = offsetx; best_pose.y = offsety; best_pose.t = theta;
						max_score = score;
					}

				}
			}

		}
		cout<<" average percent hit = "<<total_percent/total_count<<" ";
		sw.end(true);
	}

	void findBestMatchBruteForce(const pcl::PointCloud<pcl::PointXYZ>& pcl, simple_pose& best_pose, double max_score, bool publish)
	{
		max_score=0;
		//current implementation is too sensitive to the parameters below due to the need to balance the speed and accuracy
		double eva_xres = 0.5;
		double eva_xrange = 2.0;
		int x_iter = eva_xrange/eva_xres; //always assume the vehicle only move forward

		double eva_yres = 0.5;
		double eva_yrange = 2.0;
		int y_iter = eva_yrange/eva_yres*2;

		double eva_tres = 1.0/180*M_PI;
		double eva_trange = 14.0/180*M_PI;
		int t_iter = eva_trange/eva_tres*2;
		//#pragma omp parallel for
		fmutil::Stopwatch sw_transform("Perform transform");
		fmutil::Stopwatch sw_lookupdata("Lookup table");
		for(int i = 0; i <= x_iter; i++)
		{
			double dou_i = i*eva_xres;
			for(int j = 0; j <= y_iter; j++)
			{
				double dou_j = j*eva_yres-eva_yrange;
				for(int k=0; k <=t_iter; k++)
				{
					double dou_t = k*eva_tres-eva_trange;

					sw_transform.start();
					Eigen::Affine3f e_tf = pcl::getTransformation(dou_i, dou_j, 0, 0, 0, dou_t);
					pcl::PointCloud<pcl::PointXYZ> pcl_transformed;
					pcl::transformPointCloud(pcl, pcl_transformed, e_tf);
					sw_transform.end(false);
					if(publish)
					{
						bool pub_cond_1 = (i==0 && j==0 && k==t_iter/2);
						bool pub_cond_2 = (i==0 && j==y_iter/2 && k==t_iter/2);
						bool pub_cond_3 = (i==0 && j==y_iter && k==t_iter/2);
						if( pub_cond_1 || pub_cond_2 ||pub_cond_3 )
						{
							sensor_msgs::PointCloud2 msg;
							msg.header = pcl.header;
							pcl::toROSMsg(pcl_transformed, msg);
							if(pub_cond_1) pt_transformed_first_.publish(msg);
							else if(pub_cond_2) pt_transformed_sec_.publish(msg);
							else pt_transformed_th_.publish(msg);
						}
					}
					sw_lookupdata.start();
					double percent_hit;
					int score = score2D(pcl_transformed, percent_hit);
					sw_lookupdata.end(false);
					if(max_score < score)
					{
						best_pose.x = dou_i; best_pose.y = dou_j; best_pose.t = dou_t;
						max_score = score;
					}
					//cout<<dou_i<<","<<dou_j<<", "<<dou_t<<": "<<score<<" ";
				}

			}
			//cout<<endl;
		}
		cout<<"Time spent for transform: "<<sw_transform.total_<<" ms."<<endl;
		cout<<"Time spent for lookup table: "<<sw_lookupdata.total_<<" ms."<<endl;
	}

private:
	double meter_per_pixel_, map_size_, x_offset_, y_offset_, max_dist_, range_covariance_;
	int draw_circle_segments_;
	pcl::PointXYZ min_pt_, max_pt_;
	vector< vector<int> > data_;
	vector<int> gaussian_mapping_;
	vector<double> circle_segments_cos_, circle_segments_sin_;
	ros::Publisher pt_transformed_first_, pt_transformed_sec_, pt_transformed_th_;
	int gausian_length_;
	vector<int> makeGaussianLinearMapping()
	{
		//max_dist_ = sqrt(log(255)*range_covariance);

		vector<int> mapping;
		for(int i=0; i<gausian_length_; i++)
		{
			double d = meter_per_pixel_ * i;
			mapping.push_back((int) (255*exp(-d*d/range_covariance_)));
			//cout<<d<<": "<<mapping[mapping.size()-1]<<" ";
		}
		//cout<<endl;
		return mapping;
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
		bool outside = (grid_x < 0 || grid_y < 0 || grid_x >= data_.size() || grid_y >= data_[0].size());
		//if(outside) cout<<"("<<grid_x<<","<<grid_y<<")";
		return outside;
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
};
