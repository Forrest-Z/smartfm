/*
 * psoMatching.h
 *
 *  Created on: Jan 23, 2013
 *      Author: demian
 */

#include "pcl_downsample.h"

template< int dimension>
class NormalsCorrelativeMatchingProblem
{
public:
	static const int nb_parameters = dimension;
	static int count;
	static double rotate_res_;
	static RasterMapImage* rm_;//does not name a type??
	static vector<vector<cv::Point2f> > rotated_dst_;
	static vector<vector<double> > rotated_normal_dst_;
	static double trans_res_;


	//complete pso part for normalCorrelative matching
	static void init(pcl::PointCloud<pcl::PointNormal> input_cloud, pcl::PointCloud<pcl::PointNormal> matching_cloud)
	{
		//pso easier to fall into local minima when higher rotate_res_ is used
		rotate_res_ = 1.0;
		//trans_res_ = 0.05;
		rm_ = new RasterMapImage(trans_res_, 0.03);


		input_cloud = pcl_downsample(input_cloud, trans_res_/2,trans_res_/2,trans_res_/2);
		matching_cloud = pcl_downsample(matching_cloud, trans_res_/2,trans_res_/2,trans_res_/2);

		vector<cv::Point2f> input_pts, input_normals;
		input_pts.resize(input_cloud.points.size());
		input_normals.resize(input_cloud.points.size());
		for(size_t i=0; i<input_cloud.points.size(); i++)
		{
			input_pts[i].x = input_cloud.points[i].x;
			input_pts[i].y = input_cloud.points[i].y;
			input_normals[i].x = input_cloud.points[i].normal_x;
			input_normals[i].y = input_cloud.points[i].normal_y;
		}
		rm_->getInputPoints(input_pts, input_normals);

		fmutil::Stopwatch rotate_time("rotate time");


		for(double i=-180; i<180; i+=rotate_res_)
		{
			pcl::PointCloud<pcl::PointNormal> matching_cloud_tf = matching_cloud;
			double yaw_rotate = i/180.*M_PI;
			Eigen::Vector3f bl_trans(0, 0, 0.);
			Eigen::Quaternionf bl_rotation (cos(yaw_rotate/2.), 0, 0, -sin(yaw_rotate/2.) );
			pcl_utils::transformPointCloudWithNormals<pcl::PointNormal>(matching_cloud_tf, matching_cloud_tf, bl_trans, bl_rotation);

			vector<double> normal_pt;
			normal_pt.resize(matching_cloud_tf.points.size());
			vector<cv::Point2f> search_pt;
			search_pt.resize(matching_cloud_tf.points.size());

			for(size_t j=0; j<matching_cloud_tf.points.size(); j++)
			{
				normal_pt[j] = atan2(matching_cloud_tf.points[j].normal_y, matching_cloud_tf.points[j].normal_x);
				search_pt[j].x = matching_cloud_tf.points[j].x;
				search_pt[j].y = matching_cloud_tf.points[j].y;
			}
			rotated_normal_dst_.push_back(normal_pt);
			rotated_dst_.push_back(search_pt);
		}
		rotate_time.end();
	}

	static void free(void)
	{
	}

	static double get_lbound(int index)
	{
		if(index == 2) return -M_PI;
		else return -20;
	}

	static double get_ubound(int index)
	{
		if(index == 2) return M_PI;
		else return 20;
	}

	static bool stop(double fitness, int epoch)
	{
		//it seems to have a lower probability to reach global optimum when count is used.
		//merely co-incidents?
		//return count>25000;
		//return fitness<40 || epoch>20000;// ;
		//return epoch>1000;
		return fitness<0.583;
	}

	static double evaluate(void * x)
	{
		double * params = (double*) x;
		int offset_x = round(params[0]/trans_res_);
		int offset_y = round(params[1]/trans_res_);
		int rot_value = round((params[2]/M_PI*180)/rotate_res_)+180/rotate_res_;

		if(rot_value < 0 || rot_value > rotated_dst_.size()-1) return 0.;
		vector<cv::Point2f> rotated_dst = rotated_dst_[rot_value];
		vector<double> rotated_normal_dst = rotated_normal_dst_[rot_value];
		double score = rm_->getScoreWithNormal(rotated_dst, rotated_normal_dst, offset_x, offset_y);
		count++;
    if(score<0) score=0;
		return sqrt(score);
	}
};
template<int dimension> int     NormalsCorrelativeMatchingProblem<dimension>::count;
template<int dimension> RasterMapImage* NormalsCorrelativeMatchingProblem<dimension>::rm_;
template<int dimension> vector<vector<cv::Point2f> > NormalsCorrelativeMatchingProblem<dimension>::rotated_dst_;
template<int dimension> vector<vector<double> > NormalsCorrelativeMatchingProblem<dimension>::rotated_normal_dst_;
template<int dimension> double NormalsCorrelativeMatchingProblem<dimension>::rotate_res_;
template<int dimension> double NormalsCorrelativeMatchingProblem<dimension>::trans_res_;
