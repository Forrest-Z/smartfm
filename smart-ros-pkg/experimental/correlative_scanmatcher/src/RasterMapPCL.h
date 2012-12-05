/*
 * RasterMapPCL.h
 *
 *  Created on: Nov 28, 2012
 *      Author: demian
 */

#include "RasterMapImageOpt.h"
#include <sensor_msgs/PointCloud.h>


class RasterMapPCL
{
public:
	//changing the 1st RasterMap resolution and range from 0.2,0.5 to 0.1,0.1 seems to miss some
	//good potential close loop while increasing false detection
	//apparent decreasing the range that causes the above scenario
	RasterMapPCL(): rm_(1.0, 0.02), rm2_(0.1, 0.02), rm3_(0.01, 0.01)
	{};

	template <class T>
	void setInputPts(vector<T> &pc, bool verification=false)
	{
		rm2_.getInputPoints(pc);
		if(!verification)
		{
			rm_.getInputPoints(pc);
			rm3_.getInputPoints(pc);
		}
	}

	transform_info getBestTf(sensor_msgs::PointCloud &pc)
	{
		vector<cv::Point2f>  query_pts;
		for(size_t i=0; i<pc.points.size(); i++)
		{
			cv::Point2f pt;
			pt.x = pc.points[i].x;
			pt.y = pc.points[i].y;
			query_pts.push_back(pt);
		}
		return findBestTf(query_pts);
	}

	template <class T>
	double getScore(vector<T> &pc)
	{
		vector<cv::Point>  query_pts;
		for(size_t i=0; i<pc.size(); i++)
		{
			cv::Point pt;
			cv::Point2f pt2f;
			pt2f.x = pc[i].x;
			pt2f.y = pc[i].y;
			pt = rm2_.imageCoordinate(pt2f);
			query_pts.push_back(pt);
		}

		return rm2_.scorePoints(query_pts, 0,0, false);
	}


private:
	RasterMapImage rm_, rm2_, rm3_;

	transform_info findBestTf(vector<cv::Point2f>& query_pts)
	{
		fmutil::Stopwatch sw;
		sw.start("New matching apporach");

		//has to be very careful with quantization error. Rule of thumb, keep ratio between step size and resolution as close to 1 as possible
		cv::Mat covariance;
		fmutil::Stopwatch sw_sub;
		bool show_time = true;
		sw_sub.start("1");
		//cout<<query_pts.size()<<endl;


		//7x4 = 28
		//becareful, the first pass is important to avoid falling into local minimal
		transform_info best_info;
		vector<transform_info> best_first_pass, best_sec_pass;
		best_first_pass = rm_.searchRotations(query_pts, 10.0, 2.0, M_PI, M_PI/10., best_info, 10, false);
		//best_info = rm_.searchRotation(query_pts, 16.0, 4.0, M_PI, M_PI/4., best_info, false);
		//best_info = rm_.searchRotation(query_pts, 4.0, 2.0, 0, M_PI/4, best_info, false);
		//best_info = rm_.searchRotation(query_pts, 1.0, 0.2, M_PI/8, M_PI/16., best_info, false);
		//cout<<"Best translation "<<best_info.translation_2d<<" "<<best_info.rotation<<" with score "<<best_info.score<<endl;

		sw_sub.end(show_time);
		sw_sub.start("2");
		for(size_t i=0; i<best_first_pass.size(); i++)
			best_sec_pass.push_back(rm2_.searchRotation(query_pts, 1.0, 0.2, M_PI/20, M_PI/60., best_first_pass[i], true));
		//for(size_t k=0; k<best_sec_pass.size(); k++)
		//	cout<<best_sec_pass[k].translation_2d << " "<<best_sec_pass[k].rotation<<": "<<best_sec_pass[k].score<<endl;
		covariance = best_sec_pass[0].covariance;
		sort(best_sec_pass.begin(), best_sec_pass.end(), sortScore);


		best_info = best_sec_pass[0];

		//cout<<"Best translation "<<best_info.translation_2d<<" "<<best_info.rotation<<" with score "<<best_info.score<<endl;
		sw_sub.end(show_time);

		sw_sub.start("3");
		//best_info = rm3_.searchRotation(query_pts, 0.12, 0.02, M_PI/96., M_PI/768., best_info, false);
		best_info = rm3_.searchRotation(query_pts, 0.12, 0.06, M_PI/120., M_PI/720., best_info, false);
		best_info = rm3_.searchRotation(query_pts, 0.06, 0.01, 0, M_PI/720., best_info, false);
		sw_sub.end(show_time);
		//cout<<"Best translation "<<best_info.translation_2d<<" "<<best_info.rotation<<" with score "<<best_info.score<<endl;

		best_info.real_pts.resize(query_pts.size());
		double cos_val = cos(best_info.rotation);
		double sin_val = sin(best_info.rotation);
		for(size_t i=0; i<query_pts.size();i++)
		{
			double rot_x = query_pts[i].x, rot_y = query_pts[i].y;
			cv::Point2f rot_pt;
			rot_pt.x = cos_val * rot_x - sin_val * rot_y;
			rot_pt.y = sin_val * rot_x + cos_val * rot_y;
			best_info.real_pts[i] = rot_pt;
			best_info.real_pts[i].x += best_info.translation_2d.x;
			best_info.real_pts[i].y += best_info.translation_2d.y;
		}
		best_info.covariance = covariance;
		sw.end(show_time);
		return best_info;
	}
};
