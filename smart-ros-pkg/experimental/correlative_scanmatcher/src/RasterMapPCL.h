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
	RasterMapPCL(): rm_(0.2, 1.0), rm2_(0.1, 0.1), rm3_(0.01, 0.008)
	{};

	void setInputPts(sensor_msgs::PointCloud &pc)
	{
		vector<cv::Point2f> raster_pts;
		for(size_t i=0; i<pc.points.size(); i++)
		{
			cv::Point2f pt;
			pt.x = pc.points[i].x;
			pt.y = pc.points[i].y;
			raster_pts.push_back(pt);
		}
		rm_.getInputPoints(raster_pts);
		rm2_.getInputPoints(raster_pts);
		rm3_.getInputPoints(raster_pts);
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

private:
	RasterMapImage rm_, rm2_, rm3_;

	transform_info findBestTf(vector<cv::Point2f>& query_pts)
	{
		fmutil::Stopwatch sw;
		sw.start("New matching apporach");

		//has to be very careful with quantization error. Rule of thumb, keep ratio between step size and resolution as close to 1 as possible
		cv::Mat covariance;
		fmutil::Stopwatch sw_sub;
		bool show_time = false;
		sw_sub.start("1");
		//cout<<query_pts.size()<<endl;
		transform_info best_info;
		best_info.rotation = 0.0;

		//7x4 = 28
		//becareful, the first pass is important to avoid falling into local minimal

		best_info = rm_.searchRotation(query_pts, 14.0, 2.0, M_PI, M_PI/10., best_info, false);
		//best_info = rm_.searchRotation(query_pts, 16.0, 4.0, M_PI, M_PI/4., best_info, false);
		//best_info = rm_.searchRotation(query_pts, 4.0, 2.0, 0, M_PI/4, best_info, false);
		//best_info = rm_.searchRotation(query_pts, 1.0, 0.2, M_PI/8, M_PI/16., best_info, false);
		//cout<<"Best translation "<<best_info.translation_2d<<" "<<best_info.rotation<<" with score "<<best_info.score<<endl;

		sw_sub.end(show_time);
		sw_sub.start("2");
		best_info = rm2_.searchRotation(query_pts, 1.2, 0.4, M_PI/10., M_PI/60., best_info, true);
		covariance = best_info.covariance;
		best_info = rm2_.searchRotation(query_pts, 0.4, 0.2, 0., M_PI/60., best_info, false);

		//cout<<"Best translation "<<best_info.translation_2d<<" "<<best_info.rotation<<" with score "<<best_info.score<<endl;
		sw_sub.end(show_time);

		sw_sub.start("3");
		//best_info = rm3_.searchRotation(query_pts, 0.12, 0.02, M_PI/96., M_PI/768., best_info, false);
		best_info = rm3_.searchRotation(query_pts, 0.12, 0.06, M_PI/120., M_PI/720., best_info, false);
		best_info = rm3_.searchRotation(query_pts, 0.06, 0.01, 0, M_PI/720., best_info, false);
		sw_sub.end(show_time);
		//cout<<"Best translation "<<best_info.translation_2d<<" "<<best_info.rotation<<" with score "<<best_info.score<<endl;

		best_info.real_pts.resize(best_info.pts.size());
		for(size_t i=0; i<best_info.pts.size();i++)
		{
			best_info.real_pts[i] = rm3_.realCoordinate(best_info.pts[i]);
			best_info.real_pts[i].x += best_info.translation_2d.x;
			best_info.real_pts[i].y += best_info.translation_2d.y;
		}
		best_info.covariance = covariance;
		sw.end(show_time);
		return best_info;
	}
};
