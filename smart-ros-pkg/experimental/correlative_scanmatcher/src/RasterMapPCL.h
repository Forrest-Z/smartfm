/*
 * RasterMapPCL.h
 *
 *  Created on: Nov 28, 2012
 *      Author: demian
 */

#include "RasterMapImageOpt.h"
#include "renderMap.h"
#include <sensor_msgs/PointCloud.h>

class RasterMapPCL
{
public:
	//changing the 1st RasterMap resolution and range from 0.2,0.5 to 0.1,0.1 seems to miss some
	//good potential close loop while increasing false detection
	//apparent decreasing the range that causes the above scenario
	dbgstream dbg;
	//rastering map with too low resolution causes more noise than desired. 1.0 is the bare minimum resolution that is acceptable
	RasterMapPCL(): rm_(1.0, 0.05), rm2_(0.25, 0.05), rm3_(0.03, 0.05)
	{
		//dbg.enable_output();
	};


	template <class T>
	void setInputPts(vector<T> &pc, bool verification=false)
	{

		RenderMap rm;
		vector<geometry_msgs::Point32> pc_pts;
		pc_pts.resize(pc.size());
		for(size_t i=0; i<pc.size(); i++)
		{
			pc_pts[i].x = pc[i].x;
			pc_pts[i].y = pc[i].y;
		}
		rm.drawMap(pc_pts, 0.02);
		vector<cv::Point2f>  query_pts = rm.mapToRealPts();

		vector<T> pt_proc;
		pt_proc.resize(query_pts.size());

		for(size_t i=0; i<query_pts.size(); i++)
		{
			pt_proc[i].x = query_pts[i].x;
			pt_proc[i].y = query_pts[i].y;
		}
		rm2_.getInputPoints(pt_proc);

		if(!verification)
		{
			rm_.getInputPoints(pt_proc);
			rm3_.getInputPoints(pt_proc);

		}
	}

	transform_info getBestTf(sensor_msgs::PointCloud &pc)
	{
		RenderMap rm;
		rm.drawMap(pc.points, 0.02);
		vector<cv::Point2f>  query_pts = rm.mapToRealPts();


		/*for(size_t i=0; i<pc.points.size(); i++)
		{
			cv::Point2f pt;
			pt.x = pc.points[i].x;
			pt.y = pc.points[i].y;
			query_pts.push_back(pt);
			cout<<pt.x<<" "<<pt.y<<endl;
		}*/
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

	cv::Mat getCovarianceWithTf(sensor_msgs::PointCloud &pc, transform_info best_tf)
	{
		vector<cv::Point2f>  query_pts;
		for(size_t i=0; i<pc.points.size(); i++)
		{
			cv::Point2f pt;
			pt.x = pc.points[i].x;
			pt.y = pc.points[i].y;
			query_pts.push_back(pt);
		}

		vector<transform_info> best_tf_temp = rm_.searchRotations(query_pts, 10.0, 1.0, M_PI, M_PI/30., best_tf, -1, true);// rm2_.searchRotations(query_pts, 1.0, 0.2, M_PI/45, M_PI/90., best_tf, -1, true);
		//complain about positive definite and crashes with inf residual
		for(int i=0; i<3; i++)
			for(int j=0; j<3; j++)
				best_tf_temp[0].covariance.at<float>(i,j) = fabs(best_tf_temp[0].covariance.at<float>(i,j));
		dbg<< best_tf_temp[0].covariance<<endl;
		return best_tf_temp[0].covariance;//cv::Mat::eye(3,3, CV_32F)*100;
	}
private:
	RasterMapImage rm_, rm2_, rm3_;




	void removeRepeatedEntry(vector<transform_info> &tf)
	{
		for(size_t i=1; i<tf.size();)
		{
			cv::Point2f p1 = tf[i-1].translation_2d, p2 = tf[i].translation_2d;
			double r1 = tf[i-1].rotation, r2 = tf[i].rotation;
			if(p1.x == p2.x && p1.x == p2.y && fabs(r1-r2)<0.00001)
				{
				dbg<<"match found "<<p1<<" "<<p2<<" "<<r1<<" "<<r2<<" "<<endl;
				tf.erase(tf.begin() + i);
				}
			else i++;
		}
		/*std::vector<transform_info>::iterator end_pos( tf.end() );

		std::vector<transform_info>::iterator start_pos( tf.begin() );

		transform_info value( *start_pos );
		while (++start_pos != end_pos)
		{
			end_pos = std::remove( start_pos, end_pos, value );
			value = *start_pos;
		}
		tf.erase( end_pos, tf.end() );*/
	}


	transform_info findBestTf(vector<cv::Point2f>& query_pts)
	{
		fmutil::Stopwatch sw;
		sw.start("New matching apporach");

		//has to be very careful with quantization error. Rule of thumb, keep ratio between step size and resolution as close to 1 as possible
		cv::Mat covariance;
		fmutil::Stopwatch sw_sub;
		bool show_time = false;
		sw_sub.start("1");
		//dbg<<query_pts.size()<<endl;


		//7x4 = 28
		//becareful, the first pass is important to avoid falling into local minimal
		transform_info best_info;
		//very important initialization
		best_info.rotation = 0.0;
		vector<transform_info> best_first_pass_a, best_first_pass_b, best_sec_pass, best_thi_pass;


		best_first_pass_a = rm_.searchRotations(query_pts, 11.0, 1.0, M_PI, M_PI/10., best_info, -1, false);

		sort(best_first_pass_a.begin(), best_first_pass_a.end(), sortScore);
		size_t first_pass_idx=1;
		double first_score = best_first_pass_a[0].score;
		for(; first_pass_idx<best_first_pass_a.size(); first_pass_idx++)
		{
			if(best_first_pass_a[first_pass_idx].score<(first_score-20.) || best_first_pass_a[first_pass_idx].score < 15.) break;

		}
		best_first_pass_a.resize(first_pass_idx);

		sort(best_first_pass_a.begin(), best_first_pass_a.end(), sortScore);
		//remove the repeated entry
		//dbg<<"Before removed "<<best_first_pass_b.size()<<endl;
		//removeRepeatedEntry(best_first_pass_b);

		//dbg<<"Size of first pass "<<best_first_pass_b.size()<<endl;

		sw_sub.end(show_time);
		sw_sub.start("2");

		for(size_t i=0; i<best_first_pass_a.size(); i++)
		{
			dbg<<i<<": "<<best_first_pass_a[i].translation_2d<<" "<<best_first_pass_a[i].rotation<<" "<<best_first_pass_a[i].score<<endl;
			vector<transform_info> best_tf_temp = rm2_.searchRotations(query_pts, 0.5, 0.25, M_PI/20, M_PI/60., best_first_pass_a[i], -1, false);
			best_sec_pass.insert(best_sec_pass.end(),  best_tf_temp.begin(), best_tf_temp.end());
		}
		sort(best_sec_pass.begin(), best_sec_pass.end(), sortScore);

		size_t sec_pass_idx = 0;

		best_sec_pass.resize(1);

		sw_sub.end(show_time);
		sw_sub.start("3");

		vector<transform_info> best_thi_tf;

		for(size_t i=0; i<best_sec_pass.size(); i++)
		{
			dbg<<i<<"/"<<best_sec_pass.size()<<": "<<best_sec_pass[i].translation_2d<<" "<<best_sec_pass[i].rotation<<" "<<best_sec_pass[i].score<<endl;
			vector<transform_info> best_tf_temp = rm3_.searchRotations(query_pts, 0.12, 0.03, M_PI/120, M_PI/720., best_sec_pass[i], -1, false);
			best_thi_tf.insert(best_thi_tf.end(), best_tf_temp.begin(), best_tf_temp.end());
			//keeping all the points is useless here, and it may cause bad alloc due to huge memory requirement
			//best_thi_pass.insert(best_thi_pass.end(),  best_tf_temp.begin(), best_tf_temp.end());
		}
		dbg<<"end best_sec_pass loop total third pass size="<<best_thi_tf.size()<<endl;
		sort(best_thi_tf.begin(), best_thi_tf.end(), sortScore);

		best_info = best_thi_tf[0];

		sw_sub.end(show_time);
		//dbg<<"Best translation "<<best_info.translation_2d<<" "<<best_info.rotation<<" with score "<<best_info.score<<endl;

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

		best_first_pass_a = rm_.searchRotations(query_pts, 4.0, 1.0, M_PI/10, M_PI/45., best_info, -1, true);
		covariance = best_first_pass_a[0].covariance;

		best_info.covariance = covariance;
		sw.end(show_time);

		//some house keeping
		best_sec_pass.clear();
		best_thi_pass.clear();

		//take care case where |rotatation|> M_PI
		if(best_info.rotation > M_PI) best_info.rotation = best_info.rotation - 2 * M_PI;
		else if(best_info.rotation < -M_PI) best_info.rotation = best_info.rotation + 2 * M_PI;
		return best_info;
	}
};
