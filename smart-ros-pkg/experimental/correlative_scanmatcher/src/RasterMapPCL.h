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

	//rastering map with too low resolution causes more noise than desired. 0.5 is the bare minimum resolution that is acceptable
	RasterMapPCL(): rm_(0.5, 0.005), rm2_(0.1, 0.05), rm3_(0.01, 0.05)
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
		cout<< best_tf_temp[0].covariance<<endl;
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
				cout<<"match found "<<p1<<" "<<p2<<" "<<r1<<" "<<r2<<" "<<endl;
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
		//cout<<query_pts.size()<<endl;


		//7x4 = 28
		//becareful, the first pass is important to avoid falling into local minimal
		transform_info best_info;
		//very important initialization
		best_info.rotation = 0.0;
		vector<transform_info> best_first_pass_a, best_first_pass_b, best_sec_pass, best_thi_pass;


		best_first_pass_a = rm_.searchRotations(query_pts, 10.0, 1.0, M_PI, M_PI/30., best_info, -1, false);

		sort(best_first_pass_a.begin(), best_first_pass_a.end(), sortScore);
		size_t first_pass_idx=0;
		for(; first_pass_idx<best_first_pass_a.size(); first_pass_idx++)
		{
			if(best_first_pass_a[first_pass_idx].score<55) break;
		}
		if(first_pass_idx == 0) best_first_pass_a.resize(1);
		else best_first_pass_a.resize(first_pass_idx);
		/*for(size_t i=0; i<best_first_pass_a.size(); i++)
		{
			cout<<i<<": "<<best_first_pass_a[i].translation_2d<<" "<<best_first_pass_a[i].rotation<<" "<<best_first_pass_a[i].score<<endl;
			//when perform second pass, do not use rotation as it will produce further more local maximal that cannot be handled by small number of top scores
			vector<transform_info> best_tf_temp = rm_.searchRotations(query_pts, 1.0, 0.5, 0, M_PI/60, best_first_pass_a[i], -1, false);
			best_first_pass_b.insert(best_first_pass_b.end(), best_tf_temp.begin(), best_tf_temp.end());
		}*/
		best_first_pass_b = best_first_pass_a;
		sort(best_first_pass_b.begin(), best_first_pass_b.end(), sortScore);
		//remove the repeated entry
		//cout<<"Before removed "<<best_first_pass_b.size()<<endl;
		//removeRepeatedEntry(best_first_pass_b);

		//cout<<"Size of first pass "<<best_first_pass_b.size()<<endl;

		sw_sub.end(show_time);
		sw_sub.start("2");

		for(size_t i=0; i<best_first_pass_b.size(); i++)
		{
			//cout<<i<<": "<<best_first_pass_b[i].translation_2d<<" "<<best_first_pass_b[i].rotation<<" "<<best_first_pass_b[i].score<<endl;
			//crashes while removing repeated entry when resolution of 0.25 is used, weird
			vector<transform_info> best_tf_temp = rm2_.searchRotations(query_pts, 0.5, 0.2, 0, M_PI/90., best_first_pass_b[i], -1, false);
			best_sec_pass.insert(best_sec_pass.end(),  best_tf_temp.begin(), best_tf_temp.end());
		}

		//for(size_t k=0; k<best_sec_pass.size(); k++)
		//	cout<<best_sec_pass[k].translation_2d << " "<<best_sec_pass[k].rotation<<": "<<best_sec_pass[k].score<<endl;

		sort(best_sec_pass.begin(), best_sec_pass.end(), sortScore);
		//cout<<"Removing repeated entry "<<best_sec_pass.size()<<endl;
		//removeRepeatedEntry(best_sec_pass);
		//remove operation is very very slow!!
		//cout<<"After remove "<<best_sec_pass.size()<<endl;

		size_t sec_pass_idx = 0;
		for(; sec_pass_idx<best_sec_pass.size(); sec_pass_idx++)
		{
			if(best_sec_pass[sec_pass_idx].score<50) break;
		}
		if(sec_pass_idx ==0 ) best_sec_pass.resize(1);
		else best_sec_pass.resize(sec_pass_idx);

		//cout<<"Best translation "<<best_info.translation_2d<<" "<<best_info.rotation<<" with score "<<best_info.score<<endl;
		sw_sub.end(show_time);

		sw_sub.start("3");

		transform_info best_thi_tf;
		best_thi_tf.score = -1;
		for(size_t i=0; i<best_sec_pass.size(); i++)
		{
			//cout<<i<<": "<<best_sec_pass[i].translation_2d<<" "<<best_sec_pass[i].rotation<<" "<<best_sec_pass[i].score<<endl;
			vector<transform_info> best_tf_temp = rm3_.searchRotations(query_pts, 0.1, 0.05, M_PI/60, M_PI/360., best_sec_pass[i], -1, false);
			sort(best_tf_temp.begin(), best_tf_temp.end(), sortScore);
			if(best_thi_tf.score < best_tf_temp[0].score)
				best_thi_tf = best_tf_temp[0];
			//keeping all the points is useless here, and it may cause bad alloc due to huge memory requirement
			//best_thi_pass.insert(best_thi_pass.end(),  best_tf_temp.begin(), best_tf_temp.end());
		}




		best_info = best_thi_tf;

		//best_info.translation_2d.x = -6.3;
		//best_info.translation_2d.y = -3.31;
		//best_info.rotation = -3.15032;

		best_info = rm3_.searchRotation(query_pts, 0.025, 0.01, M_PI/720, M_PI/720., best_info, false);
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

		best_first_pass_a = rm_.searchRotations(query_pts, 4.0, 1.0, M_PI/10, M_PI/90., best_info, -1, true);
		covariance = best_first_pass_a[0].covariance;

		best_info.covariance = covariance;
		sw.end(show_time);

		//some house keeping
		best_sec_pass.clear();
		best_thi_pass.clear();

		return best_info;
	}
};
