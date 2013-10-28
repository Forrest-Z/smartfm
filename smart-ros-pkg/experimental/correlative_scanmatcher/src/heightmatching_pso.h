/*
 * heightmatching_pso.h
 *
 *  Created on: Jan 11, 2013
 *      Author: demian
 */

//#include "RasterMapImageOpt.h"
#include "renderMap.h"
template< int dimension>
class HeightMatchingProblem
{
public:
	static const int nb_parameters = dimension;
	static int count;

	static cv::Mat src_, dst_, base_img_;
	static double max_score_;
	static map<int, cv::Mat> expand_img_sat_rois_;
	static vector<cv::Mat> rotated_img_;
	static double rot_res_;
	static double trans_res_;
	static fmutil::Stopwatch rot_time_;
	static cv::Mat rotateCanvasOpt(int rot)
	{

		cv::Mat rot_base_img;
		cv::Mat rotationMat = cv::getRotationMatrix2D(cv::Point2f(base_img_.cols/2, base_img_.rows/2), rot, 1.0);

		cv::warpAffine(base_img_, rot_base_img, rotationMat, base_img_.size());

		/*cv::Mat expand_img = cv::Mat::zeros(cv::Size(newSize+dst_.cols, newSize+dst_.rows), src_.type());
		roi = expand_img(cv::Rect(dst_.cols/2, dst_.rows/2, newSize, newSize));
		base_img.copyTo(roi);*/
		int size_of_matching_x = 40 + dst_.cols;
		int size_of_matching_y = 40 + dst_.rows;
		cv::Mat expand_img = rot_base_img(cv::Rect((base_img_.cols - size_of_matching_x)/2., (base_img_.rows - size_of_matching_y)/2., size_of_matching_x, size_of_matching_y));

		return expand_img;
	}

	static void init(sensor_msgs::PointCloud src, sensor_msgs::PointCloud dst)
	{
		init(src.points, dst.points);
	}

	static void init(vector<cv::Mat> &src_imgs, cv::Mat &dst_img)
	{
		trans_res_ = 0.2;
		rot_res_ = 0.5;
		rotated_img_ = src_imgs;

		dst_ = dst_img;
		max_score_ = cv::sum(dst_)[0];
	}
	static void init(vector<geometry_msgs::Point32> src, vector<geometry_msgs::Point32> dst, bool write_image=false, int image_idx=0)
	{
		fmutil::Stopwatch sw_img("start rotate img");
		RenderMap rm_src_img;
		trans_res_ = 0.2;
		rot_res_ = 0.5;
		rm_src_img.drawHeightMap(src, trans_res_, 140, 140);
		vector<cv::Mat> rotated_img;
		rotated_img.resize(ceil(360/rot_res_)+1);
		int begin_rotation = -180/rot_res_;
		int end_rotation = 180/rot_res_;
#pragma omp parallel for
		for(int i=begin_rotation; i<end_rotation; i++)
		{
			cv::Mat rotationMat = cv::getRotationMatrix2D(cv::Point2f(rm_src_img.image_.cols/2, rm_src_img.image_.rows/2), i*rot_res_, 1.0);
			cv::Mat rot_base_img;
			cv::warpAffine(rm_src_img.image_, rot_base_img, rotationMat, rm_src_img.image_.size(),cv::INTER_NEAREST);
			rotated_img[i-begin_rotation] = rot_base_img*2;
			//cv::imshow("rot_base_img", rot_base_img);
				//cv::waitKey();
			if(write_image)
			{
				stringstream ss;
				ss<<"src_"<<setfill('0')<<setw(5)<<image_idx<<"_"<<setfill('0')<<setw(4)<<i-begin_rotation<<".png";
				cv::imwrite(ss.str(), rot_base_img);
			}
		}

		RenderMap rm_dst;
		rm_dst.drawHeightMap(dst, trans_res_, 100, 100);
		rotated_img_ = rotated_img;
		dst_ = rm_dst.image_;
		max_score_ = cv::sum(dst_)[0];
		//cv::imshow("dst_base_img", dst_);
		//cv::waitKey();
		if(write_image)
		{
			stringstream ss;
			ss<<"dst_"<<setfill('0')<<setw(5)<<image_idx<<".png";
			cv::imwrite(ss.str(), dst_);
		}
		sw_img.end();
	}
	static void init(cv::Mat input, cv::Mat dst)
	{

		count = 0;
		src_ = input;
		dst_ = dst;
		max_score_ = cv::sum(dst_)[0];

		int newSize = ceil(sqrt(src_.cols*src_.cols + src_.rows*src_.rows));
		base_img_ = cv::Mat::zeros(cv::Size(newSize,newSize), src_.type());
		cv::Mat roi = base_img_(cv::Rect((newSize-src_.cols)/2, (newSize-src_.rows)/2, src_.cols, src_.rows));
		src_.copyTo(roi);
		rot_time_.start("rotation");
		//#pragma omp parallel for
		for(int i=-180; i<=180; i++)
		{

			int newSize = ceil(sqrt(src_.cols*src_.cols + src_.rows*src_.rows));
			cv::Mat expand_img = rotateCanvasOpt(i)*2;
			//cv::Mat expand_img_sat = expand_img * 2;

			//#pragma omp critical
			expand_img_sat_rois_[i] = expand_img;

		}
		rot_time_.end(false);
	}

	static void free(void)
	{
	}

	static double get_lbound(int index)
	{
		if(index == 2) return -180.;
		else return 0;
	}

	static double get_ubound(int index)
	{
		if(index == 2) return 180;
		else return 40;
	}

	static bool stop(double fitness, int epoch)
	{
		//it seems to have a lower probability to reach global optimum when count is used.
		//merely co-incidents?
		return epoch>1000;// fitness<0.43;// ;
	}

	static double evaluate(void * x)
	{
		double * params = (double*) x;
		count++;
		double fit = 0.0;



		int param_0 = round(params[0]/trans_res_);
		int param_1 = round(params[1]/trans_res_);

		int param_2 = round((params[2]+180)/rot_res_);
		bool zero_boundary = param_0 >= 0 && param_1 >= 0;
		//redesign this portion for more flexible matching
		bool within_col_boundary = param_0 + dst_.cols <= rotated_img_[0].cols;
		bool within_row_boundary = param_1 + dst_.rows <= rotated_img_[0].rows;
		bool within_rotation = param_2 >= 0 && param_2 <= round(180/rot_res_)-1;



		double score = 1.;
		if(zero_boundary && within_col_boundary &&within_row_boundary && within_rotation)
		{
			cv::Mat expand_img_roi;
			cv::Mat raw_substract_img;
			//mapping with double has a very funny effect, it won't work when expand_img_sat_rois_[180.0] is called even when it is assigned
			//back to more reliable int
			expand_img_roi = rotated_img_[param_2](cv::Rect(param_0,param_1, dst_.cols, dst_.rows));
			assert(dst_.cols == dst_.rows);
			int smaller_size = dst_.cols/1.75;
			int corner_value = (dst_.cols-smaller_size)/2;
			raw_substract_img = (dst_(cv::Rect(corner_value, corner_value, smaller_size,smaller_size))-expand_img_roi(cv::Rect(corner_value, corner_value, smaller_size,smaller_size)));
			score = cv::sum(raw_substract_img)[0]/max_score_;
			/*cout<<param_0<<" "<<param_1<<" "<<param_2<<endl;
			cout<<score<<endl;
			cv::imshow("expand_img_roi", expand_img_roi);
			cv::imshow("raw_substract_img", raw_substract_img);
			cv::waitKey();*/
		}


		return score;
	}
};
template<int dimension> int     HeightMatchingProblem<dimension>::count;
template<int dimension> cv::Mat HeightMatchingProblem<dimension>::dst_;
template<int dimension> cv::Mat HeightMatchingProblem<dimension>::src_;
template<int dimension> cv::Mat HeightMatchingProblem<dimension>::base_img_;
template<int dimension> double HeightMatchingProblem<dimension>::max_score_;
template<int dimension> map<int, cv::Mat> HeightMatchingProblem<dimension>::expand_img_sat_rois_;
template<int dimension> fmutil::Stopwatch HeightMatchingProblem<dimension>::rot_time_;
template<int dimension> vector<cv::Mat> HeightMatchingProblem<dimension>::rotated_img_;
template<int dimension> double HeightMatchingProblem<dimension>::rot_res_;
template<int dimension> double HeightMatchingProblem<dimension>::trans_res_;


template< int dimension>
class NormalsAndHeightMatchingProblem
{
public:
	static const int nb_parameters = dimension;
	static int count;
	static RasterMapImage* rm_;//does not name a type??
	static vector<vector<cv::Point> > rotated_dst_;
	static double rotate_res_, trans_res_;
	static void init(cv::Mat &raster_img, cv::Point2f min_pt, cv::Point2f max_pt, vector<geometry_msgs::Point32> &dst)
	{
		//pso easier to fall into local minima when higher rotate_res_ is used
		rotate_res_ = 0.2;
		trans_res_ = 0.1;
		rm_ = new RasterMapImage(trans_res_, 0.03);
		//rm_->getInputPoints(src_norms, src);
		rm_->image_ = raster_img;
		rm_->max_pt_ = max_pt;
		rm_->min_pt_ = min_pt;
		fmutil::Stopwatch rotate_time("rotate time");
		rotated_dst_.resize(360/rotate_res_+1);
		int rotated_dst_idx = 0;
		for(double i=-180; i<180; i+=rotate_res_)
		{
			double cos_val = cos(i/180*M_PI);
			double sin_val = sin(i/180*M_PI);
			vector<cv::Point> rotated_search_pt;
			rotated_search_pt.resize(dst.size());
			for(size_t j=0; j<dst.size(); j++)
			{
				double rot_x = dst[j].x, rot_y = dst[j].y;
				geometry_msgs::Point32 rot_pt;
				rot_pt.x = cos_val * rot_x - sin_val * rot_y;
				rot_pt.y = sin_val * rot_x + cos_val * rot_y;
				rotated_search_pt[j] = rm_->imageCoordinate(rot_pt);
			}
			rotated_dst_[rotated_dst_idx] = rotated_search_pt;
			rotated_dst_idx++;
		}
		rotate_time.end();
	}

	static void init(vector<geometry_msgs::Point32> &src, vector<geometry_msgs::Point32> &src_norms, vector<geometry_msgs::Point32> &dst)
	{
		//pso easier to fall into local minima when higher rotate_res_ is used
		rotate_res_ = 0.2;
		trans_res_ = 0.1;
		rm_ = new RasterMapImage(trans_res_, 0.03);
		rm_->getInputPoints(src_norms, src);
		fmutil::Stopwatch rotate_time("rotate time");
		rotated_dst_.resize(360/rotate_res_+1);
		int rotated_dst_idx = 0;
		for(double i=-180; i<180; i+=rotate_res_)
		{
			double cos_val = cos(i/180*M_PI);
			double sin_val = sin(i/180*M_PI);
			vector<cv::Point> rotated_search_pt;
			rotated_search_pt.resize(dst.size());
			for(size_t j=0; j<dst.size(); j++)
			{
				double rot_x = dst[j].x, rot_y = dst[j].y;
				geometry_msgs::Point32 rot_pt;
				rot_pt.x = cos_val * rot_x - sin_val * rot_y;
				rot_pt.y = sin_val * rot_x + cos_val * rot_y;
				rotated_search_pt[j] = rm_->imageCoordinate(rot_pt);
			}
			rotated_dst_[rotated_dst_idx] = rotated_search_pt;
			rotated_dst_idx++;
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
		return epoch>2000;
	}

	static double evaluate(void * x)
	{
		double * params = (double*) x;
		int offset_x = round(params[0]/trans_res_);
		int offset_y = round(params[1]/trans_res_);
		int rot_idx = round(((params[2]+M_PI)/M_PI*180)/rotate_res_);
		if(rot_idx > rotated_dst_.size()-1 || rot_idx<0) return 1.;
		double score = rm_->scorePoints(rotated_dst_[rot_idx], offset_x, offset_y, false);
		count++;
		return 100. - score;
	}
};
template<int dimension> int     NormalsAndHeightMatchingProblem<dimension>::count;
template<int dimension> RasterMapImage* NormalsAndHeightMatchingProblem<dimension>::rm_;
template<int dimension> vector<vector<cv::Point> > NormalsAndHeightMatchingProblem<dimension>::rotated_dst_;
template<int dimension> double NormalsAndHeightMatchingProblem<dimension>::rotate_res_;
template<int dimension> double NormalsAndHeightMatchingProblem<dimension>::trans_res_;
