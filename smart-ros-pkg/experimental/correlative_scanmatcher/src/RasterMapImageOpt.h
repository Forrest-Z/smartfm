
#include "fmutil/fm_stopwatch.h"
#include <cv.h>
#include <highgui.h>
#include "fmutil/fm_math.h"




#include "dbgstream.h"
using namespace std;                    // make std:: accessible

class transform_info
{
public:
	cv::Point2f translation_2d;
	double rotation;
	double score;
	vector<cv::Point> pts;
	vector<cv::Point2f> real_pts;

	cv::Mat covariance;
	//to support variance calculation
	vector<double> score_vec;
	vector<cv::Point3f> evaluated_pts;

	inline bool operator == (const transform_info &b) const
	{
		bool match = (b.translation_2d.x == translation_2d.x) &&
				(b.translation_2d.y==translation_2d.y) &&
				(b.rotation == rotation);
		//if(match)
		//	dbg<<"Found match "<<b.translation_2d <<" "<<translation_2d<<" "<<b.rotation<<" "<<rotation<<endl;
		return match;
	}

};

bool sortScore (transform_info t1, transform_info t2)
{
	return (t1.score > t2.score);
}

class RasterMapImage
{
public:

	cv::Mat image_;
	dbgstream dbg;
	RasterMapImage(double resolution, double range_covariance): res_(resolution), range_covariance_(range_covariance),
			max_dist_ (sqrt(log(255)*range_covariance)),
			gausian_length_((int) (max_dist_ / res_ + 1)),
			min_pt_(1e99,1e99), max_pt_(-1e99,-1e99)
	{
		gaussian_mapping_ = makeGaussianLinearMapping();
	}

	~RasterMapImage()
	{

	}

	template <class T>
	inline cv::Point imageCoordinate(T pt)
	{
		return cv::Point(round((pt.x-min_pt_.x)/res_), round(image_.rows -(pt.y-min_pt_.y)/res_));
	}

	inline cv::Point2f realCoordinate(cv::Point pt)
	{
		return cv::Point2f(pt.x*res_+min_pt_.x, (image_.rows - pt.y)* res_ + min_pt_.y);
	}

	inline double scorePoints(vector<cv::Point> &search_pt, int offset_x, int offset_y, bool within_prior)
	{
		//fmutil::Stopwatch sw("scorePoints");

		assert(image_.data != NULL);

		uint score = 0;
		int count = 0;
		uint penalize_pt = 100;
		//it only takes 25 ms for 6k loops on 0.03 res

		for(vector<cv::Point>::iterator i=search_pt.begin(); i!=search_pt.end(); i++)
		{
			cv::Point pt = *i;// cv::Point(0,0);// imageCoordinate(search_pt[i]);
			//penalized each points fall outside of map to zero
			pt.x += offset_x;
			pt.y -= offset_y;

			if(outsideMap(image_, pt))
			{
				//it forces the alignment with the prior, which might not be correct
				//got to be careful
				//if(score>=penalize_pt) score -= penalize_pt;
				continue;
			}
			int score_temp = getPixel(pt.x, pt.y);
			if(score_temp == 1)
			{
				//this has caused mismatch of the EA car park area. Perhaps not a good idea?
				if(score>=penalize_pt) score -= penalize_pt;
			}
			else score += score_temp;
			count++;
		}

		double norm_score = (double)score/search_pt.size()*100/255;
		if(within_prior) norm_score = (double)score/count*100/255;
		//dbg<<"Score = "<<score/255*100<<"%"<<endl;
		//sw.end();
		return norm_score;
	}
	template <class T>
	void getInputPoints(vector<T> const &raster_pt_input)
	{
		fmutil::Stopwatch sw;

		sw.start("Raster Map");

		//very fast process, only needs 5 ms on 1620 points with 1000 loops
		vector<cv::Point2f> search_pt;search_pt.resize(raster_pt_input.size());
		for(size_t i=0; i<raster_pt_input.size(); i++)
		{
			search_pt[i].x = raster_pt_input[i].x;
			search_pt[i].y = raster_pt_input[i].y;
		}
		fmutil::Stopwatch sw2("getInputPoints downsample");
		dbg<<"before downsample: "<<search_pt.size()<<endl;
		vector<cv::Point2f> raster_pt = search_pt;// pcl_downsample(search_pt, res_/2., res_/2., res_/2.);
		dbg<<"after downsample: "<<raster_pt.size()<<endl;
		sw2.end(false);

		for(size_t i=0; i<raster_pt.size(); i++)
		{
			if(raster_pt[i].x < min_pt_.x) min_pt_.x = raster_pt[i].x;
			if(raster_pt[i].x > max_pt_.x) max_pt_.x = raster_pt[i].x;

			if(raster_pt[i].y < min_pt_.y) min_pt_.y = raster_pt[i].y;
			if(raster_pt[i].y > max_pt_.y) max_pt_.y = raster_pt[i].y;
		}

		//dbg << "MinMax "<<min_pt_ << " " <<max_pt_<<endl;
		cv::Point2f map_size(max_pt_.x - min_pt_.x, max_pt_.y - min_pt_.y);
		map_size.x = ceil(map_size.x/res_); map_size.y =ceil(map_size.y/res_);

		//morph may cause a single pixel in some very special case, hence minimum value of the raster size should be 1x1 pixel
		if(map_size.x<1) map_size.x = 1;
		if(map_size.y<1) map_size.y = 1;
		//dbg << "Size "<<map_size<<endl;
		//lost about 10ms when using 32F instead of 8U, and total of 45 ms if draw circle function is called, perhaps too much
		image_ = cv::Mat::ones( (int) map_size.y, (int) map_size.x  , CV_8UC1);


		//no point performing gaussian and draw circle when resolution is too low
		//if(res_<=0.1)
		{
			fmutil::Stopwatch sw_draw("Confirming rastermap time");
			for(int j=gaussian_mapping_.size()-1; j>=0; j--)
			{
				//openmp helps reduce the rastering time from 150+ to 60+ms
				//#pragma omp parallel for
				for(size_t i=0; i<raster_pt.size(); i++)
				{
					cv::Point pt =imageCoordinate(raster_pt[i]);
					if(pt.x >= image_.cols || pt.y >= image_.rows) continue;

					this->rasterCircle(pt, j, image_, gaussian_mapping_[j]);

				}
				//dbg<<gaussian_mapping_[j]<<" ";

			}
			sw_draw.end(false);
		}
		/*else
		{
			for(size_t i=0; i<raster_pt.size(); i++)
			{
				cv::Point pt =imageCoordinate(raster_pt[i]);
				if(pt.x >= image_.cols || pt.y >= image_.rows) continue;
				setPixel(pt.x, pt.y, image_, 255);
			}
		}*/
		//dbg<<endl;
		sw.end(false);
		stringstream ss;
		ss<<"rastered_map_"<<res_<<"_"<< range_covariance_<<".png";

		//cv::imwrite(ss.str(), image_);
	}




	vector<transform_info> searchRotations(vector<cv::Point2f> const &search_pt, double translate_range, double translate_step, double rot_range, double rot_step, transform_info const &initialization, int eva_top_count, bool est_cov, bool within_prior=false)
	{
		return searchRotations(search_pt, translate_range, translate_range, translate_step, rot_range, rot_step, initialization, eva_top_count, est_cov, within_prior);
	}

	vector<transform_info> searchRotations(vector<cv::Point2f> const &search_pt, double translate_rangex, double translate_rangey, double translate_step, double rot_range, double rot_step, transform_info const &initialization, int eva_top_count, bool est_cov, bool within_prior=false)
	{

		//translate range is directly corresponds to number of cells in the grid
		//precalculate cosine
		fmutil::Stopwatch sw_init, sw_end,sw;
		sw_init.start("searchRotate init");
		vector<double> cos_vals, sin_vals, rotations;
		for(double i=initialization.rotation-rot_range; i<=initialization.rotation+rot_range; i+=rot_step)
		{
			cos_vals.push_back(cos(i));
			sin_vals.push_back(sin(i));

			//just skip through the same rotation;
			if(i == M_PI && rot_range == M_PI) continue;

			rotations.push_back(i);
		}

		vector<transform_info> best_rotates;

		transform_info best_rotate;

		//#pragma omp parallel for
		vector<cv::Point> rotated_search_pt;

		//careful with downsampling. May lost matching with a single thin line
		vector<cv::Point2f> query_pts_downsample = search_pt;//pcl_downsample(search_pt, translate_step/2., translate_step/2., translate_step/2.);
		//search_pt = query_pts_downsample;
		rotated_search_pt.resize(query_pts_downsample.size());
		sw_init.end(false);
		sw.start("calculate");
		int rangex = translate_rangex / res_;
		int rangey = translate_rangey / res_;
		int step = translate_step / res_;

		cv::Mat K = cv::Mat::zeros(3,3,CV_32F), u = cv::Mat::zeros(3,1,CV_32F);
		//eva_top_count = 10;
		vector<transform_info> best_trans_s;


		for(size_t i=0; i<rotations.size(); i++)
		{

			//rotate each point with by using the rotation center at the sensor's origin
			for(size_t j=0; j<query_pts_downsample.size(); j++)
			{
				double rot_x = query_pts_downsample[j].x, rot_y = query_pts_downsample[j].y;
				cv::Point2f rot_pt;
				rot_pt.x = cos_vals[i] * rot_x - sin_vals[i] * rot_y;
				rot_pt.y = sin_vals[i] * rot_x + cos_vals[i] * rot_y;
				rotated_search_pt[j] = imageCoordinate(rot_pt);
			}

			vector<transform_info> best_trans_s_temp;
			//dbg<<rotations[i]<<": "<<endl;
			best_trans_s_temp = searchTranslations(rotated_search_pt, rangex, rangey, step, initialization.translation_2d, rotations[i], est_cov, within_prior);
			//dbg<<endl;
			best_trans_s.insert(best_trans_s.end(), best_trans_s_temp.begin(), best_trans_s_temp.end());

			if(est_cov)
			{
				best_rotate.evaluated_pts.insert(best_rotate.evaluated_pts.end(), best_trans_s_temp[0].evaluated_pts.begin(), best_trans_s_temp[0].evaluated_pts.end());
				best_rotate.score_vec.insert(best_rotate.score_vec.end(), best_trans_s_temp[0].score_vec.begin(), best_trans_s_temp[0].score_vec.end());
			}
		}
		sw.end(false);
		sw_end.start("end");
		//dbg<<"Total time spent in searchTranslation = "<<sw.total_/1000.0<<endl;



		if(eva_top_count != -1)
		{
			sort(best_trans_s.begin(), best_trans_s.end(), sortScore);
			best_trans_s.resize(eva_top_count);
		}

		best_rotate.rotation = best_trans_s[0].rotation;
		best_rotate.translation_2d = best_trans_s[0].translation_2d;
		if(est_cov)
			best_trans_s[0].covariance = getCovariance(best_rotate);

		sw_end.end(false);
		return best_trans_s;

	}

	transform_info searchRotation(vector<cv::Point2f> const &search_pt, double translate_range, double translate_step, double rot_range, double rot_step, transform_info const &initialization, bool est_cov, bool within_prior=false)
	{

		vector<transform_info> best_rotates = searchRotations(search_pt, translate_range, translate_step, rot_range, rot_step, initialization, 1, est_cov, within_prior);

		transform_info best_rotate = best_rotates[0];

		return best_rotate;
	}

	cv::Mat getCovariance(transform_info &best_info)
	{
		cv::Mat x_i = cv::Mat::zeros(3,1, CV_32F);
		cv::Mat K,u;
		double s = 0.0;
		K = cv::Mat::zeros(3,3, CV_32F);
		u = cv::Mat::zeros(3,1, CV_32F);

		for(size_t i=0; i<best_info.evaluated_pts.size(); i++)
		{
			x_i.at<float>(0,0) = best_info.evaluated_pts[i].x;// - best_info.translation_2d.x;
			x_i.at<float>(1,0) = best_info.evaluated_pts[i].y;// - best_info.translation_2d.y;
			x_i.at<float>(2,0) = best_info.evaluated_pts[i].z;
			double score = best_info.score_vec[i];
			K += x_i * x_i.t() * score;
			u += x_i * score;
			s += score;
		}

		cv::Mat cov = K/s - (u * u.t())/(s*s);
		//dbg<<cov<<endl;

		return cov;
	}

	vector<transform_info> searchTranslations(vector<cv::Point> &search_pt, int range, int stepsize, cv::Point2f initial_pt, double rotation, bool est_cov, bool within_prior)
	{
		return searchTranslations(search_pt, range, range, stepsize, initial_pt, rotation, est_cov, within_prior);
	}
	vector<transform_info> searchTranslations(vector<cv::Point> &search_pt, int rangex, int rangey, int stepsize, cv::Point2f initial_pt, double rotation, bool est_cov, bool within_prior)
	{
		//investigate why low bottom of confusion matrix has overall higher score
		//solved: It is a bad idea to normalize the score with only the number of point within the source map

		fmutil::Stopwatch sw, sw1,sw2,sw3;
		sw.start("");
		//result deteriorate when the resolution is more than 0.3. This is due to quantization error when mapping
		//from pts to grid. Need independent control of map grid size and stepping size
		int startx = initial_pt.x/res_ - rangex, endx = initial_pt.x/res_ + rangex;
		int starty = initial_pt.y/res_ - rangey, endy = initial_pt.y/res_ + rangey;
		//parallel seems to only improve marginally, and because the whole vector of results need to
		//be stored into memory, in the end there is no improvement
		//adding omp critical directive eliminate the need to allocate a large memory
		//improved calculation from 0.7 to 0.45, further improvement to 0.4 when searchRotation also run in omp

		//best_info.pts = search_pt;
		//dbg<<startx<<" "<<endx<<" "<<starty<<" "<<endy<<endl;
		//vector<cv::Point> new_search_pt;
		//new_search_pt.resize(search_pt.size());
		vector<transform_info> best_infos;
		for(int j=starty; j<=endy; j+=stepsize)
		{
			for(int i=startx; i<=endx; i+=stepsize)
			{
				//gained about 80 ms when fixed size of new_search_pt is used instead of keep pushing the search pt
				sw1.start("");
				//elimintate this loop gain much needed boost
				/*for(size_t k=0; k<search_pt.size(); k++)
					{
						new_search_pt[k].x = (cv::Point(search_pt[k].x+i, search_pt[k].y-j));
					}*/
				sw1.end(false);
				sw2.start("");
				double cur_score = scorePoints(search_pt, i, j, within_prior);

				sw2.end(false);
				sw3.start("");
				//dbg<<"offset "<<new_search_pt[0]<<" score "<<cur_score<<endl;
				//dbg<<cur_score<<" ";
				transform_info record_score;
				record_score.translation_2d.x = i*res_;
				record_score.translation_2d.y = j*res_;
				record_score.rotation = rotation;
				record_score.score = cur_score;
				best_infos.push_back(record_score);

				if(est_cov)
				{
					//all calculation for covariance is stored on the first item of the vector
					best_infos[0].score_vec.push_back(cur_score);
					cv::Point3f evaluated_pt;
					evaluated_pt.x = i*res_;
					evaluated_pt.y = j*res_;
					evaluated_pt.z = rotation;
					best_infos[0].evaluated_pts.push_back(evaluated_pt);
				}
				sw3.end(false);

			}
			//dbg<<endl;
		}
		sw.end(false);
		/*double t1 = sw1.total_/1000.0;
			double t2 = sw2.total_/1000.0;
			double t3 = sw3.total_/1000.0;
			dbg<<"Detail: "<<sw.total_/1000.0<<" check:"<<t1<<"+"<<t2<<"+"<<t3<<"="<<t1+t2+t3<<endl;
		 */
		//dbg<<endl;

		return best_infos;
	}

private:
	vector<int> gaussian_mapping_;
	vector<double> circle_segments_cos_, circle_segments_sin_;
	int draw_circle_segments_;
	double res_, range_covariance_, max_dist_, gausian_length_;
	cv::Point2f min_pt_, max_pt_;
	inline bool outsideMap(cv::Mat &img, cv::Point pt)
	{
		if(pt.x >= img.cols || pt.y >= img.rows) return true;
		if(pt.x < 0 || pt.y < 0) return true;
		return false;
	}

	vector<int> makeGaussianLinearMapping()
	{
		vector<int> mapping;
		for(int i=0; i<gausian_length_; i++)
		{
			double d = res_ * i;
			int gaussian_value = (int) (254*exp(-d*d/range_covariance_));
			mapping.push_back(gaussian_value+1);
			//dbg<<d<<":"<<mapping[mapping.size()-1]<<endl;
		}
		//dbg<<endl;
		return mapping;
	}

	inline int getPixel(int x, int y)
	{
		return image_.data[y*image_.cols + x];
	}



	inline void setPixel(int x, int y, cv::Mat &image, int color)
	{
		if(outsideMap(image, cv::Point(x,y))) return;

		//image.at<uchar>(y, x) = color;
		//gain 40ms by direct assignment of color
		image.data[y*image.cols + x] = color;

		if(!outsideMap(image, cv::Point(x-1,y))) image.data[y*image.cols + x -1] = color;
		if(!outsideMap(image, cv::Point(x,y-1))) image.data[(y-1)*image.cols + x] = color;
		if(!outsideMap(image, cv::Point(x+1,y))) image.data[y*image.cols + x + 1] = color;
		if(!outsideMap(image, cv::Point(x,y+1))) image.data[(y+1)*image.cols + x] = color;
	}


	//http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
	//tested with another algorithm that is more assembly code friendly but there is no measureable difference

	void rasterCircle(cv::Point pt, int radius, cv::Mat &image, int color)
	{
		cv::circle(image, pt, radius, cv::Scalar(color,color,color), -1);
		/*
		int x0 = pt.x, y0 = pt.y;
		int f = 1 - radius;
		int ddF_x = 1;
		int ddF_y = -2 * radius;
		int x = 0;
		int y = radius;

		setPixel(x0, y0 + radius, image, color);
		setPixel(x0, y0 - radius, image, color);
		setPixel(x0 + radius, y0, image, color);
		setPixel(x0 - radius, y0, image, color);

		while(x < y)
		{
			// ddF_x == 2 * x + 1;
			// ddF_y == -2 * y;
			// f == x*x + y*y - radius*radius + 2*x - y + 1;
			if(f >= 0)
			{
				y--;
				ddF_y += 2;
				f += ddF_y;
			}
			x++;
			ddF_x += 2;
			f += ddF_x;
			setPixel(x0 + x, y0 + y, image, color);
			setPixel(x0 - x, y0 + y, image, color);
			setPixel(x0 + x, y0 - y, image, color);
			setPixel(x0 - x, y0 - y, image, color);
			setPixel(x0 + y, y0 + x, image, color);
			setPixel(x0 - y, y0 + x, image, color);
			setPixel(x0 + y, y0 - x, image, color);
			setPixel(x0 - y, y0 - x, image, color);
		}*/
	}


};

