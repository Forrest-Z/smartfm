#include <fstream>
#include <fmutil/fm_stopwatch.h>
#include <cv.h>
#include <highgui.h>
#include <fmutil/fm_math.h>
using namespace std;                    // make std:: accessible

struct transform_info
{
	cv::Point2f translation_2d;
	double rotation;
	double score;
	vector<cv::Point> pts;
	vector<cv::Point2f> real_pts;
	cv::Mat K,u;
	double s;
	cv::Mat covariance;
};

class RasterMapImage
{
public:

	cv::Mat image_;

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

	inline cv::Point imageCoordinate(cv::Point2f pt)
	{
		return cv::Point((pt.x-min_pt_.x)/res_, (int) image_.rows -(pt.y-min_pt_.y)/res_);
	}

	inline cv::Point2f realCoordinate(cv::Point pt)
	{
		return cv::Point2f(pt.x*res_+min_pt_.x, (image_.rows - pt.y)* res_ + min_pt_.y);
	}

	double scorePoints(vector<cv::Point> search_pt, int offset_x, int offset_y)
	{
		//fmutil::Stopwatch sw;

		assert(image_.data != NULL);

		uint score = 0;


		//it only takes 25 ms for 6k loops on 0.03 res

		for(vector<cv::Point>::iterator i=search_pt.begin(); i!=search_pt.end(); i++)
		{
			cv::Point pt = *i;// cv::Point(0,0);// imageCoordinate(search_pt[i]);
			//penalized each points fall outside of map to zero
			pt.x += offset_x;
			pt.y -= offset_y;
			if(outsideMap(image_, pt)) continue;
			score += getPixel(pt.x, pt.y);

		}

		double norm_score = (double)score/search_pt.size()*100/255;

		//cout<<"Score = "<<score/255*100<<"%"<<endl;
		return norm_score;
	}
	void getInputPoints(vector<cv::Point2f> raster_pt)
	{
		fmutil::Stopwatch sw;

		sw.start("Raster Map");

		//very fast process, only needs 5 ms on 1620 points with 1000 loops

		for(size_t i=0; i<raster_pt.size(); i++)
		{
			if(raster_pt[i].x < min_pt_.x) min_pt_.x = raster_pt[i].x;
			if(raster_pt[i].x > max_pt_.x) max_pt_.x = raster_pt[i].x;

			if(raster_pt[i].y < min_pt_.y) min_pt_.y = raster_pt[i].y;
			if(raster_pt[i].y > max_pt_.y) max_pt_.y = raster_pt[i].y;
		}

		//cout << "MinMax "<<min_pt_ << " " <<max_pt_<<endl;
		cv::Point2f map_size(max_pt_.x - min_pt_.x, max_pt_.y - min_pt_.y);
		map_size.x = ceil(map_size.x/res_); map_size.y =ceil(map_size.y/res_);
		//cout << "Size "<<map_size<<endl;
		//lost about 10ms when using 32F instead of 8U, and total of 45 ms if draw circle function is called, perhaps too much
		image_ = cv::Mat::zeros( (int) map_size.y, (int) map_size.x  , CV_8UC1);


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

		}
		sw.end(false);
		//cv::imwrite("map.png", image_);
	}

	transform_info searchRotation(vector<cv::Point2f> search_pt, double translate_range, double translate_step, double rot_range, double rot_step, transform_info initialization, bool est_cov)
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
			rotations.push_back(i);
		}

		vector<transform_info> best_rotates;

		transform_info best_rotate;
		best_rotate.score = -1e99;
		//#pragma omp parallel for
		vector<cv::Point> rotated_search_pt;
		rotated_search_pt.resize(search_pt.size());
		sw_init.end(false);
		sw.start("calculate");
		int range = translate_range / res_;
		int step = translate_step / res_;

		cv::Mat K = cv::Mat::zeros(3,3,CV_32F), u = cv::Mat::zeros(3,1,CV_32F);
		double s = 0.0;

		for(size_t i=0; i<rotations.size(); i++)
		{

			//rotate each point with by using the rotation center at the sensor's origin


			for(size_t j=0; j<search_pt.size(); j++)
			{
				double rot_x = search_pt[j].x, rot_y = search_pt[j].y;
				cv::Point2f rot_pt;
				rot_pt.x = cos_vals[i] * rot_x - sin_vals[i] * rot_y;
				rot_pt.y = sin_vals[i] * rot_x + cos_vals[i] * rot_y;

				rotated_search_pt[j] = imageCoordinate(rot_pt);
			}


			transform_info best_trans = searchTranslation(rotated_search_pt, range, step, initialization.translation_2d, rotations[i], est_cov);


			if(est_cov)
			{
				K += best_trans.K;
				u += best_trans.u;
				s += best_trans.s;
			}
			//best_rotates[i] = best_trans;
			//#pragma omp critical
			//cout<<rotations[i]<<" "<<best_trans.translation_2d<<": "<<best_trans.score<<endl;
			if(best_trans.score > best_rotate.score)
			{
				best_rotate = best_trans;
				best_rotate.rotation =rotations[i];
				best_rotate.pts = rotated_search_pt;
			}

		}
		sw.end(false);
		sw_end.start("end");
		//cout<<"Total time spent in searchTranslation = "<<sw.total_/1000.0<<endl;

		if(est_cov)
		{
			cv::Mat cov = K/s + (u * u.t())/(s*s);
			//cout<<cov<<endl;
			//cout<<"cov_x="<<sqrt(cov.at<float>(0,0))<<" cov_y="<<sqrt(cov.at<float>(1,1))<<" cov_t="<<sqrt(cov.at<float>(2,2))/M_PI*180<<endl;
			best_rotate.covariance = cov;
		}
		sw_end.end(false);
		return best_rotate;

	}

	transform_info searchTranslation(vector<cv::Point> &search_pt, int range, int stepsize, cv::Point2f initial_pt, double rotation, bool est_cov)
	{
		//investigate why low bottom of confusion matrix has overall higher score
		//solved: It is a bad idea to normalize the score with only the number of point within the source map

		fmutil::Stopwatch sw, sw1,sw2,sw3;
		sw.start("");
		//result deteriorate when the resolution is more than 0.3. This is due to quantization error when mapping
		//from pts to grid. Need independent control of map grid size and stepping size
		int startx = initial_pt.x/res_ - range, endx = initial_pt.x/res_ + range;
		int starty = initial_pt.y/res_ - range, endy = initial_pt.y/res_ + range;
		//parallel seems to only improve marginally, and because the whole vector of results need to
		//be stored into memory, in the end there is no improvement
		//adding omp critical directive eliminate the need to allocate a large memory
		//improved calculation from 0.7 to 0.45, further improvement to 0.4 when searchRotation also run in omp
		transform_info best_info;
		best_info.score = -1e99;

		//best_info.pts = search_pt;
		//cout<<startx<<" "<<endx<<" "<<starty<<" "<<endy<<endl;
		//vector<cv::Point> new_search_pt;
		//new_search_pt.resize(search_pt.size());
		cv::Mat x_i = cv::Mat::zeros(3,1, CV_32F);
		best_info.K = cv::Mat::zeros(3,3, CV_32F);
		best_info.u = cv::Mat::zeros(3,1, CV_32F);
		best_info.s = 0;
		x_i.at<float>(2,0)= rotation;

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
				double cur_score = scorePoints(search_pt, i, j);

				sw2.end(false);
				sw3.start("");
				//cout<<"offset "<<new_search_pt[0]<<" score "<<cur_score<<endl;
				//cout<<cur_score<<" ";
				if(cur_score>best_info.score)
				{
					best_info.translation_2d.x = i*res_;
					best_info.translation_2d.y = j*res_;
					best_info.score = cur_score;
					//best_info.pts = search_pt;
				}
				x_i.at<float>(0,0) = i*res_;
				x_i.at<float>(1,0) = j*res_;
				if(est_cov)
				{
					best_info.K += x_i * x_i.t() * cur_score;
					best_info.u += x_i * cur_score;
					best_info.s += cur_score;
				}
				sw3.end(false);

			}
			//cout<<endl;
		}
		sw.end(false);
		/*double t1 = sw1.total_/1000.0;
		double t2 = sw2.total_/1000.0;
		double t3 = sw3.total_/1000.0;
		cout<<"Detail: "<<sw.total_/1000.0<<" check:"<<t1<<"+"<<t2<<"+"<<t3<<"="<<t1+t2+t3<<endl;
		*/
		//cout<<endl;

		return best_info;
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
			mapping.push_back((int) (255*exp(-d*d/range_covariance_)));
			//cout<<d<<":"<<mapping[mapping.size()-1]<<endl;
		}
		//cout<<endl;
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
		}
	}


};

