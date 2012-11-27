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

	double scorePoints(vector<cv::Point> search_pt)
	{
		fmutil::Stopwatch sw;
		sw.start("Score Pts");
		assert(image_.data != NULL);

		double score = 0;


		//it only takes 25 ms for 6k loops on 0.03 res
		for(size_t i=0; i<search_pt.size(); i++)
		{
			cv::Point pt = search_pt[i];// cv::Point(0,0);// imageCoordinate(search_pt[i]);
			//penalized each points fall outside of map to zero

			if(outsideMap(image_, pt)) continue;
			score += getPixel(pt.x, pt.y);
		}

		score/=search_pt.size();
		sw.end(false);
		//cout<<"Score = "<<score/255*100<<"%"<<endl;
		return score/255*100;
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
		sw.end();
		//cv::imwrite("map.png", image_);
	}

	transform_info searchRotation(vector<cv::Point2f> search_pt, int translate_range, double rot_range, double rot_step, transform_info initialization)
	{
		//translate range is directly corresponds to number of cells in the grid
		//precalculate cosine
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
		fmutil::Stopwatch sw;
		for(size_t i=0; i<rotations.size(); i++)
		{

			//rotate each point with by using the rotation center at the sensor's origin
			sw.start("calculate");

			for(size_t j=0; j<search_pt.size(); j++)
			{
				double rot_x = search_pt[j].x, rot_y = search_pt[j].y;
				cv::Point2f rot_pt;
				rot_pt.x = cos_vals[i] * rot_x - sin_vals[i] * rot_y;
				rot_pt.y = sin_vals[i] * rot_x + cos_vals[i] * rot_y;
				rotated_search_pt[j] = imageCoordinate(rot_pt);
			}


			transform_info best_trans = searchTranslation(rotated_search_pt, translate_range, initialization.translation_2d);
			sw.end(false);

			//best_rotates[i] = best_trans;
			//#pragma omp critical
			//cout<<rotations[i]<<" "<<best_trans.translation_2d<<": "<<best_trans.score<<endl;
			if(best_trans.score > best_rotate.score)
			{
				best_rotate = best_trans;
				best_rotate.rotation =rotations[i];
			}

		}
		cout<<"Total time spent in searchTranslation = "<<sw.total_/1000.0<<endl;
		return best_rotate;

	}

	transform_info searchTranslation(vector<cv::Point> &search_pt, int range, cv::Point2f initial_pt)
	{
		//investigate why low bottom of confusion matrix has overall higher score
		//solved: It is a bad idea to normalize the score with only the number of point within the source map
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
		vector<cv::Point> new_search_pt;
		new_search_pt.resize(search_pt.size());
		for(int j=starty; j<=endy; j++)
		{
			for(int i=startx; i<=endx; i++)
			{
				//gained about 80 ms when fixed size of new_search_pt is used instead of keep pushing the search pt
				for(size_t k=0; k<search_pt.size(); k++)
					new_search_pt[k] = (cv::Point(search_pt[k].x+i, search_pt[k].y-j));
				double cur_score = scorePoints(new_search_pt);
				//cout<<"offset "<<new_search_pt[0]<<" score "<<cur_score<<endl;
				if(cur_score>best_info.score)
				{
					best_info.translation_2d.x = i*res_;
					best_info.translation_2d.y = j*res_;
					best_info.score = cur_score;
					best_info.pts = new_search_pt;
				}
			}
		}

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

bool readPt(istream &in, cv::Point2f &p)            // read point (false on EOF)
{

	if(!(in >> p.x)) return false;
	if(!(in >> p.y)) return false;

	return true;
}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char **argcv)
{
	istream*        data_in         = NULL;         // input for data points
	istream*        query_in         = NULL;         // input for query points
	vector<cv::Point2f> raster_pts, query_pts;
	cv::Point2f data_pts;
	ifstream dataStreamSrc, dataStreamDst;
	dataStreamSrc.open(argcv[1], ios::in);// open data file
	if (!dataStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	data_in = &dataStreamSrc;

	dataStreamDst.open(argcv[2], ios::in);// open data file
	if (!dataStreamDst) {
		cerr << "Cannot open query file\n";
		exit(1);
	}
	query_in = &dataStreamDst;

	while (readPt(*data_in, data_pts)) {
		raster_pts.push_back(data_pts);
	}
	cout << raster_pts.size() << "points read"<<endl;

	while (readPt(*query_in, data_pts)) {         // read query points
		query_pts.push_back(data_pts);
	}
	cout << query_pts.size() << "points read"<<endl;

	transform_info best_tf;
	fmutil::Stopwatch sw;
	sw.start("New matching apporach");
	RasterMapImage rm(0.3, 0.01);
	rm.getInputPoints(raster_pts);
	//searchRotation(vector<cv::Point2f> search_pt, double translate_range, double rot_range, double rot_step, transform_info initialization)
	transform_info best_info;
	best_info = rm.searchRotation(query_pts, 4.0/0.3, M_PI/4., M_PI/8., best_info);
	vector<cv::Point> rotated_search_pt;
	rotated_search_pt.resize(query_pts.size());
	//for(size_t i=0; i<query_pts.size(); i++) rotated_search_pt[i] = rm.imageCoordinate(query_pts[i]);
	//best_info = rm.searchTranslation(rotated_search_pt, 4.0/0.2, best_info.translation_2d);
	cout<<"Best translation "<<best_info.translation_2d<<" "<<best_info.rotation<<" with score "<<best_info.score<<endl;
	RasterMapImage rm2(0.03, 0.01);
	rm2.getInputPoints(raster_pts);
	//for(size_t i=0; i<query_pts.size(); i++) rotated_search_pt[i] = rm2.imageCoordinate(query_pts[i]);
	best_info.score = -1e99;
	//best_info = rm2.searchTranslation(rotated_search_pt, 0.2/0.02, best_info.translation_2d);
	//cout<<"Best translation "<<best_info.translation_2d<<" "<<best_info.rotation<<" with score "<<best_info.score<<endl;
	best_info = rm2.searchRotation(query_pts, 0.15/0.03, M_PI/16., M_PI/180., best_info);
	cout<<"Best translation "<<best_info.translation_2d<<" "<<best_info.rotation<<" with score "<<best_info.score<<endl;
	sw.end();


	ros::init(argc, argcv, "RasterMapImage");
	ros::NodeHandle nh;
	ros::Publisher src_pub, dst_pub, query_pub;
	src_pub = nh.advertise<sensor_msgs::PointCloud>("src_pts", 5);
	dst_pub = nh.advertise<sensor_msgs::PointCloud>("dst_pts", 5);
	query_pub = nh.advertise<sensor_msgs::PointCloud>("query_pts", 5);
	ros::Rate rate(10);

	sensor_msgs::PointCloud src_pc, dst_pc, query_pc;
	src_pc.header.frame_id = dst_pc.header.frame_id = query_pc.header.frame_id = "scan";
	for(size_t i=0; i<raster_pts.size();i++)
	{
		geometry_msgs::Point32 pt;
		pt.x = raster_pts[i].x;
		pt.y = raster_pts[i].y;
		src_pc.points.push_back(pt);
	}
	for(size_t i=0; i<best_info.pts.size();i++)
	{
		geometry_msgs::Point32 pt;
		cv::Point2f cv_pt2f = rm2.realCoordinate(best_info.pts[i]);
		pt.x = cv_pt2f.x;
		pt.y = cv_pt2f.y;
		dst_pc.points.push_back(pt);
	}
	for(size_t i=0; i<query_pts.size();i++)
	{
		geometry_msgs::Point32 pt;
		pt.x = query_pts[i].x;
		pt.y = query_pts[i].y;
		query_pc.points.push_back(pt);
	}
	while(ros::ok())
	{
		src_pc.header.stamp = dst_pc.header.stamp = query_pc.header.stamp = ros::Time::now();
		src_pub.publish(src_pc);
		dst_pub.publish(dst_pc);
		query_pub.publish(query_pc);
		rate.sleep();
	}
	return 0;
}
