#include <fmutil/fm_stopwatch.h>
#include <cv.h>
#include <highgui.h>
#include <fmutil/fm_math.h>


#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

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

	template <class T>
	void getInputPoints(vector<T> raster_pt_input)
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
		vector<cv::Point2f> raster_pt = pcl_downsample(search_pt, res_/2., res_/2., res_/2.);

		for(size_t i=0; i<raster_pt.size(); i++)
		{
			if(raster_pt[i].x < min_pt_.x) min_pt_.x = raster_pt[i].x;
			if(raster_pt[i].x > max_pt_.x) max_pt_.x = raster_pt[i].x;

			if(raster_pt[i].y < min_pt_.y) min_pt_.y = raster_pt[i].y;
			if(raster_pt[i].y > max_pt_.y) max_pt_.y = raster_pt[i].y;
		}

		cv::Point2f map_size(max_pt_.x - min_pt_.x, max_pt_.y - min_pt_.y);
		map_size.x = ceil(map_size.x/res_); map_size.y =ceil(map_size.y/res_);

		//lost about 10ms when using 32F instead of 8U, and total of 45 ms if draw circle function is called, perhaps too much
		image_ = cv::Mat::ones( (int) map_size.y, (int) map_size.x  , CV_8UC1);

		fmutil::Stopwatch sw_draw("Confirming rastermap time");
		for(int j=gaussian_mapping_.size()-1; j>=0; j--)
		{
			//openmp helps reduce the rastering time from 150+ to 60+ms
			//#pragma omp parallel for
			int color = gaussian_mapping_[j];
			for(size_t i=0; i<raster_pt.size(); i++)
			{
				cv::Point pt =imageCoordinate(raster_pt[i]);
				if(pt.x >= image_.cols || pt.y >= image_.rows) continue;
				cv::circle(image_, pt, j, cv::Scalar(color,color,color), -1);
			}
		}
		sw_draw.end();

		sw.end(true);
		stringstream ss;
		ss<<"rastered_map_"<<res_<<"_"<< range_covariance_<<".png";
		cv::imwrite(ss.str(), image_);
	}

private:
	vector<int> gaussian_mapping_;
	double res_, range_covariance_, max_dist_, gausian_length_;
	cv::Point2f min_pt_, max_pt_;

	vector<int> makeGaussianLinearMapping()
	{
		vector<int> mapping;
		for(int i=0; i<gausian_length_; i++)
		{
			double d = res_ * i;
			//0-254 and offset to 1-255;
			int gaussian_value = (int) (254*exp(-d*d/range_covariance_));
			mapping.push_back(gaussian_value+1);
		}
		return mapping;
	}
}
