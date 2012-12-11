/*
 * renderMap.h
 *
 *  Created on: Dec 6, 2012
 *      Author: demian
 */




cv::Point2f min_pt_(1e99,1e99), max_pt_(-1e99,-1e99);
cv::Mat image_;

template <class T>
inline cv::Point imageCoordinate(T pt, double res_)
{
	return cv::Point((pt.x-min_pt_.x)/res_, (int) image_.rows -(pt.y-min_pt_.y)/res_);
}

void drawMap(vector<geometry_msgs::Point32> raster_pt, double res_, string filename)
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

	for(size_t i=0; i<raster_pt.size(); i++)
	{
		cv::Point pt =imageCoordinate(raster_pt[i], res_);
		if(pt.x >= image_.cols || pt.y >= image_.rows) continue;

		image_.data[pt.y*image_.cols + pt.x -1] = 255;

	}

	sw.end(false);
	cv::imwrite(filename, image_);
}
