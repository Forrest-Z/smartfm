#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <iostream>
#include <fstream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <fmutil/fm_math.h>
#include <fmutil/fm_stopwatch.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
//#include "renderMap.h"
#include "ReadFileHelper.h"
#include "geometry_helper.h"
#include "pcl_downsample.h"
#include "RasterMapImageOpt.h"
#include "heightmatching_pso.h"
#include <pcl/io/pcd_io.h>
void writeToFile(ofstream& file, vector<geometry_msgs::Point32> pts, string header)
{
	file<<header;
	file << pts.size()<<" ";
	for(size_t i=0; i<pts.size(); i++)
	{
		file<<pts[i].x<<" "<<pts[i].y<<" "<<pts[i].z<<" ";
	}
	file<<endl;
}

/*cv::Point imageCoordinate(cv::Point2f pt, cv::Point2f min_pt_, int image_rows, double res_)
{
	return cv::Point(round((pt.x-min_pt_.x)/res_), round(image_rows -(pt.y-min_pt_.y)/res_));
}

void getInputPoints(cv::Mat &image, vector<cv::Point2f> &raster_pt_input, vector<cv::Point3f> height_pc, cv::Point2f &min_pt_, cv::Point2f &max_pt_, double res_)
{
	cv::Mat image_;
	//fmutil::Stopwatch sw;

	//sw.start("Raster Map");

	//very fast process, only needs 5 ms on 1620 points with 1000 loops
	vector<cv::Point2f> search_pt;search_pt.resize(raster_pt_input.size());
	for(size_t i=0; i<raster_pt_input.size(); i++)
	{
		search_pt[i].x = raster_pt_input[i].x;
		search_pt[i].y = raster_pt_input[i].y;
	}

	//fmutil::Stopwatch sw2("getInputPoints downsample");
	//dbg<<"before downsample: "<<search_pt.size()<<endl;
	vector<cv::Point2f> raster_pt = search_pt;// pcl_downsample(search_pt, res_/2., res_/2., res_/2.);
	//dbg<<"after downsample: "<<raster_pt.size()<<endl;
	//sw2.end(false);

	for(size_t i=0; i<raster_pt.size(); i++)
	{
		if(raster_pt[i].x < min_pt_.x) min_pt_.x = raster_pt[i].x;
		if(raster_pt[i].x > max_pt_.x) max_pt_.x = raster_pt[i].x;

		if(raster_pt[i].y < min_pt_.y) min_pt_.y = raster_pt[i].y;
		if(raster_pt[i].y > max_pt_.y) max_pt_.y = raster_pt[i].y;
	}

	cv::Point2f map_size(max_pt_.x - min_pt_.x, max_pt_.y - min_pt_.y);
	map_size.x = ceil(map_size.x/res_); map_size.y =ceil(map_size.y/res_);

	//morph may cause a single pixel in some very special case, hence minimum value of the raster size should be 1x1 pixel
	if(map_size.x<1) map_size.x = 1;
	if(map_size.y<1) map_size.y = 1;
	//lost about 10ms when using 32F instead of 8U, and total of 45 ms if draw circle function is called, perhaps too much
	image_ = cv::Mat::zeros( (int) map_size.y, (int) map_size.x  , CV_8UC1);

	//fmutil::Stopwatch sw_draw("Confirming rastermap time");
	for(size_t i=0; i<height_pc.size(); i++)
	{
		cv::Point pt;
		pt.x = height_pc[i].x;
		pt.y = height_pc[i].y;
		pt =imageCoordinate(pt, min_pt_, image_.rows,res_);
		if(pt.x >= image_.cols || pt.y >= image_.rows) continue;
		int value = image_.data[pt.y*image_.cols +  pt.x];
		double height = height_pc[i].z;
		fmutil::bound(0.0, height , 2.0);
		uchar value_height = round(height*50);

		if(value<value_height) //image_.data[pt.y*image_.cols +  pt.x] = value_height;
			cv::circle(image_, pt, 0, cv::Scalar(value_height, value_height, value_height), 1);
	}

	stringstream ss;
	//ss<<"rastered_map_"<<res_<<"_"<< range_covariance_<<".png";

	vector<int> gaussian_mapping_;
	gaussian_mapping_.push_back(255);
	gaussian_mapping_.push_back(226);
	for(int j=1; j>=0; j--)
	{
		//openmp helps reduce the rastering time from 150+ to 60+ms
		//#pragma omp parallel for
		for(size_t i=0; i<raster_pt.size(); i++)
		{
			cv::Point pt =imageCoordinate(raster_pt[i], min_pt_, image_.rows,res_);
			if(pt.x >= image_.cols || pt.y >= image_.rows) continue;
			if(pt.x < 0 || pt.y < 0) continue;
			cv::circle(image_, pt, j, cv::Scalar(gaussian_mapping_[j],gaussian_mapping_[j],gaussian_mapping_[j]), -1);
			//cv::circle(image_, pt, 0, cv::Scalar(255,255,255),1);
			//image_.at<uchar>(pt.y, pt.x) = 255;
		}

	}
	//sw_draw.end(false);

	//cv::imwrite(ss.str(), image_);
	//sw.end(false);

	//return image_;
}*/
int main(int argc, char** argcv)
{
	 ros::init(argc, argcv, "mapHeightGenerate");
	 ros::NodeHandle nh;
	istream*        data_in         = NULL;         // input for data points
	ifstream dataStreamSrc;
	cout<<"Opening "<<argcv[1]<<endl;
	dataStreamSrc.open(argcv[1], ios::in);// open data file
	if (!dataStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	data_in = &dataStreamSrc;
	uint64_t time;
	vector<geometry_msgs::Point32> pc;
	geometry_msgs::Pose pose;
	int skip_read = 2;
	size_t accumulate_read =5;
	int read_line =0;

	ofstream raw_downsample_src, src_norm, dst_norm;
	stringstream timestamp;
	timestamp<<ros::WallTime::now().toNSec();
	string raw_file = timestamp.str()+"raw_src.txt";
	string src_norm_file = timestamp.str()+"src_norm.txt";
	string dst_norm_file = timestamp.str()+"dst_norm.txt";
	raw_downsample_src.open(raw_file.c_str());
	src_norm.open(src_norm_file.c_str());
	dst_norm.open(dst_norm_file.c_str());

	vector<geometry_msgs::Pose> poses;
	list<vector<geometry_msgs::Point32> > pc_vecs;
	//pc_vecs.reserve(10);
	while(readPts3D(*data_in, &pc, pose, time) && ros::ok())
	{
		stringstream pcd_file;
		pcd_file<<"raw_pcd_nus/raw_nus_"<<setfill('0') << setw(5)<<read_line<<".pcd";
		sensor_msgs::PointCloud2 pcd_pc2;
		sensor_msgs::PointCloud pcd_pc; pcd_pc.points = pc;
		sensor_msgs::convertPointCloudToPointCloud2(pcd_pc, pcd_pc2);
		pcl::io::savePCDFile(pcd_file.str(), pcd_pc2);
		if(false)//read_line%skip_read == 0)
		{
			poses.push_back(pose);
			pc_vecs.push_back(pc);
			//cout<<pc.points.size()<<endl;

			if(poses.size()>accumulate_read)
			{
				poses.erase(poses.begin());

				pc_vecs.reverse();
				pc_vecs.resize(accumulate_read);
				pc_vecs.reverse();
			}
			stringstream header;

			/*vector<geometry_msgs::Point32> dst_norm_pc = processNormals(pc);
			header<<pose.position.x<<" "<<pose.position.y<<" "<<pose.position.z<<" "<<pose.orientation.x<<" "<<pose.orientation.y<<" "<<pose.orientation.z<<" "<<time<<" ";
			writeToFile(dst_norm, dst_norm_pc, header.str());*/
			/*if(read_line == 584)
			{
				cout<<pc.size()<<" ";
				for(size_t i=0; i<pc.size(); i++)
				{
					cout<<pc[i].x<<" "<<pc[i].y<<" "<<pc[i].z<<" ";
				}
				cout<<endl;
				exit(0);
			}*/
			if(pc_vecs.size() == accumulate_read)
			{
				//cout<<pose<<endl;
				//cout<<pc_vecs[0]<<endl;
				//RenderMap rm;
				vector<geometry_msgs::Point32> acc_pc;

				list<vector<geometry_msgs::Point32> >::iterator it = pc_vecs.begin();
				for(size_t i=0; i<accumulate_read; i++)
				{

					geometry_msgs::Pose rel_pose = ominus( poses[i], poses[accumulate_read-1]);
					vector<geometry_msgs::Point32> input_pt;
					input_pt.resize((*it).size());
					for(size_t j=0; j<input_pt.size(); j++)
					{
						input_pt[j].x = (*it)[j].x;
						input_pt[j].y = (*it)[j].y;
						input_pt[j].z = (*it)[j].z;
					}

					vector<geometry_msgs::Point32> pc_transformed  = getTransformedPts(rel_pose, input_pt);
					acc_pc.insert(acc_pc.begin(), pc_transformed.begin(), pc_transformed.end());
					it++;
				}
				vector<geometry_msgs::Point32> src_norm_pc, acc_pc_downsample;
				acc_pc_downsample = pcl_downsample(acc_pc, 0.1, 0.1, 0.1);
				//acc_pc = pcl_downsample(acc_pc, 0.03, 0.03, 0.01);
				/*raw_downsample_src<<header.str();




				processNormals(acc_pc, src_norm_pc);
				//writeToFile(src_norm, src_norm_pc.points, header.str());
				cout<<"Acc src pts bef downsample: "<<acc_pc.size()<<endl;
				acc_pc_downsample = pcl_downsample(acc_pc, 0.1, 0.1, 0.1);
				cout<<"Acc src pts after downsample: "<<acc_pc_downsample.size()<<endl;
				//writeToFile(raw_downsample_src, acc_pc_temp.points, header.str());

				double trans_res_ = 0.1;
				RasterMapImage rmi(trans_res_, 0.03);
				//at least i know which line of code cause the atomic_exchange_and_add, shared_count segfault, all sorts of problem that I didn't observe before
				//but why???


				rmi.getInputPoints(src_norm_pc, acc_pc_downsample);
				//rm.drawHeightMap(acc_pc, 0.2, 140., 140.);
				stringstream ss;
				ss<<"nus_"<<setfill('0') << setw(5)<<read_line<<".png";
				cv::imwrite(ss.str(), rmi.image_);
				src_norm<<ss.str()<<" "<<rmi.min_pt_.x<<" "<<rmi.min_pt_.y<<" "<<rmi.max_pt_.x<<" "<<rmi.max_pt_.y<<endl;
				//cv::imshow("HeightMap", rm.image_);
				//cv::waitKey();
				//cout<<ss.str()<<"\xd"<<flush;*/
				/*if(read_line == 470)
				{
					cout<<acc_pc.size()<<" ";
					for(size_t i=0; i<acc_pc.size(); i++)
					{
						cout<<acc_pc[i].x<<" "<<acc_pc[i].y<<" "<<acc_pc[i].z<<" ";
					}
					cout<<endl;

				}*/
				HeightMatchingProblem<3> hmp;
				hmp.init(acc_pc_downsample, pc, true, read_line);

			}

		}
		//pc.resize(0);
		read_line++;
	}
}
