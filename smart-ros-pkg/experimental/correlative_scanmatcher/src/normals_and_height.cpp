#include <ros/ros.h>

#include <cv.h>
#include <highgui.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Pose.h>
#include <fmutil/fm_stopwatch.h>
#include <fmutil/fm_math.h>
using namespace std;

#include "renderMap.h"
#include "ReadFileHelper.h"
#include "geometry_helper.h"
#include "pcl_downsample.h"
#include "normals_and_height.h"

bool readDstFile(istream &data, cv::Mat &img, cv::Point2f &min_pt, cv::Point2f &max_pt)
{
	string image_file;
	if(!(data >> image_file )) return false;
	if(!(data >> min_pt.x)) return false;
	if(!(data >> min_pt.y)) return false;
	if(!(data >> max_pt.x)) return false;
	if(!(data >> max_pt.y)) return false;
	img = cv::imread(image_file, CV_8UC1);

	return true;
}

int main(int argc, char** argcv)
{
	ros::init(argc, argcv, "mapheight_reorg");
	ros::NodeHandle nh;
	ros::Publisher src_pub, dst_pub, query_pub, icp_pub;
	src_pub = nh.advertise<sensor_msgs::PointCloud>("src_pc", 5);
	dst_pub = nh.advertise<sensor_msgs::PointCloud>("dst_pc", 5);
	query_pub = nh.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 5);
	icp_pub = nh.advertise<sensor_msgs::PointCloud2>("icp_out", 5);
	sensor_msgs::PointCloud src_pc, dst_pc, query_pc;
	sensor_msgs::PointCloud2 icp_pc;
	icp_pc.header.frame_id = src_pc.header.frame_id = dst_pc.header.frame_id = query_pc.header.frame_id = "scan_odo";

	istream* data_in = NULL, *dst_file = NULL;         // input for data points
	ifstream dataStreamSrc, dataStreamDst;
	cout<<"Opening "<<argcv[1]<<endl;
	dataStreamSrc.open(argcv[1], ios::in);// open data file
	if (!dataStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	dataStreamDst.open(argcv[2], ios::in);
	if (!dataStreamDst) {
		cerr << "Cannot open dst file\n";
		exit(1);
	}
	data_in = &dataStreamSrc;
	dst_file = &dataStreamDst;

	bool select_src = true, select_dst = true;
	cv::Mat src_raster_img;
	cv::Point2f min_pt, max_pt;
	int skip = 0;
	while(select_src)
	{
		readDstFile(*data_in, src_raster_img, min_pt, max_pt);
		if(skip-- > 0) continue;
		else skip = 0;
		//cv::imshow("Select src...", src_raster_img);
		int keypressed = cv::waitKey(1);

		//if(keypressed==120) exit(0);
		//if(keypressed!=110) select_src = false;
		RasterMapImage rmi(0.1, 0.03);
		rmi.image_ = src_raster_img;
		rmi.min_pt_ = min_pt;
		rmi.max_pt_ = max_pt;
		src_pc.points = rmi.mapToRealPts();;
		src_pc.header.stamp = ros::Time::now();
		src_pub.publish(src_pc);
		ros::spinOnce();
		string matching_pts;
		getline(cin, matching_pts);
		if(matching_pts.size()>0)
		{
			if(matching_pts[0] == 'x') exit(0);
			if(matching_pts[0] == 'n') select_src = false;
			else skip = atoi(matching_pts.c_str());
		}
	}
	skip = 0;
	uint64_t time;
	vector<geometry_msgs::Point32> src, dst;
	geometry_msgs::Pose pose;
	while(select_dst)
	{
		readPts3D(*dst_file, &dst, pose, time);
		if(skip-- > 0) continue;
		else skip = 0;
		dst_pc.points = dst;
		dst_pc.header.stamp = ros::Time::now();
		dst_pub.publish(dst_pc);
		ros::spinOnce();
		string matching_pts;
		getline(cin, matching_pts);
		if(matching_pts.size()>0)
		{
			if(matching_pts[0] == 'x') exit(0);
			if(matching_pts[0] == 'n') select_dst = false;
			else skip = atoi(matching_pts.c_str());
		}
	}


	//next, start restructure raw data into 3 different pieces, src, src_norm, dst_norm
	//readPts3D(*data_in, &src, pose, time);

	//src= pcl_downsample(src, 0.1, 0.1, 0.1);
	//dst= pcl_downsample(dst, 0.1, 0.1, 0.1);
	cout<<src.size()<<endl;
	cout<<dst.size()<<endl;

	vector<geometry_msgs::Point32> src_norm, dst_norm;
	//src_norm = processNormals(src);
	//dst_norm = processNormals(dst);
	dst_norm = dst;
	fmutil::Stopwatch sw("start overall");
	RasterMapImage rm(0.1, 0.03);
	//rm.getInputPoints(src_norm, src);
	RNG_GENERATOR::rng_srand();
	RNG_GENERATOR::rng_warm_up();
	Problem::init(src_raster_img, min_pt, max_pt, dst_norm);


	fmutil::Stopwatch sw2("pso");
	ros::Duration ros_sleep(0.1);


	PSO pso;
	pso.run(1);
	sw2.end();
	// Some display
	std::cout << "Best particle : " << pso.getBest().getPosition(0) << " "<< pso.getBest().getPosition(1)<<" "<<pso.getBest().getPosition(2)<< std::endl;
	//exit(0);
	vector<cv::Point2f>  query_pts;
	query_pts.resize(dst_norm.size());

	for(size_t i=0; i<dst_norm.size(); i++)
	{
		query_pts[i].x =  dst_norm[i].x;
		query_pts[i].y =  dst_norm[i].y;
	}
	transform_info best_info;
	//very important initialization
	best_info.rotation = 0.0;
	vector<transform_info> best_tf;
	//fmutil::Stopwatch rm_sw("rm");
	//best_tf = rm.searchRotations(query_pts, 20.0, 5.0, 0.5, M_PI, M_PI/360., best_info, true, false);
	//single matching took about 0.003-0.005 ms
	//with single thread, it would take 500 ms for 10,000 match  at 0.1 res. Just good enough for an arbitrary match on XY axis.
	//rm_sw.end();
	//sort(best_tf.begin(), best_tf.end(), sortScore);
	//cout<<best_tf[0].translation_2d<<" "<<best_tf[0].rotation<<" "<<best_tf[0].score<<endl;
	//alright, now that all the information is available, change the raster map 0-255 values representation
	//0-100 -> Height penalization occur as height goes higher, if a normals point fall within this region, penalize it, if not don't
	//155-255 -> Gaussian Points
	//101-154 -> reserved


	src_pc.points = src_norm;
	query_pc.points = dst_norm;

	geometry_msgs::Pose match_tf;
	match_tf.position.x = pso.getBest().getPosition(0);//best_tf[0].translation_2d.x;
	match_tf.position.y = pso.getBest().getPosition(1);//best_tf[0].translation_2d.y;
	match_tf.orientation.z = pso.getBest().getPosition(2);//best_tf[0].rotation;
	dst_pc.points = getTransformedPts(match_tf, query_pc.points);


	vector<geometry_msgs::Pose> poses;
	vector<vector<sensor_msgs::PointCloud> > pc_vecs;


	ros::Duration timer(1.0);
	while(ros::ok())
	{
		icp_pc.header.stamp = src_pc.header.stamp = dst_pc.header.stamp = query_pc.header.stamp = ros::Time::now();
		icp_pc.header = src_pc.header;
		src_pub.publish(src_pc);
		dst_pub.publish(dst_pc);
		query_pub.publish(query_pc);
		icp_pub.publish(icp_pc);
		ros::spinOnce();
		timer.sleep();
	}


	return 0;
}
