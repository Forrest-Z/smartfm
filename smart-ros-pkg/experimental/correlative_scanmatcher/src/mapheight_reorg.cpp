#include <ros/ros.h>
#include <fmutil/fm_math.h>
#include <fmutil/fm_stopwatch.h>
#include <cv.h>
#include <highgui.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
using namespace std;
#include "renderMap.h"
#include "ReadFileHelper.h"
#include "geometry_helper.h"
#include "pcl_downsample.h"
#include "pso_definition.h"


int main(int argc, char** argcv)
{
	ros::init(argc, argcv, "mapheight_reorg");
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
	sensor_msgs::PointCloud src, dst;
	geometry_msgs::Pose pose;


	readPts3D(*data_in, src, pose, time);
	readPts3D(*data_in, dst, pose, time);

	cout<<src.points.size()<<endl;
	cout<<dst.points.size()<<endl;

	fmutil::Stopwatch sw("start overall");


	RNG_GENERATOR::rng_srand();
	RNG_GENERATOR::rng_warm_up();

	// To keep track of the best particle ever found
	PSO::BestType best_particle;
	Problem::init(src, dst);

	// Let's create our swarm
	PSO pso;

	// We now run our algorithm
	fmutil::Stopwatch sw2("pso");
	pso.run(1);
	sw2.end();
	sw.end();
	// Some display
	std::cout << "Best particle : " << pso.getBest().getPosition(0) << " "<< pso.getBest().getPosition(1)<<" "<<pso.getBest().getPosition(2)<< std::endl;
	cv::Mat best_img = Problem::rotated_img_[round((pso.getBest().getPosition(2)+180)/Problem::rot_res_)];
	cv::Mat match_img;
	match_img = cv::Mat::zeros(best_img.rows, best_img.cols, Problem::dst_.type());

	cv::Mat roi;
	roi = match_img(cv::Rect(round(pso.getBest().getPosition(0)/Problem::trans_res_), round(pso.getBest().getPosition(1)/Problem::trans_res_), Problem::dst_.cols, Problem::dst_.rows));
	Problem::dst_.copyTo(roi);

	Problem::free();


	ros::NodeHandle nh;
	ros::Publisher src_pub, dst_pub, query_pub, icp_pub;
	src_pub = nh.advertise<sensor_msgs::PointCloud>("src_pc", 5);
	dst_pub = nh.advertise<sensor_msgs::PointCloud>("dst_pc", 5);
	query_pub = nh.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 5);
	icp_pub = nh.advertise<sensor_msgs::PointCloud2>("icp_out", 5);
	sensor_msgs::PointCloud src_pc, dst_pc, query_pc;
	src_pc = src;
	query_pc = dst;
	src_pc.header.frame_id = dst_pc.header.frame_id = query_pc.header.frame_id = "scan_odo";
	vector<geometry_msgs::Pose> poses;
	vector<vector<sensor_msgs::PointCloud> > pc_vecs;
	geometry_msgs::Pose match_tf;
	match_tf.position.x = -pso.getBest().getPosition(0)+20.;
	match_tf.position.y = pso.getBest().getPosition(1)-20.;
	match_tf.orientation.z = pso.getBest().getPosition(2)/180.*M_PI;
	dst_pc.points = getTransformedPts(match_tf, src_pc.points);

	ros::Duration timer(1.0);
	while(ros::ok())
	{
		src_pc.header.stamp = dst_pc.header.stamp = query_pc.header.stamp = ros::Time::now();
		src_pub.publish(src_pc);
		dst_pub.publish(dst_pc);
		query_pub.publish(query_pc);
		ros::spinOnce();
		timer.sleep();
	}

	for(int i=0; 1; i++)
	{
		if(i%2 == 0)
			cv::imshow("base_img", best_img);
		else
			cv::imshow("base_img", match_img);
		cv::waitKey();
	}
	return 0;
}
