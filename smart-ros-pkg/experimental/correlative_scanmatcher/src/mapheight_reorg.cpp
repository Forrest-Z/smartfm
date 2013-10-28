#include <ros/ros.h>
#include <fmutil/fm_math.h>
#include <fmutil/fm_stopwatch.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Pose.h>
using namespace std;

#include "ReadFileHelper.h"
#include "geometry_helper.h"
#include "pcl_downsample.h"
#include "RasterMapImageOpt.h"
#include "pso_definition.h"

void readPtsFromFile(char** argcv)
{
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
}

class MyPSO
{
public:
	void runPSO(vector<cv::Mat> &input_images, cv::Mat &dst_img, cv::Mat &best_img, cv::Mat &match_img)
	{
		RNG_GENERATOR::rng_srand();
		RNG_GENERATOR::rng_warm_up();
		Problem::init(input_images, dst_img);

		// Let's create our swarm
		PSO pso;

		// We now run our algorithm
		fmutil::Stopwatch sw2("pso");
		pso.run(1);
		sw2.end();

		// Some display
		std::cout << "Best particle : " << pso.getBest().getPosition(0) << " "<< pso.getBest().getPosition(1)<<" "<<pso.getBest().getPosition(2)<< std::endl;
		best_img = Problem::rotated_img_[round((pso.getBest().getPosition(2)+180)/Problem::rot_res_)];

		match_img = cv::Mat::zeros(best_img.rows, best_img.cols, Problem::dst_.type());

		cv::Mat roi;
		roi = match_img(cv::Rect(round(pso.getBest().getPosition(0)/Problem::trans_res_), round(pso.getBest().getPosition(1)/Problem::trans_res_), Problem::dst_.cols, Problem::dst_.rows));
		Problem::dst_.copyTo(roi);

		Problem::free();

	};
};
int main(int argc, char** argcv)
{
	ros::init(argc, argcv, "mapheight_reorg");

	bool repeat = true;
	vector<cv::Mat> input_images;
	while(repeat)
	{
		int source_pts_idx, dest_pts_idx;
		bool source_repeat = true, dst_repeat = true;

		while(source_repeat)
		{
			cout<<"Please select a matching source: (8-3124): use y to confirm "<<flush;

			string matching_pts;
			getline(cin, matching_pts);
			if(matching_pts.size()>0)
			{
				if(matching_pts[0] == 'y')
				{
					source_repeat = false;
					continue;
				}
				if(matching_pts[0] == 'x')
					exit(0);
				source_pts_idx = atoi(matching_pts.c_str());
			}
			else source_pts_idx++;
			cout<<"Matching source "<<source_pts_idx<<" selected"<<endl;
			vector<cv::Mat> images;
			fmutil::Stopwatch sw("read_rotated_img");
			//rotating is faster than reading 720 images, it took more than 10 sec to read the files, astonishing.
			images.resize(720);
#pragma omp parallel for
			for(int i=0; i<720; i++)
			{
				stringstream ss;
				ss<<"src_"<<setfill('0')<<setw(5)<<source_pts_idx<<"_"<<setfill('0')<<setw(4)<<i<<".png";
				images[i] = (cv::imread(ss.str(), CV_8UC1))*2;
			}
			sw.end();
			cv::imshow("src_img", images[360]);
			cv::waitKey();
			input_images = images;
		}

		while(dst_repeat)
		{
			cout<<"Please select a matching destination (8-3124): use y to confirm "<<flush;

			string matching_pts;
			getline(cin, matching_pts);
			if(matching_pts.size()>0)
			{
				if(matching_pts[0] == 'x')
				{
					dst_repeat = false;
					continue;
				}
				dest_pts_idx = atoi(matching_pts.c_str());
			}
			else dest_pts_idx++;
			cout<<"Destination source "<<dest_pts_idx<<" selected"<<endl;
			stringstream ss;
			ss<<"dst_"<<setfill('0')<<setw(5)<<dest_pts_idx<<".png";
			cv::Mat dst_img = cv::imread(ss.str(), CV_8UC1);
			cv::imshow("dst_img", dst_img);
			cv::waitKey();
			fmutil::Stopwatch sw("matching...");
			cv::Mat best_img, match_img;
			MyPSO pso;
			pso.runPSO(input_images, dst_img, best_img, match_img);
			sw.end();
			bool show_result = true;
			int count = 0;
			while(show_result)
			{
				if(count%2==0)
					cv::imshow("Result", best_img);
				else
					cv::imshow("Result", match_img);
				count++;
				int keyPressed = cv::waitKey();
				//cout<<keyPressed<<endl;
				if( keyPressed== 1048603) show_result = false;
			}
			//cout<<"Match found at "<<source_pts_idx<<" "<<dest_pts_idx<<" with score "<<best_tf.score<<endl;
			//cout<<best_tf.translation_2d.x<<" "<<best_tf.translation_2d.y<<" "<<best_tf.rotation<<" ";
			//cout<<cov.at<float>(0,0)<<" "<<cov.at<float>(0,1)<<" "<<cov.at<float>(0,2)<<" "<<cov.at<float>(1, 1)<<" "<<cov.at<float>(1,2)<<" "<<cov.at<float>(2,2);
		}
	}

	return 0;
}
