#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include <math.h>
#include <fmutil/fm_stopwatch.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "ReadFileHelper.h"
#include "geometry_helper.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

using namespace std;

std::istream& GotoLine(std::istream& file, unsigned int num){
	file.seekg(std::ios::beg);
	for(unsigned int i=0; i <= num; ++i){
		file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
		//cout<<"Going to line "<<i<<"\xd"<<flush;
	}
	return file;
}
cv::Mat polarTransform(cv::Mat src)
{
	cv::Mat dst(src.size(), src.type()), src2(src.size(), src.type());

	IplImage c_src = src, c_dst = dst, c_src2 = src2;

	cvLogPolar( &c_src,  &c_dst, cv::Point2f(src.cols*0.5f, src.rows*0.5f),
			20, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );
	cvLogPolar( &c_dst, &c_src2, cv::Point2f(src.cols*0.5f, src.rows*0.5f),
			20, CV_INTER_LINEAR+CV_WARP_INVERSE_MAP );
	//imshow( "log-polar", dst );
	//imshow( "inverse log-polar", src2 );
	//cv::waitKey();
	return dst;
}
cv::Size rotated_rect_size(double theta, cv::Size img_size)
{
	double ct = cos( theta/180.0*M_PI );
	double st = sin( theta/180.0*M_PI );

	int h = img_size.height, w = img_size.width;
	double hct = h * ct;
	double wct = w * ct;
	double hst = h * st;
	double wst = w * st;
	double A_y = h/2., A_x = w/2.;
	double y_min, y_max, x_min, x_max;
	if ( theta > 0 )
	{
		if ( theta > 90 )
		{
			// 0 < theta < 90
			y_min = A_y;
			y_max = A_y + hct + wst;
			x_min = A_x - hst;
			x_max = A_x + wct;
		}
		else
		{
			// 90 <= theta <= 180
			y_min = A_y + hct;
			y_max = A_y + wst;
			x_min = A_x - hst + wct;
			x_max = A_x;
		}
	}
	else
	{
		if ( theta > -90 )
		{
			// -90 < theta <= 0
			y_min = A_y + wst;
			y_max = A_y + hct;
			x_min = A_x;
			x_max = A_x + wct - hst;
		}
		else
		{
			// -180 <= theta <= -90
			y_min = A_y + wst + hct;
			y_max = A_y;
			x_min = A_x + wct; // not hct
			x_max = A_x - hst;
		}
	}
	return cv::Size(sqrt(x_min*x_min + x_max*x_max), sqrt(y_min*y_min+y_max*y_max));
}

void matchingBruteForce(cv::Mat& expand_img_sat, cv::Mat& dst_img, cv::Mat &result)
{
	result = cv::Mat::ones(expand_img_sat.rows - dst_img.rows + 1, expand_img_sat.cols - dst_img.cols + 1, CV_32FC1);
	cv::Mat mask = cv::Mat::zeros(result.rows, result.cols, CV_8UC1);
//#pragma omp parallel for
	double sum = cv::sum(dst_img)[0];

	for(int y=0; y<result.rows; y++)
	{
		for(int x=0; x<result.cols; x++)
		{
			cv::Mat expand_img_roi = expand_img_sat(cv::Rect(x,y, dst_img.cols, dst_img.rows));

			cv::Mat raw_substract_img = (dst_img-expand_img_roi);
			/*cv::imshow("expand_img_roi", expand_img_roi);
							cv::imshow("dst_img", dst_img);
							cv::imshow("raw_substract_img", raw_substract_img);
							cv::waitKey();*/
			result.at<float>(y,x) = cv::sum(raw_substract_img)[0];
			mask.at<uchar>(y,x) = 1;


			/*cv::imshow("dst_img", dst_img);
				cv::imshow("exp_img_roi", expand_img_roi);
				cv::imshow("subtract img", raw_substract_img);

				cv::waitKey();*/
		}
	}
}

pcl::PointCloud<pcl::PointXYZ> toPCL(sensor_msgs::PointCloud &pc)
		{
	sensor_msgs::PointCloud2 pc2;
	pcl::PointCloud<pcl::PointXYZ> pcl;
	sensor_msgs::convertPointCloudToPointCloud2(pc, pc2);
	pcl::fromROSMsg(pc2, pcl);
	return pcl;
		}

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argcv)
{
	istream*        data_in         = NULL;         // input for data points
	ifstream dataStreamSrc;
	dataStreamSrc.open(argcv[3], ios::in);// open data file
	if (!dataStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	data_in = &dataStreamSrc;
	string src_filename = argcv[2];
	string dst_filename = argcv[1];
	int src_idx = atoi(src_filename.substr(src_filename.size()-9, 5).c_str());
	int dst_idx = atoi(dst_filename.substr(dst_filename.size()-9, 5).c_str());
	cout<<"Getting pointclouds for src: "<<src_idx<<" dst: "<<dst_idx<<endl;
	GotoLine(*data_in, 0);
	string str;getline(*data_in, str);
	//cout<<str<<endl;
	GotoLine(*data_in, 1);
	getline(*data_in, str);
	//cout<<str<<endl;
	//exit(0);

	ros::init(argc, argcv, "height_matching");
	cv::Mat src_img = cv::imread(argcv[1], CV_8UC1);
	cv::Mat dst_img = cv::imread(argcv[2], CV_8UC1);
	cv::Point best_loc;
	cv::Mat best_img, best_sub_img;
	int best_rot;
	double score=1e-99, percent_score, percent_score_ver;
	int total_nonzero = cv::countNonZero(dst_img);
	int total_nonzero_src = cv::countNonZero(src_img);

	cout<<"Total non-zero pixels"<<total_nonzero<<endl;
	fmutil::Stopwatch sw("Start height matching");

	//src_img = src_img - cv::Mat::ones(src_img.rows, src_img.cols, src_img.type());
	//dst_img = dst_img - cv::Mat::ones(dst_img.rows, dst_img.cols, dst_img.type());
	cv::Scalar worse_score = cv::sum(dst_img);
	cv::Mat best_result;
	cv::Scalar mean, stddev, mean_rev, stddev_rev;
	fmutil::Stopwatch sw_1("1st");
	fmutil::Stopwatch sw_2("2nd");

	for(int i=-180; i<180; i++)
	{
		//if(i>-160 && i<-20) continue;
		//if(i>20 && i<160) continue;
		cout<<"Comparing angle "<<i<<"    \xd"<<flush;
		sw_1.start();
		int newSize = ceil(sqrt(src_img.cols*src_img.cols + src_img.rows*src_img.rows));
		cv::Mat base_img = cv::Mat::zeros(cv::Size(newSize,newSize), src_img.type());
		cv::Mat roi = base_img(cv::Rect((newSize-src_img.cols)/2, (newSize-src_img.rows)/2, src_img.cols, src_img.rows));
		src_img.copyTo(roi);
		cv::Mat rotationMat = cv::getRotationMatrix2D(cv::Point2f(newSize/2, newSize/2), (double)i, 1.0);
		cv::warpAffine(base_img, base_img, rotationMat, base_img.size());

		cv::Mat expand_img = cv::Mat::zeros(cv::Size(newSize+dst_img.cols, newSize+dst_img.rows), src_img.type());
		roi = expand_img(cv::Rect(dst_img.cols/2, dst_img.rows/2, newSize, newSize));
		base_img.copyTo(roi);
		int result_cols =  ((expand_img.cols - dst_img.cols)) + 1;
		int result_rows = ((expand_img.rows - dst_img.rows)) + 1;
		cv::Mat result, overlap, mask;
		//result = cv::Mat::zeros( result_rows, result_cols, CV_32FC1 );
		mask = cv::Mat::zeros( result_rows, result_cols, CV_8UC1 );
		overlap = cv::Mat::zeros( result_rows, result_cols, CV_32FC1 );
		cv::Mat empty_canvas = cv::Mat::zeros(cv::Size(expand_img.cols, expand_img.rows), dst_img.type());

		cv::Mat expand_img_sat = expand_img * 2;
		int size_of_matching_x = 40 + dst_img.cols;
		int size_of_matching_y = 40 + dst_img.rows;
		cv::Mat expand_img_sat_roi = base_img(cv::Rect((newSize - size_of_matching_x)/2., (newSize - size_of_matching_y)/2., size_of_matching_x, size_of_matching_y))*2;
		//cv::imshow("expand_img_sat", expand_img_sat);
		//cv::imshow("expand_img_sat_roi", expand_img_sat_roi);
		//cv::waitKey();
		sw_1.end(false);
		sw_2.start();

		sw_2.end(false);
		//exact same solution as the brute force method, with 7x speed improvement, incredible!
		/*expand_img_sat = polarTransform(expand_img_sat);

		cv::Mat empty_cav = cv::Mat::zeros(expand_img_sat.rows, expand_img_sat.cols, expand_img_sat.type());
		cv::Mat empty_cav_roi = empty_cav(cv::Rect(dst_img.cols/2, dst_img.rows/2, dst_img.cols, dst_img.rows));
		dst_img.copyTo(empty_cav_roi);
		cv::Mat dst_img_pol = polarTransform(empty_cav);
		cv::imshow("expand_img_sat", expand_img_sat);
		cv::imshow("dst_img_pol", dst_img_pol);
		cv::waitKey();
		 */
		cv::matchTemplate(expand_img_sat_roi, dst_img, result, CV_TM_CCOEFF);
		//matchingBruteForce(expand_img_sat_roi, dst_img, result);
		double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
		cv::Point matchLoc;
		//cv::compare(result, 0., mask, cv::CMP_GT);
		cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc);//, mask );


		if(maxVal > score)
		{
			score = maxVal;
			best_rot = i;
			best_loc = maxLoc;
			best_img = expand_img_sat_roi;
			normalize( result, best_result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
			cv::Mat expand_img_roi = expand_img(cv::Rect(maxLoc.x,maxLoc.y, dst_img.cols, dst_img.rows));
			cv::Mat dst_img_sat = dst_img *2;
			cv::Mat raw_subtract_img_ver = expand_img_roi - dst_img_sat;
			cv::Mat expand_img_roi_sat = expand_img_roi *2;
			cv::Mat raw_subtract_img = dst_img - expand_img_roi_sat;
			percent_score = cv::sum(raw_subtract_img)[0]/worse_score[0];
			cv::meanStdDev(best_result, mean, stddev);
			cv::meanStdDev(raw_subtract_img_ver, mean_rev,stddev_rev, expand_img_roi);
			percent_score_ver = cv::sum(raw_subtract_img_ver)[0]/cv::sum(expand_img_roi)[0];

			best_sub_img = raw_subtract_img;

			/*if(minVal == 21503)
			{
				cout<<dst_img<<endl;
				cout<<expand_img_roi<<endl;
				cout<<raw_subtract_img<<endl;
			}*/
		}
	}
	cout<<sw_1.total_<<endl;
	cout<<sw_2.total_<<endl;
	sw.end();
	cout<<"Best found total:"<<score/worse_score[0]<<" percent_score:"<<percent_score<<" per_score_ver:"<<percent_score_ver<<" rot: "<<best_rot<<" "<<best_loc<<" "<<endl;
	cout<<"Mean: "<<mean[0]<<" stddev: "<<stddev[0]<<endl;
	cout<<"MeanR: "<<mean_rev[0]<<" stddevR: "<<stddev_rev[0]<<endl;

	//exit(0);
	cv::Mat match_img;
	match_img = cv::Mat::zeros(best_img.rows, best_img.cols, dst_img.type());
	cout<<best_img.cols<<" "<<best_img.rows<<endl;
	cout<<match_img.cols<<" "<<match_img.rows<<endl;
	cout<<best_loc.y + dst_img.cols<<" "<<best_loc.x+dst_img.rows<<endl;
	cv::Mat roi;
	roi = match_img(cv::Rect(best_loc.x, best_loc.y, dst_img.cols, dst_img.rows));
	dst_img.copyTo(roi);
	cout<<match_img.cols<<" "<<match_img.rows<<endl;

	double real_x = (best_loc.x < 20 )? - (20 -best_loc.x) : best_loc.x - 20;
	double real_y =  (best_loc.y < 20)? - (20 -best_loc.y): best_loc.y - 20;
	cout<<"real coordinate: "<< real_x <<" "<< real_y <<endl;

	geometry_msgs::Pose match_tf;
	match_tf.position.x = real_x;
	match_tf.position.y = real_y;
	match_tf.position.z = 0.;
	match_tf.orientation.z = best_rot/180.*M_PI;

	ros::NodeHandle nh;
	ros::Publisher src_pub, dst_pub, query_pub, icp_pub;
	src_pub = nh.advertise<sensor_msgs::PointCloud>("src_pc", 5);
	dst_pub = nh.advertise<sensor_msgs::PointCloud>("dst_pc", 5);
	query_pub = nh.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 5);
	icp_pub = nh.advertise<sensor_msgs::PointCloud2>("icp_out", 5);
	sensor_msgs::PointCloud src_pc, dst_pc, query_pc;
	src_pc.header.frame_id = dst_pc.header.frame_id = query_pc.header.frame_id = "scan_odo";
	vector<geometry_msgs::Pose> poses;
	vector<vector<sensor_msgs::PointCloud> > pc_vecs;

	//there are weird things need to be resolved, why src_idx -2? and why -match_tf.position.x?
	//it could well be the aliasing issue, but there will be at most a 1 meter discrepancy, not 4 meters

	geometry_msgs::Pose pose_temp;
	uint64_t time;
	GotoLine(*data_in, 0);

	if(!readPts3D(*data_in, src_pc, pose_temp,time)) cout<<"Failed to get src_idx"<<endl;

	//GotoLine(*data_in, 1);
	if(!readPts3D(*data_in, query_pc, pose_temp,time)) cout<<"Failed to get query_idx"<<endl;

	match_tf.position.x = -match_tf.position.x;
	dst_pc.points = getTransformedPts(match_tf, query_pc.points);


	pcl::PointCloud<pcl::PointXYZ> input_pc = toPCL(src_pc);
	pcl::PointCloud<pcl::PointXYZ> output_pc = toPCL(dst_pc);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(input_pc.makeShared());
	icp.setInputTarget(output_pc.makeShared());
	pcl::PointCloud<pcl::PointXYZ> final_pc;
	fmutil::Stopwatch sw_icp("icp");
	icp.align(final_pc);
	sw_icp.end();
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
			icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	for(int k=0; k<3; k++)
	{
		final_pc.header = src_pc.header;
		src_pub.publish(src_pc);
		dst_pub.publish(dst_pc);
		query_pub.publish(query_pc);
		sensor_msgs::PointCloud2 final_pc2;
		pcl::toROSMsg(final_pc, final_pc2);
		icp_pub.publish(final_pc2);
		ros::spinOnce();
	}
	for(int i=0; 1; i++)
	{
		if(i%2 == 0)
			cv::imshow("base_img", best_img);
		else
			cv::imshow("base_img", match_img);
		cv::imshow("res_mat", best_result);
		cv::waitKey();
	}
	return 0;
}
