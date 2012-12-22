/*
 * RasterMapImageOptSample.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "pcl/ros/conversions.h"
#include "RasterMapPCL.h"
#include "ReadFileHelper.h"
#include <isam/isam.h>
#include "GraphPF.h"

vector<geometry_msgs::Point32> getTransformedPts(geometry_msgs::Point32 pose, vector<geometry_msgs::Point32>& pts)
{
	double ct = cos(pose.z), st = sin(pose.z);
	vector<geometry_msgs::Point32> final_pt;
	final_pt.resize(pts.size());
	for(size_t j=0; j<pts.size(); j++)
	{

		geometry_msgs::Point32 pt = pts[j], rot_pt;
		rot_pt.x = ct * pt.x - st * pt.y + pose.x;
		rot_pt.y = st * pt.x + ct * pt.y + pose.y;
		final_pt[j] = rot_pt;
	}
	return final_pt;
}

geometry_msgs::Point32 ominus(geometry_msgs::Point32 point2, geometry_msgs::Point32 point1)
{
	double ctheta = cos(point1.z), stheta = sin(point1.z);
	geometry_msgs::Point32 relative_tf;
	relative_tf.x  = (point2.x - point1.x) * ctheta + (point2.y - point1.y) * stheta;
	relative_tf.y =  -(point2.x - point1.x) * stheta + (point2.y - point1.y) * ctheta;
	relative_tf.z = point2.z - point1.z;
	//cout<<relative_tf<<endl;
	return relative_tf;
}


int main(int argc, char **argcv)
{

	fmutil::Stopwatch sw;
	sw.start("isam_full");
	ros::init(argc, argcv, "RasterMapImage");
	ros::NodeHandle nh;
	ros::Publisher src_pub, dst_pub, query_pub, overall_pub;
	src_pub = nh.advertise<sensor_msgs::PointCloud>("src_pc", 5);
	dst_pub = nh.advertise<sensor_msgs::PointCloud>("dst_pc", 5);
	query_pub = nh.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 5);
	overall_pub = nh.advertise<sensor_msgs::PointCloud>("pc_graph_overall", 5);
	sensor_msgs::PointCloud src_pc, dst_pc, query_pc;
	src_pc.header.frame_id = dst_pc.header.frame_id = query_pc.header.frame_id = "scan_odo";

	istream* data_in = NULL, *pc_data_in = NULL;         // input for data points

	vector<sensor_msgs::PointCloud> pc_vec;
	sensor_msgs::PointCloud pc;
	ifstream dataStreamSrc, pcStreamSrc;
	dataStreamSrc.open(argcv[1], ios::in);// open data file
	if (!dataStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	pcStreamSrc.open(argcv[2], ios::in);// open data file
	if (!pcStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	data_in = &dataStreamSrc;
	pc_data_in = &pcStreamSrc;

	double vec_no;
	readHeader(*data_in, vec_no);
	vector< vector<double> > scores_array;

	int skip_reading = 2;
	uint scores_size = ceil(vec_no/skip_reading);
	for(size_t i=0; i<scores_size; i++)
	{
		vector<double> scores_temp;
		scores_temp.resize(scores_size);
		if(!readScores(*data_in, scores_temp))
		{
			cout<<"Unexpected end of data at score line "<<i<<endl;
			exit(1);
		}
		scores_array.push_back(scores_temp);
	}
	cout<<scores_array[0][0]<<" "<<scores_array[scores_size-1][scores_size-1]<<endl;
	cout<<"Successfully read all the scores"<<endl;

	int size = vec_no;
	vector<geometry_msgs::Point32> poses;
	if(!readFrontEndFile(*pc_data_in, pc_vec, poses)) cout<<"Failed read front end file"<<endl;

	if(size == (int)pc_vec.size())
		cout <<"All system green, going for matching"<<endl;
	else
		cout << "Failed in checking size of pc_vec and scores"<<endl;


	// instance of the main class that manages and optimizes the pose graph
	isam::Slam slam;

	// locally remember poses
	vector<isam::Pose2d_Node*> pose_nodes;

	Eigen::MatrixXd eigen_noise(3,3);
	eigen_noise(0,0) = 50;
	eigen_noise(1,1) = 50;
	eigen_noise(2,2) = 900;

	isam::Noise noise3 = isam::Information(eigen_noise);
	isam::Noise noise2 = isam::Information(100. * isam::eye(2));


	// create a first pose (a node)
	isam::Pose2d_Node* new_pose_node = new isam::Pose2d_Node();
	// add it to the graph
	slam.add_node(new_pose_node);
	// also remember it locally
	pose_nodes.push_back(new_pose_node);
	// create a prior measurement (a factor)
	isam::Pose2d origin(0., 0., 0.);
	isam::Pose2d_Factor* prior = new isam::Pose2d_Factor(pose_nodes[0], origin, noise3);
	// add it to the graph
	slam.add_factor(prior);

	ros::Rate rate(2);

	GraphParticleFilter graphPF(scores_array, &slam, &pc_vec, 100, skip_reading);
	sensor_msgs::PointCloud overall_pts;
	double opt_error = 0;
	for(int i=skip_reading; i<size; i+=skip_reading)
	{
		cout<<"**************************"<<endl;
		fmutil::Stopwatch sw("overall", true);
		RasterMapPCL rmpcl;
		vector<geometry_msgs::Point32> combines_prior, prior_m5, prior_p5;
		geometry_msgs::Point32 odo = ominus(poses[i], poses[i-skip_reading]);
		isam::Pose2d_Node* new_pose_node = new isam::Pose2d_Node();
		slam.add_node(new_pose_node);
		pose_nodes.push_back(new_pose_node);
		// connect to previous with odometry measurement
		isam::Pose2d odometry(odo.x, odo.y, odo.z); // x,y,theta

		transform_info odo_tf;
		odo_tf.translation_2d.x = odo.x;
		odo_tf.translation_2d.y = odo.y;
		odo_tf.rotation = odo.z;
		/*RasterMapPCL rmpcl_odo;
		rmpcl_odo.setInputPts(pc_vec[i-skip_reading].points);
		cv::Mat odo_cov = rmpcl_odo.getCovarianceWithTf(pc_vec[i], odo_tf);
		Eigen::MatrixXd eigen_noise(3,3);
		for(int k=0; k<3; k++)
			for(int j=0; j<3; j++)
				eigen_noise(k,j) = odo_cov.at<float>(k,j);
		noise3 = isam::Information(100. * isam::eye(3));*/

		isam::Pose2d_Pose2d_Factor* constraint = new isam::Pose2d_Pose2d_Factor(pose_nodes[i/skip_reading-1], pose_nodes[i/skip_reading], odometry, noise3);
		slam.add_factor(constraint);

		fmutil::Stopwatch sw_cl("close_loop", true); //~100 ms Raster map causes it
		int cl_node_idx = graphPF.getCloseloop(i);
		sw_cl.end();

		fmutil::Stopwatch sw_foundcl("found_cl", true);
		if(cl_node_idx != -1)
		{
			fmutil::Stopwatch sw_found_cl_a("found_cl_a");
			rmpcl.setInputPts(pc_vec[i].points);
			RasterMapPCL rmpcl_ver;
			int j= cl_node_idx;
			transform_info best_tf = rmpcl.getBestTf(pc_vec[j]);

			//verification
			rmpcl_ver.setInputPts(best_tf.real_pts, true);
			double temp_score = rmpcl_ver.getScore(pc_vec[i].points);
			double ver_score = sqrt(temp_score * best_tf.score);

			src_pc.points = pc_vec[i].points;
			query_pc.points = pc_vec[j].points;


			cv::Mat cov = best_tf.covariance;
			cout<<"CL Cov: "<<endl;
			cout<<cov<<endl;
			dst_pc.points.clear();
			for(size_t k=0; k<best_tf.real_pts.size();k++)
			{

				geometry_msgs::Point32 pt;
				pt.x = best_tf.real_pts[k].x;
				pt.y = best_tf.real_pts[k].y;
				dst_pc.points.push_back(pt);
			}

			src_pc.header.stamp = dst_pc.header.stamp = query_pc.header.stamp = ros::Time::now();

			for(int k=0; k<3; k++)
			{
				src_pub.publish(src_pc);
				dst_pub.publish(dst_pc);
				query_pub.publish(query_pc);
				ros::spinOnce();
			}


			cout<<"Match found at "<<i<<" "<<j<<" with score "<<best_tf.score <<" recorded "<<scores_array[i/skip_reading][j/skip_reading] <<" ver_score "<<ver_score<<" "<<temp_score<<endl;
			//if(temp_score < 55)continue;
			cout<<i<<" "<<j<<" "<<best_tf.translation_2d.x<<" "<<best_tf.translation_2d.y<<" "<<best_tf.rotation<<" ";
			//cout<<cov.at<float>(0,0)<<" "<<cov.at<float>(0,1)<<" "<<cov.at<float>(0,2)<<" "<<cov.at<float>(1, 1)<<" "<<cov.at<float>(1,2)<<" "<<cov.at<float>(2,2);
			//cout<<" "<<endl;
			string s;
			//getline(cin, s);
			if(s.size() > 0)
			{
				if(s[0] == 'x') return 0;

			}
			else
			{

				isam::Pose2d odometry(best_tf.translation_2d.x, best_tf.translation_2d.y, best_tf.rotation); // x,y,theta

				Eigen::MatrixXd eigen_noise(3,3);
				for(int k=0; k<3; k++)
					for(int j=0; j<3; j++)
						eigen_noise(k,j) = cov.at<float>(k,j);
				isam::Noise noise = isam::Information(eigen_noise);
				isam::Pose2d_Pose2d_Factor* constraint = new isam::Pose2d_Pose2d_Factor(pose_nodes[i/skip_reading], pose_nodes[j/skip_reading], odometry, noise);
				slam.add_factor(constraint);
				int iteration = slam.batch_optimization();
				double diff_opt_error = 0;
				if(opt_error == 0)
				{
					opt_error = slam._opt.opt_error_;
				}
				else
				{
					diff_opt_error = opt_error - slam._opt.opt_error_;

				}

				//slam._opt.
				//seems to be simple addition, but improve things tremendously
				cout<<"Pre Opt error: "<<opt_error<<" Now Opt error "<<slam._opt.opt_error_<<" diff: "<<diff_opt_error<<endl;

				if(fabs(diff_opt_error) > 1.0)
				{
					cout<<"Removing factor"<<endl;
					slam.remove_factor(constraint);
					slam.update();
					slam.batch_optimization();
					cout<<"Batch optimized again"<<endl;
				}
				else
				{
					opt_error = slam._opt.opt_error_;
				}
			}
		}
		sw_foundcl.end();

		fmutil::Stopwatch sw_opt("optimization", true);
		// optimize the graph
		slam.batch_optimization();
		sw_opt.end();

		//output and downsampling occupy  when there is no close loop found
		fmutil::Stopwatch sw_out("output", true); //~130 ms
		list<isam::Node*> nodes = slam.get_nodes();
		overall_pts.points.clear();
		overall_pts.header.frame_id = "scan_odo";
		int node_idx=0;
		for(std::list<isam::Node*>::const_iterator it = nodes.begin(); it!=nodes.end(); it++) {
			isam::Node& node = **it;
			geometry_msgs::Point32 estimated_pt;
			estimated_pt.x = node.vector(isam::ESTIMATE)[0];
			estimated_pt.y = node.vector(isam::ESTIMATE)[1];
			estimated_pt.z = node.vector(isam::ESTIMATE)[2];
			//cout<<estimated_pt.x << " "<< estimated_pt.y<< " "<<estimated_pt.z<<endl;
			vector<geometry_msgs::Point32> tfed_pts = getTransformedPts(estimated_pt, pc_vec[node_idx++*skip_reading].points);
			overall_pts.points.insert(overall_pts.points.end(), tfed_pts.begin(), tfed_pts.end());
		}
		sw_out.end();
		fmutil::Stopwatch sw_ds("downsampling", true);
		RasterMapImage rmi(1,1);
		/*RenderMap rmap;
		rmap.drawMap(overall_pts.points, 0.05); //~450 ms, understably as the points exceeding 400k pts after downsampling
		vector<cv::Point2f> mapped_pts = rmap.mapToRealPts();
		vector<geometry_msgs::Point32> downsampled_pt;
		downsampled_pt.resize(mapped_pts.size());
		for(size_t i=0; i<mapped_pts.size(); i++)
		{
			downsampled_pt[i].x = mapped_pts[i].x;
			downsampled_pt[i].y = mapped_pts[i].y;
		}*/
		vector<geometry_msgs::Point32> downsampled_pt = rmi.pcl_downsample(overall_pts.points, 0.05, 0.05, 0.05);

		overall_pts.points = downsampled_pt;
		stringstream ss;
		ss<<"isam_map_progress"<<i<<".png";
		//RenderMap rm;
		//rm.drawMap(overall_pts.points, 0.05, ss.str());
		for(int k=0; k<3; k++)
		{
			overall_pts.header.stamp = ros::Time::now();
			overall_pub.publish(overall_pts);

			ros::spinOnce();
		}
		sw_ds.end();

		sw.end();
		cout<<"***********************************"<<endl;
	}
	RenderMap rm;
	rm.drawMap(overall_pts.points, 0.05, "isam_map_overall.png");
	// printing the complete graph
	cout << endl << "Full graph:" << endl;
	slam.write(cout);
	sw.end();
	return 0;
}
