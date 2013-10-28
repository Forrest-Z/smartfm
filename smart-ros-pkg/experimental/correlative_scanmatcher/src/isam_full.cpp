/*
 * RasterMapImageOptSample.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include "pcl/ros/conversions.h"
#include "RasterMapPCL.h"
#include "ReadFileHelper.h"
#include <isam/isam.h>
#include "GraphPF.h"
#include "mysql_helper.h"
#include "pcl_downsample.h"
#include "geometry_helper.h"

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

	
	sensor_msgs::PointCloud pc;
	ifstream dataStreamSrc, pcStreamSrc;


	vector< vector<double> > scores_array, rotations_array;
	int skip_reading = 2;
	int size;

	if(argc > 2)
	{

		dataStreamSrc.open(argcv[2], ios::in);// open data file
		if (!dataStreamSrc) {
			cerr << "Cannot open data file\n";
			exit(1);
		}
		data_in = &dataStreamSrc;
		double vec_no;
		readHeader(*data_in, vec_no);
		size = vec_no;
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
	}
	else
	{
		MySQLHelper sql(skip_reading, "scanmatch_result", argcv[1]);
		scores_array = sql.retrieve_score();
		rotations_array = sql.retrieve_rotation();
		size = scores_array.size();
		assert(scores_array.size() == rotations_array.size());
	}
	cout<<scores_array[0][0]<<" "<<scores_array[scores_array.size()-1][scores_array.size()-1]<<endl;
	cout<<rotations_array[0][0]<<" "<<rotations_array[scores_array.size()-1][scores_array.size()-1]<<endl;
	for(int i=0; i<rotations_array.size(); i++)
	{
		for(int j=0; j<i; j++)
		{
			cout<<rotations_array[i][j]<<";";
		}
		cout<<endl;
	}
	exit(0);
	cout<<"Successfully read all the scores"<<endl;

	vector<vector<sensor_msgs::PointCloud> > pc_vecs;
  vector<geometry_msgs::Pose> poses;
	for(int i=0; i<3; i++)
	{
		stringstream ss;
		ss<<argcv[1]<<"_"<<i;
		istream*        data_in         = NULL;         // input for data points
		ifstream dataStreamSrc;
		cout<<"Opening "<<ss.str()<<endl;
		dataStreamSrc.open(ss.str().c_str(), ios::in);// open data file
		if (!dataStreamSrc) {
			cerr << "Cannot open data file\n";
			exit(1);
		}
		data_in = &dataStreamSrc;

		vector<sensor_msgs::PointCloud> pc_vec;
		if(!readFrontEndFile(*data_in, pc_vec, poses)) cout<<"Failed read front end file"<<endl;
		pc_vecs.push_back(pc_vec);
	}
	//size = ceil(pc_vecs[0].size()/(double)skip_reading);
	if(size == ceil(pc_vecs[0].size()/(double)skip_reading))
		cout <<"All system green, going for matching"<<endl;
	else
		cout << "Failed in checking size of pc_vec and scores"<<endl;


	// instance of the main class that manages and optimizes the pose graph
	isam::Slam slam;
	slam._prop.max_iterations = 250;
	// locally remember poses
	vector<isam::Pose3d_Node*> pose_nodes;

	Eigen::MatrixXd eigen_noise(6,6);
	eigen_noise(0,0) = 50;
	eigen_noise(1,1) = 50;
	eigen_noise(2,2) = 100;
	eigen_noise(3,3) = 900;
	eigen_noise(4,4) = 900;
	eigen_noise(5,5) = 900;

	isam::Noise noise3 = isam::Information(eigen_noise);


	isam::Noise noise2 = isam::Information(100. * isam::eye(2));


	// create a first pose (a node)
	isam::Pose3d_Node* new_pose_node = new isam::Pose3d_Node();
	// add it to the graph
	slam.add_node(new_pose_node);
	// also remember it locally
	pose_nodes.push_back(new_pose_node);
	// create a prior measurement (a factor)
	isam::Pose3d origin(0., 0., 0., 0., 0., 0.);
	isam::Pose3d_Factor* prior = new isam::Pose3d_Factor(pose_nodes[0], origin, noise3);
	// add it to the graph
	slam.add_factor(prior);

	ros::Rate rate(2);

	GraphParticleFilter graphPF(scores_array, rotations_array, &slam, pc_vecs, 200, skip_reading);
	sensor_msgs::PointCloud overall_pts;
	double opt_error = 0;
	isam::Pose3d_Pose3d_Factor *previous_constraint, *current_constraint;
	bool previous_cl= false, current_cl= false;
	int previous_cl_count = 0;
	for(int i=skip_reading; i<size*skip_reading; i+=skip_reading)
	{
		cout<<"**************************"<<endl;
		fmutil::Stopwatch sw("overall", true);
		RasterMapPCL rmpcl;
		//vector<geometry_msgs::Point32> combines_prior, prior_m5, prior_p5;
		geometry_msgs::Pose odo = ominus(poses[i], poses[i-skip_reading]);
		isam::Pose3d_Node* new_pose_node = new isam::Pose3d_Node();
		slam.add_node(new_pose_node);
		pose_nodes.push_back(new_pose_node);
		// connect to previous with odometry measurement
		isam::Pose3d odometry(odo.position.x, odo.position.y, odo.position.z, odo.orientation.z, 0., 0.); // x,y,theta



		isam::Pose3d_Pose3d_Factor* constraint = new isam::Pose3d_Pose3d_Factor(pose_nodes[i/skip_reading-1], pose_nodes[i/skip_reading], odometry, noise3);
		slam.add_factor(constraint);

		fmutil::Stopwatch sw_cl("close_loop", true); //~100 ms Raster map causes it
		int cl_node_idx = graphPF.getCloseloop(i);
		sw_cl.end();

		fmutil::Stopwatch sw_foundcl("found_cl", true);

		if(cl_node_idx != -1)
		{
			fmutil::Stopwatch sw_found_cl_a("found_cl_a");
      vector<sensor_msgs::PointCloud> input_pcs, output_pcs;
      for(int k=0; k<3; k++) input_pcs.push_back(pc_vecs[k][i]);
			rmpcl.setInputPts(input_pcs);
			RasterMapPCL rmpcl_ver;
			int j= cl_node_idx;
      for(int k=0; k<3; k++) output_pcs.push_back(pc_vecs[k][j]);
			transform_info best_tf = rmpcl.getBestTf(output_pcs);

			//verification
			rmpcl_ver.setInputPts(best_tf.real_pts, true);
			double temp_score = rmpcl_ver.getScore(pc_vecs[1][i].points);
			double ver_score = sqrt(temp_score * best_tf.score);

			src_pc.points = pc_vecs[2][i].points;
			query_pc.points = pc_vecs[2][j].points;


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


			cout<<"Match found at "<<i<<" "<<j<<" with score "<<endl;//best_tf.score <<" recorded "<<scores_array[i/skip_reading][j/skip_reading] <<" ver_score "<<ver_score<<" "<<temp_score<<endl;
			//if(temp_score < 55)continue;
			cout<<i<<" "<<j<<" "<<best_tf.translation_2d.x<<" "<<best_tf.translation_2d.y<<" "<<best_tf.rotation<<" ";
			//cout<<cov.at<float>(0,0)<<" "<<cov.at<float>(0,1)<<" "<<cov.at<float>(0,2)<<" "<<cov.at<float>(1, 1)<<" "<<cov.at<float>(1,2)<<" "<<cov.at<float>(2,2);
			//cout<<" "<<endl;


			isam::Pose3d odometry(best_tf.translation_2d.x, best_tf.translation_2d.y, 0.00001, best_tf.rotation, 0.00001, 0.00001); // x,y,theta

			Eigen::MatrixXd eigen_noise(6,6);
			for(int x=0; x<6; x++)
				for(int y=0; y<6; y++)
					eigen_noise(x,y) = 1e9;

			for(int x=0; x<2; x++)
				for(int y=0; y<2; y++)
					eigen_noise(x,y) = cov.at<float>(x,y);
			eigen_noise(0, 3) = cov.at<float>(0, 2);
			eigen_noise(1, 3) = cov.at<float>(1, 2);
			eigen_noise(3, 3) = cov.at<float>(2, 2);

			isam::Noise noise = isam::Information(eigen_noise);
			isam::Pose3d_Pose3d_Factor* constraint = new isam::Pose3d_Pose3d_Factor(pose_nodes[i/skip_reading], pose_nodes[j/skip_reading], odometry, noise3);
			slam.add_factor(constraint);
			current_constraint = constraint;
			slam.batch_optimization();
			double diff_opt_error = 0;
			diff_opt_error = opt_error - slam._opt.opt_error_;

			//slam._opt.
			//seems to be simple addition, but improve things tremendously
			cout<<"Pre Opt error: "<<opt_error<<" Now Opt error "<<slam._opt.opt_error_<<" diff: "<<diff_opt_error<<endl;

			if(fabs(diff_opt_error) > 15.0)
			{
				cout<<"Removing factor"<<endl;
				//add to false close loop count at GraphPF
				//falseCL_node_[j/skip_reading]++;
				slam.remove_factor(constraint);
				slam.update();
				slam.batch_optimization();
				cout<<"Batch optimized again"<<endl;
				current_cl = false;

			}
			else
			{
				current_cl = true;

			}
		}
		else
		{
			current_cl = false;
		}

		//check if 2 continuous close loop is found, if it is not, remove the previous close loop
		if(previous_cl)
		{
			if(!current_cl && previous_cl_count < 2)
			{
				cout<<"Close loop not continuous, removing previous constraint"<<endl;
				slam.remove_factor(previous_constraint);
				slam.update();
				previous_cl = false;
			}
		}

		if(current_cl)
		{
			cout<<"Storing current close loop for consistency check in the next iteration"<<endl;
			previous_constraint = current_constraint;
			previous_cl = true;
			previous_cl_count ++;
		}
		else
		{
			previous_cl = false;
			previous_cl_count = 0;
		}

		sw_foundcl.end();

		fmutil::Stopwatch sw_opt("optimization", true);
		// optimize the graph
		slam.batch_optimization();
		//moving the error value outside of the optimization to ensure whatever last optimized error is recorded
		opt_error = slam._opt.opt_error_;
		sw_opt.end();

		//output and downsampling occupy  when there is no close loop found
		fmutil::Stopwatch sw_out("output", true); //~130 ms
		list<isam::Node*> nodes = slam.get_nodes();
		overall_pts.points.clear();
		overall_pts.header.frame_id = "scan_odo";
		int node_idx=0;
		double last_height = -1;
		for(std::list<isam::Node*>::const_iterator it = nodes.begin(); it!=nodes.end(); it++) {
			isam::Node& node = **it;
			geometry_msgs::Pose estimated_pt;
			estimated_pt.position.x = node.vector(isam::ESTIMATE)[0];
			estimated_pt.position.y = node.vector(isam::ESTIMATE)[1];
			estimated_pt.position.z = node.vector(isam::ESTIMATE)[2];
			estimated_pt.orientation.z = node.vector(isam::ESTIMATE)[3];
			estimated_pt.orientation.y = node.vector(isam::ESTIMATE)[4];
			estimated_pt.orientation.x = node.vector(isam::ESTIMATE)[5];
			last_height = estimated_pt.position.z;
			//cout<<estimated_pt.x << " "<< estimated_pt.y<< " "<<estimated_pt.z<<endl;
			vector<geometry_msgs::Point32> tfed_pts = getTransformedPts(estimated_pt, pc_vecs[2][node_idx++*skip_reading].points);
			overall_pts.points.insert(overall_pts.points.end(), tfed_pts.begin(), tfed_pts.end());

		}

		cout<<"Last opt height = "<<last_height<<"pointcloud = "<<overall_pts.points[overall_pts.points.size()-1].z<<endl;
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
		vector<geometry_msgs::Point32> downsampled_pt = pcl_downsample(overall_pts.points, 0.1, 0.1, 0.1);

		overall_pts.points = downsampled_pt;
		cout<<"After downsampling = "<<overall_pts.points[overall_pts.points.size()-1].z<<endl;
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
	rm.drawMap(overall_pts.points, 0.1, "isam_map_overall.png");
	// printing the complete graph
	cout << endl << "Full graph:" << endl;
	slam.write(cout);
	sw.end();
	return 0;
}
