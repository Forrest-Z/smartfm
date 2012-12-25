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
#include "mysql_helper.h"

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

vector<geometry_msgs::Point32> getTransformedPts(geometry_msgs::Pose pose, vector<geometry_msgs::Point32>& pts)
		{
	double ct = cos(pose.orientation.z), st = sin(pose.orientation.z);
	vector<geometry_msgs::Point32> final_pt;
	final_pt.resize(pts.size());
	for(size_t j=0; j<pts.size(); j++)
	{

		geometry_msgs::Point32 pt = pts[j], rot_pt;
		rot_pt.x = ct * pt.x - st * pt.y + pose.position.x;
		rot_pt.y = st * pt.x + ct * pt.y + pose.position.y;
		rot_pt.z = pose.position.z;
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

geometry_msgs::Pose ominus(geometry_msgs::Pose point2, geometry_msgs::Pose point1)
{
	double ctheta = cos(point1.orientation.z), stheta = sin(point1.orientation.z);
	geometry_msgs::Pose relative_tf;
	relative_tf.position.x  = (point2.position.x - point1.position.x) * ctheta + (point2.position.y - point1.position.y) * stheta;
	relative_tf.position.y =  -(point2.position.x - point1.position.x) * stheta + (point2.position.y - point1.position.y) * ctheta;
	relative_tf.position.z = point2.position.z - point1.position.z;
	relative_tf.orientation.z = point2.orientation.z - point1.orientation.z;
	relative_tf.orientation.y = point2.orientation.y - point1.orientation.y;
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


	vector< vector<double> > scores_array;
	int skip_reading = 10;
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
		size = scores_array.size();
	}
	cout<<scores_array[0][0]<<" "<<scores_array[scores_array.size()-1][scores_array.size()-1]<<endl;
	cout<<"Successfully read all the scores"<<endl;

	pcStreamSrc.open(argcv[1], ios::in);// open data file
	if (!pcStreamSrc) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	pc_data_in = &pcStreamSrc;
	vector<geometry_msgs::Pose> poses;
	if(!readFrontEndFile(*pc_data_in, pc_vec, poses)) cout<<"Failed read front end file"<<endl;

	if(size == (int)(pc_vec.size()/skip_reading)+1)
		cout <<"All system green, going for matching"<<endl;
	else
		cout << "Failed in checking size of pc_vec and scores"<<endl;


	// instance of the main class that manages and optimizes the pose graph
	isam::Slam slam;

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

	GraphParticleFilter graphPF(scores_array, &slam, &pc_vec, 200, skip_reading);
	sensor_msgs::PointCloud overall_pts;
	double opt_error = 0;
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
		isam::Pose3d odometry(odo.position.x, odo.position.y, odo.position.z, odo.orientation.z, odo.orientation.y, 0.); // x,y,theta



		isam::Pose3d_Pose3d_Factor* constraint = new isam::Pose3d_Pose3d_Factor(pose_nodes[i/skip_reading-1], pose_nodes[i/skip_reading], odometry, noise3);
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


			isam::Pose3d odometry(best_tf.translation_2d.x, best_tf.translation_2d.y, 0.0, best_tf.rotation, 0.0, 0.0); // x,y,theta

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
			slam.batch_optimization();
			double diff_opt_error = 0;
			diff_opt_error = opt_error - slam._opt.opt_error_;

			//slam._opt.
			//seems to be simple addition, but improve things tremendously
			cout<<"Pre Opt error: "<<opt_error<<" Now Opt error "<<slam._opt.opt_error_<<" diff: "<<diff_opt_error<<endl;

			string s;
			getline(cin, s);
			if(s.size() > 0)
			{
				if(s[0] == 'x') return 0;
				if(s[0] == 'n')//fabs(diff_opt_error) > 5.0)
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
			vector<geometry_msgs::Point32> tfed_pts = getTransformedPts(estimated_pt, pc_vec[node_idx++*skip_reading].points);
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
		vector<geometry_msgs::Point32> downsampled_pt = rmi.pcl_downsample(overall_pts.points, 0.05, 0.05, 0.05);

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
	rm.drawMap(overall_pts.points, 0.05, "isam_map_overall.png");
	// printing the complete graph
	cout << endl << "Full graph:" << endl;
	slam.write(cout);
	sw.end();
	return 0;
}
