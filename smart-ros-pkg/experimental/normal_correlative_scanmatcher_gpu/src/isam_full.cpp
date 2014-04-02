/*
 * RasterMapImageOptSample.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: demian
 */

/*#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


#include <geometry_msgs/Pose.h>

#include <pcl/ros/conversions.h>
#include <fmutil/fm_stopwatch.h>
#include <fmutil/fm_math.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
*/

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <occupancy_grid_utils/ray_tracer.h>
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <pcl/common/transforms.h>
#include "NormalCorrelativeMatching.h"
#include "geometry_helper.h"
#include "readfrontend.h"
#include <isam/isam.h>
//#include "mysql_helper.h"
#include "GraphPF.h"

//#include "pcl_downsample.h"

typedef boost::shared_ptr<occupancy_grid_utils::LocalizedCloud> CloudPtr;
typedef boost::shared_ptr<occupancy_grid_utils::LocalizedCloud const> CloudConstPtr;
typedef boost::circular_buffer<CloudConstPtr> CloudBuffer;

void mat2RPY(const Eigen::Matrix3f& t, double& roll, double& pitch, double& yaw) {
    roll = atan2(t(2,1),t(2,2));
    pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
    yaw = atan2(t(1,0),t(0,0));
}

visualization_msgs::Marker getMarker(int id, int particle_no, string text, geometry_msgs::Pose pose)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = particle_no*2;
    marker.scale.y = particle_no*2;
    marker.scale.z = 0.1;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    return marker; 
}

void publishNodeIdVis(int id, string text, geometry_msgs::Pose pose, ros::Publisher &pub)
{
  
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    ros::Time ros_time = ros::Time::now();
    marker.header.stamp = ros_time;
    marker.ns = "node_id";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose.position.x;
    marker.pose.position.y = pose.position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 4.;
    marker.scale.y = 4.;
    marker.scale.z = 0.5;
    marker.color.r = 1.;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1.0;

    marker.text = text;
    pub.publish(marker);
}

void computeBoundingBox(vector<geometry_msgs::Point32> &overall_pts, double &sizeX, double &sizeY, double &minX, double &minY){
    minX = 1e99; minY = 1e99;
    double maxX = 1e-99, maxY = 1e-99;
    for(size_t i=0; i<overall_pts.size(); i++) {
      if(minX > overall_pts[i].x) minX = overall_pts[i].x;
      if(minY > overall_pts[i].y) minY = overall_pts[i].y;
      if(maxX < overall_pts[i].x) maxX = overall_pts[i].x;
      if(maxY < overall_pts[i].y) maxY = overall_pts[i].y;
    }
    sizeX = maxX - minX;
    sizeY = maxY - minY;
}

void sendOccupancy(vector<geometry_msgs::Point32> &overall_pts, CloudBuffer &clouds, ros::Publisher &map_pub)
{
    double sizeX, sizeY, minX, minY;
    computeBoundingBox(overall_pts, sizeX, sizeY, minX, minY);
    nav_msgs::OccupancyGrid grid_info;
    double resolution = 0.1;
    double min_occ = 0.07;
    double max_range = 80;
    grid_info.info.height = sizeY/resolution;
    grid_info.info.width = sizeX/resolution;
    geometry_msgs::Pose origin;
    origin.orientation.w = 1.0;
    origin.position.x = minX;
    origin.position.y = minY;
    grid_info.info.origin = origin;
    grid_info.info.resolution = resolution;
    grid_info.info.origin.orientation.w = 1.0;
    occupancy_grid_utils::OverlayClouds overlay =
      occupancy_grid_utils::createCloudOverlay(grid_info, "map", min_occ, max_range, 2);
    BOOST_FOREACH  (CloudConstPtr cloud, clouds)
    {
        //cout<<cloud->cloud.points.size()<<endl;
        occupancy_grid_utils::addCloud(&overlay, cloud);
    }
    nav_msgs::OccupancyGrid::ConstPtr grid = occupancy_grid_utils::getGrid(overlay);
    map_pub.publish(grid);
}

sensor_msgs::PointCloud pclToPC(pcl::PointCloud<pcl::PointNormal> &matching_cloud)
{
    sensor_msgs::PointCloud2 matching_pc2;
    sensor_msgs::PointCloud pc_vec;
    pcl::toROSMsg(matching_cloud, matching_pc2);
    sensor_msgs::convertPointCloud2ToPointCloud(matching_pc2, pc_vec);
    return pc_vec;
}

struct FactorGraph{
  int i, j;
  isam::Pose3d pose;
  isam::Noise noise;
  bool new_node;
  FactorGraph(int _i, int _j, isam::Pose3d _pose, isam::Noise _noise, bool _new_node=false):
  i(_i), j(_j), pose(_pose), noise(_noise), new_node(_new_node) {};
};

void removeLastFactor(isam::Slam **slam,
		      vector<FactorGraph> &factors,
		      map<int, isam::Pose3d_Node*> &pose_nodes,
		      int factor_idx = -1
		     ){
  cout<<"Memory address of isam before "<<*slam<<endl;
  delete *slam;
  *slam = new isam::Slam();
  pose_nodes.clear();
  (*slam)->_prop.max_iterations = 100;
  cout<<"Before erase there are "<<factors.size()<<" factors"<<endl;
  if(factor_idx<0)
    factors.erase(factors.end()-1);
  else
    factors.erase(factors.begin()+factor_idx);
  cout<<"After erase there are "<<factors.size()<<" factors"<<endl;
  for(size_t i=0; i<factors.size(); i++){
    isam::Pose3d_Node* new_pose_node = new isam::Pose3d_Node();
    if(factors[i].new_node){
      (*slam)->add_node(new_pose_node);
      pose_nodes[factors[i].j] = new_pose_node;
    }
    if(i==0){
      isam::Pose3d_Factor* factor = new isam::Pose3d_Factor(pose_nodes[factors[i].j], 
							    factors[i].pose, factors[i].noise);
      (*slam)->add_factor(factor);
    }
    else {
      isam::Pose3d_Pose3d_Factor* factor = 
      new isam::Pose3d_Pose3d_Factor(pose_nodes[factors[i].i], pose_nodes[factors[i].j],
					factors[i].pose, factors[i].noise);
      (*slam)->add_factor(factor);
    }
  }
  (*slam)->batch_optimization();
  cout<<"Memory address of isam after "<<*slam<<endl;
}

  
int main(int argc, char **argv) {

    fmutil::Stopwatch sw;
    sw.start("isam_full");
    ros::init(argc, argv, "RasterMapImage");
    ros::NodeHandle nh;
    ros::Publisher src_pub, dst_pub, query_pub, overall_pub, factors_pub, nodeid_pub;
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 5 );
    ros::Publisher matching_src = nh.advertise<sensor_msgs::PointCloud>("matching_src", 5);
    ros::Publisher matching_dst = nh.advertise<sensor_msgs::PointCloud>("matching_dst", 5);
    ros::Publisher map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    src_pub = nh.advertise<sensor_msgs::PointCloud>("src_pc", 5);
    dst_pub = nh.advertise<sensor_msgs::PointCloud>("dst_pc", 5);
    query_pub = nh.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 5);
    overall_pub = nh.advertise<sensor_msgs::PointCloud>("pc_graph_overall", 5);
    factors_pub = nh.advertise<nav_msgs::Path>("factors", 5);
    nodeid_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 5);
    sensor_msgs::PointCloud src_pc, dst_pc, query_pc;
    nav_msgs::Path factors_msg;
    factors_msg.header.frame_id = src_pc.header.frame_id = dst_pc.header.frame_id = query_pc.header.frame_id =
            "map";

    istream* data_in = NULL, *pc_data_in = NULL;        // input for data points

    sensor_msgs::PointCloud pc;
    ifstream dataStreamSrc, pcStreamSrc;

    vector<vector<double> > scores_array, rotations_array;
    int skip_reading = atoi(argv[2]);

    int start_node = 5;
    double res_ = 0.1;
    vector<pcl::PointCloud<pcl::PointNormal> > matching_clouds;
    
    string frontend_file = argv[1];
    int startfile_idx = frontend_file.find_last_of("/") + 1;
    string folder = frontend_file.substr(0, startfile_idx);
    frontend_file = frontend_file.substr(startfile_idx,
            frontend_file.size() - startfile_idx - 4);
    //MySQLHelper mysql("normal_scanmatch", frontend_file);

    vector<geometry_msgs::Pose> poses;
    vector<sensor_msgs::PointCloud> pc_vecs, pc_vecs_raw;
    readFrontEndFile(argv[1], poses);
    ofstream isam_graph_file("isam_graph.g2o");
    int cl_count = poses.size()/skip_reading+1;
    for (size_t j = 0; j < poses.size(); j+=skip_reading) {
        stringstream matching_file;
        matching_file << folder << setfill('0') << setw(5) << j
                << ".pcd";
        pcl::PointCloud<pcl::PointNormal> matching_cloud_raw, matching_cloud;
        pcl::io::loadPCDFile(matching_file.str(), matching_cloud_raw);
        matching_cloud = matching_cloud_raw;//pcl_downsample(matching_cloud_raw, res_ * 2, res_ * 2,
          //      res_ * 2);
        matching_clouds.push_back(matching_cloud_raw);
        
        pc_vecs.push_back(pclToPC(matching_cloud));
        pc_vecs_raw.push_back(pclToPC(matching_cloud_raw));
        cout<<"reading frontend "<<matching_file.str()<<"          \xd"<<flush;
    }
    cout<<endl;
    int size = poses.size()/skip_reading-6;//647;    
    // instance of the main class that manages and optimizes the pose graph
    isam::Slam *slam = new isam::Slam();
    slam->_prop.max_iterations = 100;
    // locally remember poses
    map<int, isam::Pose3d_Node*> pose_nodes;

    Eigen::MatrixXd eigen_noise(6, 6);
    eigen_noise(0, 0) = 50;
    eigen_noise(1, 1) = 50;
    eigen_noise(2, 2) = 100;
    eigen_noise(3, 3) = 900;
    eigen_noise(4, 4) = 900;
    eigen_noise(5, 5) = 900;
    Eigen::MatrixXd cl_noise(6, 6);
//     eigen_noise(0, 0) = 600;
//     eigen_noise(1, 1) = 600;
//     eigen_noise(2, 2) = 600;
//     eigen_noise(3, 3) = 600;
//     eigen_noise(4, 4) = 600;
//     eigen_noise(5, 5) = 600;
    isam::Noise noise6 = isam::Information(100. * isam::eye(6));

    isam::Noise noise3 = isam::Information(eigen_noise);

    isam::Noise noise2 = isam::Information(100. * isam::eye(2));

    // create a first pose (a node)
    isam::Pose3d_Node* new_pose_node = new isam::Pose3d_Node();
    // add it to the graph
    slam->add_node(new_pose_node);
    // also remember it locally
    pose_nodes[start_node-1] = (new_pose_node);
    
    vector<FactorGraph> factors;
    // create a prior measurement (a factor)
    isam::Pose3d origin(0., 0., 0., 0., 0., 0.);
    isam::Pose3d_Factor* prior = new isam::Pose3d_Factor(pose_nodes[start_node-1], origin,
            noise3);
    factors.push_back(FactorGraph(0, start_node-1, origin, noise3, true));
    // add it to the graph
    slam->add_factor(prior);
    isam_graph_file<<"VERTEX_SE2 "<<start_node-5<<" 0 0 0"<<endl;
    
    ros::Rate rate(2);

    GraphParticleFilter graphPF(50, skip_reading, argv[1]);
    sensor_msgs::PointCloud overall_pts;
    double opt_error = 0;
    bool previous_cl = false, current_cl = false;
    int previous_cl_count = 0;
    int last_closeloop_idx = -1;
    vector<int> added_closeloop_factor_idx;
    //comment out the cout part to only output node info
    for (int i = start_node; i < size; i ++) {
      if(!ros::ok()) exit(0);
        cout << "**************************" << endl;
        fmutil::Stopwatch sw("overall", true);
        //vector<geometry_msgs::Point32> combines_prior, prior_m5, prior_p5;
        geometry_msgs::Pose odo = ominus(poses[i*skip_reading], poses[i*skip_reading - skip_reading]);
        isam::Pose3d_Node* new_pose_node = new isam::Pose3d_Node();
        slam->add_node(new_pose_node);
        pose_nodes[i] = (new_pose_node);
        // connect to previous with odometry measurement
        isam::Pose3d odometry(odo.position.x, odo.position.y, odo.position.z,
                odo.orientation.z, 0., 0.); // x,y,theta

        isam::Pose3d_Pose3d_Factor* constraint = new isam::Pose3d_Pose3d_Factor(
                pose_nodes[i-1], pose_nodes[i],
                odometry, noise3);
	factors.push_back(FactorGraph(i-1, i, odometry, noise3, true));
	
        slam->add_factor(constraint);
	
	geometry_msgs::Pose odom = ominus(poses[i*skip_reading], poses[(start_node-1)*skip_reading]);
	isam_graph_file<<"VERTEX_SE2 "<<i-4<<" "<<odom.position.x<<" "<<odom.position.y<<" "
	<<odom.orientation.z<<endl;
	isam_graph_file<<"EDGE_SE2 "<<i-5<<" "<<i-4<<" "<<odo.position.x<<" "<<odo.position.y<<" "
	<<odo.orientation.z<<" 1000 0 0 1000 0 2000"<<endl;
	
        fmutil::Stopwatch sw_cl("close_loop", true); //~100 ms Raster map causes it
        //because first pose start with 00002.pcd
        int cl_node_idx = graphPF.getCloseloop(slam, i);
        sw_cl.end();
        fmutil::Stopwatch sw_foundcl("found_cl", true);
        //cl_node_idx = -1;
        if (cl_node_idx != -1) {
            fmutil::Stopwatch sw_found_cl_a("found_cl_a");
            int j = cl_node_idx;
            if(j<start_node) continue;

            src_pc.points = pc_vecs[i].points;
            //another first pose issue here
            query_pc.points = pc_vecs[j].points;

            //no covariance estimate for now
            //cv::Mat cov = best_tf.covariance;
            //cout << "CL Cov: " << endl;
            //cout << cov << endl;
            dst_pc.points.clear();
            ScoreData sd;
            sd.node_src = j*skip_reading;
            sd.node_dst = i*skip_reading;
            //sd.t = -sd.t;
            //mysql.getData(sd);
	    sd = graphPF.sd_;
	    cout<<"mysql.getData: "<<sd.x<<" "<<sd.y<<" "<<sd.t<<" "<<sd.score<<" "<<sd.score_ver<<" "<<sd.node_src<<" "<<sd.node_dst<<endl;
            pcl::PointCloud<pcl::PointNormal> matching_cloud = matching_clouds[j];
            Eigen::Matrix4f transform;
	  double d = sd.t/180*M_PI;
	  transform<<cos(d),-sin(d),0,sd.x,
		      sin(d),cos(d),0,sd.y,
		      0,0,1,0,
		      0,0,0,1;
            pcl::transformPointCloudWithNormals<pcl::PointNormal>(
                    matching_cloud, matching_cloud, transform);
            
            for (size_t k = 0; k < matching_cloud.points.size(); k++) {

                geometry_msgs::Point32 pt;
                pt.x = matching_cloud.points[k].x;
                pt.y =  matching_cloud.points[k].y;
                dst_pc.points.push_back(pt);
            }

            src_pc.header.stamp = dst_pc.header.stamp = query_pc.header.stamp =
                    ros::Time::now();

            for (int k = 0; k < 3; k++) {
                src_pub.publish(src_pc);
                dst_pub.publish(dst_pc);
                query_pub.publish(query_pc);
                ros::spinOnce();
            }
            
            cout << "Match found at " << i << " " << j << " with score "
                    << endl; //best_tf.score <<" recorded "<<scores_array[i/skip_reading][j/skip_reading] <<" ver_score "<<ver_score<<" "<<temp_score<<endl;
            //if(temp_score < 55)continue;
            cout << i << " " << j << " " << sd.x << " "
                    << sd.y << " " << sd.t
                    << " ";
            
	    isam_graph_file<<"VERTEX_SWITCH "<<cl_count<<" 1"<<endl;
	    isam_graph_file<<"EDGE_SWITCH_PRIOR "<<cl_count<<" 1 1.0"<<endl;
	    isam_graph_file<<"EDGE_SE2_SWITCHABLE "<<j-4<<" "<<i-4<<" "<<cl_count++<<" "<<sd.x<<" "<< sd.y<<" "<< sd.t<<" 0.1 0 0 0.1 0 0.25"<<endl;
	  
            //cout<<cov.at<float>(0,0)<<" "<<cov.at<float>(0,1)<<" "<<cov.at<float>(0,2)<<" "<<cov.at<float>(1, 1)<<" "<<cov.at<float>(1,2)<<" "<<cov.at<float>(2,2);
            //cout<<" "<<endl;
            string str;
            //cin >> str;
            isam::Pose3d odometry(sd.x,
                    sd.y, 0.00001, sd.t/180.0*M_PI,
                    0.00001, 0.00001); // x,y,theta

            isam::Noise noise = isam::Information(cl_noise);
            isam::Pose3d_Pose3d_Factor* cl_constraint =
                    new isam::Pose3d_Pose3d_Factor(pose_nodes[i],
                            pose_nodes[j], odometry, noise3);
	    factors.push_back(FactorGraph(i, j, odometry, noise3));
	    
            slam->add_factor(cl_constraint);
            slam->batch_optimization();
            double diff_opt_error = 0;
            diff_opt_error = opt_error - slam->_opt.opt_error_;

            //slam._opt.
            //seems to be simple addition, but improve things tremendously
            cout << "Pre Opt error: " << opt_error << " Now Opt error "
                    << slam->_opt.opt_error_ << " diff: " << diff_opt_error
                    << endl;

            if (fabs(diff_opt_error) > 10.0 || std::isnan(slam->_opt.opt_error_)) {
                cout << "Removing factors" << endl;
                //add to false close loop count at GraphPF
                //falseCL_node_[j/skip_reading]++;
		removeLastFactor(&slam, factors, pose_nodes);
                cout << "Batch optimized again checking address " <<slam<< endl;
                current_cl = false;

            } else {
                current_cl = true;
		previous_cl_count++;
		last_closeloop_idx = i;
		added_closeloop_factor_idx.push_back(factors.size()-1);
            }
        } else {
            current_cl = false;
        }
	
	//if last close loop is not part of the continuous close loop, remove it
	if(i-last_closeloop_idx == 3) {
	  if (previous_cl_count < 2) {
	  cout<< "Close loop not continuous, removing previous constraint"<< endl;
		removeLastFactor(&slam, factors, pose_nodes, added_closeloop_factor_idx[added_closeloop_factor_idx.size()-1]);
		previous_cl_count = 0;
	  }
	  
	}
	cout <<"previous_cl_count:"<<previous_cl_count<<" last_closeloop_idx"<<last_closeloop_idx<<endl;
	if(i-last_closeloop_idx > 3) previous_cl_count = 0;
	/*
        //check if 2 continuous close loop is found, if it is not, remove the previous close loop
        if (previous_cl) {
            if (!current_cl && previous_cl_count < 2) {
                cout
                        << "Close loop not continuous, removing previous constraint"
                        << endl;
                slam.remove_factor(previous_constraint);
                slam.update();
                previous_cl = false;
            }
        }

        if (current_cl) {
            cout
                    << "Storing current close loop for consistency check in the next iteration"
                    << endl;
            previous_constraint = current_constraint;
            previous_cl = true;
            previous_cl_count++;
        } else {
            previous_cl = false;
            previous_cl_count = 0;
        }*/

        sw_foundcl.end();

        fmutil::Stopwatch sw_opt("optimization", true);
        // optimize the graph
        slam->batch_optimization();
        //moving the error value outside of the optimization to ensure whatever last optimized error is recorded
        opt_error = slam->_opt.opt_error_;
        sw_opt.end();

        //output and downsampling occupy  when there is no close loop found
        fmutil::Stopwatch sw_out("output", true); //~130 ms
        list<isam::Node*> nodes = slam->get_nodes();

        overall_pts.points.clear();
        overall_pts.header.frame_id = "map";
        int node_idx = start_node-1;
        double last_height = -1;

        //only if using a MESH_RESOURCE marker type:

        visualization_msgs::MarkerArray marker_arr;
        sensor_msgs::PointCloud matching_src_pc, matching_dst_pc;
        int node_id = 4;
        CloudBuffer clouds(3000);
        graphPF.nodes_pose_.clear();
	
	//using Powell's dog leg optimizer that optionally a robust estimator (pseudo Huber) to deal with occasional outliers
	//hopefully solve the optimizer disintegration when an wrong node is added and removed
	//not using it, some modification needed to bring on the residual value
	//Now remove both the close loop factor and last odometry's node
// 	isam::Properties prop = slam.properties();
// 	prop.method = isam::DOG_LEG;
// 	slam.set_properties(prop);

        for (std::list<isam::Node*>::const_iterator it = nodes.begin();
                it != nodes.end(); it++, node_id++) {
            isam::Node& node = **it;
            geometry_msgs::Pose estimated_pt;
            estimated_pt.position.x = node.vector(isam::ESTIMATE)[0];
            estimated_pt.position.y = node.vector(isam::ESTIMATE)[1];
            estimated_pt.position.z = node.vector(isam::ESTIMATE)[2];
            estimated_pt.orientation.z = node.vector(isam::ESTIMATE)[3];
            estimated_pt.orientation.y = node.vector(isam::ESTIMATE)[4];
            estimated_pt.orientation.x = node.vector(isam::ESTIMATE)[5];
            last_height = estimated_pt.position.z;
            //cout<<estimated_pt.position.x << " "<< estimated_pt.position.y<< " "<<estimated_pt.position.z<<endl;
            vector<geometry_msgs::Point32> tfed_pts = getTransformedPts(
                    estimated_pt, pc_vecs[node_idx].points);
            if(node_id == cl_node_idx)
            	matching_dst_pc.points = tfed_pts;
            if(node_id == i)
            	matching_src_pc.points = tfed_pts;
            overall_pts.points.insert(overall_pts.points.end(),
                    tfed_pts.begin(), tfed_pts.end());
            stringstream ss;
            int node_particle_no = 0;
            map<int,int>::iterator node_particle = graphPF.unique_nodes_.find(node_id);
            if(node_particle != graphPF.unique_nodes_.end()) node_particle_no = node_particle->second;
            ss<<"node_"<<node_id<<"_"<<node_particle_no;
            graphPF.nodes_pose_[node_id] = estimated_pt.position;
            marker_arr.markers.push_back(getMarker(node_id,node_particle_no, ss.str(), estimated_pt));
            publishNodeIdVis(node_id, ss.str(), estimated_pt, nodeid_pub);

            //building final occupancy map
            CloudPtr localized_cloud(new occupancy_grid_utils::LocalizedCloud());
            localized_cloud->header.frame_id = "map";
            localized_cloud->sensor_pose.orientation = tf::createQuaternionMsgFromYaw(estimated_pt.orientation.z);
            geometry_msgs::Point p = estimated_pt.position;
            p.z = 0.0;
            localized_cloud->sensor_pose.position = p;
            localized_cloud->cloud = pc_vecs_raw[node_idx];
            clouds.push_back(localized_cloud);
            node_idx++;
        }

        //send occupancy map
        if(i==size-1)
          sendOccupancy(overall_pts.points, clouds, map_pub_);

        list<isam::Factor*> factors = slam->get_factors();
        factors_msg.poses.clear();
        factors_msg.header.stamp = ros::Time::now();
        for(std::list<isam::Factor*>::const_iterator it = factors.begin();
                it != factors.end(); it++){
            isam::Factor& factor = **it;
            vector<isam::Node*> nodes = factor.nodes();
            for(size_t i=0; i<nodes.size(); i++)
            {
                geometry_msgs::PoseStamped factor;
                factor.header = factors_msg.header;
                geometry_msgs::Pose estimated_pt;
                estimated_pt.position.x = nodes[i]->vector(isam::ESTIMATE)[0];
                estimated_pt.position.y = nodes[i]->vector(isam::ESTIMATE)[1];
                estimated_pt.position.z = nodes[i]->vector(isam::ESTIMATE)[2];
                estimated_pt.orientation.z =nodes[i]->vector(isam::ESTIMATE)[3];
                estimated_pt.orientation.y = nodes[i]->vector(isam::ESTIMATE)[4];
                estimated_pt.orientation.x = nodes[i]->vector(isam::ESTIMATE)[5];
                factor.pose = estimated_pt;
                factors_msg.poses.push_back(factor);
            }
        }
        cout << "Size of overall_pts = "<<overall_pts.points.size()<<endl;
        cout << "Last opt height = " << last_height << "pointcloud = "
                << overall_pts.points[overall_pts.points.size() - 1].z << endl;
        sw_out.end();
        fmutil::Stopwatch sw_ds("downsampling", true);
        RasterMapImage rmi(1, 1);
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
	vector<geometry_msgs::Point32> downsampled_pt; 
	//if(i > size - 10)
        downsampled_pt = pcl_downsample(
                overall_pts.points, 0.1, 0.1, 0.1);

        overall_pts.points = downsampled_pt;
        cout << "After downsampling = "
                << overall_pts.points.size() << endl;
        stringstream ss;
        ss << "isam_map_progress" << i << ".png";
        RenderMap rm;
	if(i > size - 10)
	  rm.drawMap(overall_pts.points, 0.1, ss.str());
        for (int k = 0; k < 3; k++) {
            overall_pts.header.stamp = ros::Time::now();
            overall_pub.publish(overall_pts);
            factors_pub.publish(factors_msg);
            vis_pub.publish( marker_arr );
            matching_dst_pc.header = matching_src_pc.header = overall_pts.header;
            matching_src.publish(matching_src_pc);
            matching_dst.publish(matching_dst_pc);
            ros::spinOnce();
        }
        sw_ds.end();

        sw.end();
        string name;
        //std::getline (std::cin,name);
        
        cout << "***********************************" << endl;
    }
    RenderMap rm;
    rm.drawMap(overall_pts.points, 0.1, "isam_map_overall.png");
    // printing the complete graph
    cout << endl << "Full graph:" << endl;
    slam->write(cout);
    sw.end();
    return 0;
}
