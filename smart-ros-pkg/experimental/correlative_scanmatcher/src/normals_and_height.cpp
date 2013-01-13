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


#include <popot/rng_generators.h>

typedef popot::rng::CRNG RNG_GENERATOR;
#include <popot/popot.h>
#include <stdio.h>
#include <iostream>

#include "heightmatching_pso.h"
typedef NormalsAndHeightMatchingProblem<3> Problem;
typedef popot::PSO::initializer::PositionUniformRandom PositionInit;
typedef popot::PSO::initializer::VelocityZero VelocityInit;
typedef popot::PSO::particle::ModifiedBareboneParticle<Problem, PositionInit, VelocityInit> Particle;
typedef popot::PSO::topology::RandomInformants<60, 3, Particle> Topology;
class PSO_Params
{
public:
  static bool random_shuffle() { return false;}
  static int evaluation_mode() {return popot::PSO::algorithm::ASYNCHRONOUS_EVALUATION;}
};
class StopCriteria
{
public:
  static bool stop(double fitness, int epoch)
  {
    return Problem::stop(fitness, epoch);
  }
};
typedef popot::PSO::algorithm::Base<PSO_Params, Particle, Topology, StopCriteria> PSO;



double normal_thres_ = 0.5;
double norm_radius_search_ = 0.2;
double density_radius_search_ = 0.2;
double density_min_neighbors_ = 5;

pcl::PointCloud<pcl::PointNormal> normalEstimation(pcl::PointCloud<pcl::PointXYZ>& input)
{
	//if(input.points.size() == 0) return;
	//down sampling moved to rolling windows
	fmutil::Stopwatch sw;

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

	ne.setInputCloud (input.makeShared());

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal> cloud_normals;

	// Use all neighbors in a sphere of radius 10cm
	ne.setRadiusSearch (norm_radius_search_);

	// Set view point
	ne.setViewPoint(0.,0.,0.);//viewpoint[0], viewpoint[1], viewpoint[2]);
	// Compute the features
	ne.compute (cloud_normals);

	// concatentate the fileds
	pcl::PointCloud<pcl::PointNormal> point_normals;
	sw.start("Concatenate fields");
	pcl::concatenateFields(input, cloud_normals, point_normals);
	sw.end();
	//cout<<"Point normal "<<point_normals.points[0].normal_x<<' '<<point_normals.points[0].normal_y<<' '<<point_normals.points[0].normal_z<<endl;

	//publish normals as pose arrays if needed
	return point_normals;

}

pcl::PointCloud<pcl::PointNormal> filterNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud)
{
	fmutil::Stopwatch sw;
	stringstream ss;
	ss<<"Filter clouds with "<<pcl_cloud.size()<<" pts";
	sw.start(ss.str());

	//absolutely stunningly quick!
	pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointNormal> ());
	range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
			pcl::FieldComparison<pcl::PointNormal> ("normal_z", pcl::ComparisonOps::LT, normal_thres_)));
	//range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
	//      pcl::FieldComparison<pcl::PointNormal> ("z", pcl::ComparisonOps::GT, 0.0)));
	pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond);
	condrem.setInputCloud (pcl_cloud.makeShared());
	condrem.filter (pcl_cloud);

	pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond2 (new pcl::ConditionAnd<pcl::PointNormal> ());
	range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
			pcl::FieldComparison<pcl::PointNormal> ("normal_z", pcl::ComparisonOps::GT, -normal_thres_)));
	//range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
	//      pcl::FieldComparison<pcl::PointNormal> ("z", pcl::ComparisonOps::GT, 0.0)));
	pcl::ConditionalRemoval<pcl::PointNormal> condrem2 (range_cond2);
	condrem2.setInputCloud (pcl_cloud.makeShared());
	condrem2.filter (pcl_cloud);
	sw.end();
	cout<<"After normal filter "<<pcl_cloud.size()<<endl;

	if(pcl_cloud.size()==0) return pcl_cloud;

	//perform density based filtering
	sw.start("Density filtering");
	pcl::PointCloud<pcl::PointNormal> radius_filtered_pcl2;// = pcl_cloud;
	pcl::RadiusOutlierRemoval<pcl::PointNormal> outrem;
	outrem.setInputCloud(pcl_cloud.makeShared());
	outrem.setRadiusSearch(density_radius_search_);
	outrem.setMinNeighborsInRadius(density_min_neighbors_);
	outrem.filter(radius_filtered_pcl2);
	sw.end();
	cout<<"Remaining clouds: "<<radius_filtered_pcl2.size()<<endl;
	return radius_filtered_pcl2;
}

sensor_msgs::PointCloud processNormals(sensor_msgs::PointCloud &src)
{
	sensor_msgs::PointCloud2 src_pc2;
	sensor_msgs::PointCloud out;
	pcl::PointCloud<pcl::PointXYZ> src_pcl;
	sensor_msgs::convertPointCloudToPointCloud2(src, src_pc2);
	pcl::fromROSMsg(src_pc2, src_pcl);
	pcl::PointCloud<pcl::PointNormal> src_pointnorm = normalEstimation(src_pcl);
	src_pointnorm = filterNormal(src_pointnorm);

	pcl::toROSMsg(src_pointnorm, src_pc2);
	sensor_msgs::convertPointCloud2ToPointCloud(src_pc2, out);
	cout<<"before downsample "<<out.points.size()<<endl;
	out.points = pcl_downsample(out.points, 0.1, 0.1, 20.);
	cout<<"after downsample "<<out.points.size()<<endl;
	return out;
}
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

	sensor_msgs::PointCloud src_norm, dst_norm;
	src_norm = processNormals(src);
	dst_norm = processNormals(dst);
	fmutil::Stopwatch sw("start overall");
	RasterMapImage rm(0.05, 0.03);
	rm.getInputPoints(src_norm.points, src.points);
	RNG_GENERATOR::rng_srand();
	RNG_GENERATOR::rng_warm_up();

	// To keep track of the best particle ever found
	PSO::BestType best_particle;
	Problem::init(src.points, src_norm.points, dst_norm.points);


	fmutil::Stopwatch sw2("pso");
	ros::Duration ros_sleep(0.1);
	ros::NodeHandle nh;

	PSO pso;
	pso.run(1);
	sw2.end();
	// Some display
	std::cout << "Best particle : " << pso.getBest().getPosition(0) << " "<< pso.getBest().getPosition(1)<<" "<<pso.getBest().getPosition(2)<< std::endl;
	//exit(0);
	vector<cv::Point2f>  query_pts;
	query_pts.resize(dst_norm.points.size());

	for(size_t i=0; i<dst_norm.points.size(); i++)
	{
		query_pts[i].x =  dst_norm.points[i].x;
		query_pts[i].y =  dst_norm.points[i].y;
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

	ros::Publisher src_pub, dst_pub, query_pub, icp_pub;
	src_pub = nh.advertise<sensor_msgs::PointCloud>("src_pc", 5);
	dst_pub = nh.advertise<sensor_msgs::PointCloud>("dst_pc", 5);
	query_pub = nh.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 5);
	icp_pub = nh.advertise<sensor_msgs::PointCloud2>("icp_out", 5);
	sensor_msgs::PointCloud src_pc, dst_pc, query_pc;
	sensor_msgs::PointCloud2 icp_pc;
	src_pc = src;
	query_pc = dst;

	geometry_msgs::Pose match_tf;
	match_tf.position.x = pso.getBest().getPosition(0);//best_tf[0].translation_2d.x;
	match_tf.position.y = pso.getBest().getPosition(1);//best_tf[0].translation_2d.y;
	match_tf.orientation.z = pso.getBest().getPosition(2);//best_tf[0].rotation;
	dst_pc.points = getTransformedPts(match_tf, query_pc.points);

	icp_pc.header.frame_id = src_pc.header.frame_id = dst_pc.header.frame_id = query_pc.header.frame_id = "scan_odo";
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
