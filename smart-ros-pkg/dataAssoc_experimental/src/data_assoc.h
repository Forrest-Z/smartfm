/*
 * data_assoc.h
 *
 *  Created on: Sep 15, 2011
 *      Author: golfcar
 */

#ifndef DATA_ASSOC_H_
#define DATA_ASSOC_H_
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
#include <sensing_on_road/pedestrian_vision_batch.h>
#include <sensing_on_road/camera_project.h>
#include <feature_detection/clusters.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dataAssoc_experimental/PedDataAssoc.h>
#include <dataAssoc_experimental/PedDataAssoc_vector.h>
#include <geometry_msgs/PolygonStamped.h>
#include <fmutil/fm_math.h>

using namespace std;

#define NN_MATCH_THRESHOLD 1.0
#define NN_ANG_MATCH_THRESHOLD 0.0873 //5 degree

struct centroid_local_global
{
    geometry_msgs::Point32 global_centroid;
    geometry_msgs::Point32 local_centroid;
};

struct local_lPedInView
{
    int id;
    geometry_msgs::PointStamped location;
};
class data_assoc
{
public:
    data_assoc(int argc, char** argv);
    //data_assoc();
    ~data_assoc();
    void pedClustCallback(feature_detection::clustersConstPtr cluster_vector);
    void pedVisionCallback(sensing_on_road::pedestrian_vision_batchConstPtr pedestrian_vision_vector);
    void pedVisionAngularCallback(sensor_msgs::PointCloudConstPtr pedestrian_vision_angular);
    void publishPed();
    bool transformPointToGlobal(std_msgs::Header header, geometry_msgs::Point32 input_point, geometry_msgs::Point32& output_point);
    bool transformGlobalToLocal(geometry_msgs::PointStamped& global_point, geometry_msgs::PointStamped& local_point);
    void getLatestLaserCluster(vector<centroid_local_global> &clg_copy);
    void cleanUp();
    void increaseConfidence(int id);
    ros::Publisher pedPub_;
    ros::Publisher visualizer_;
    string frame_id_, global_frame_, camera_frame_;
    bool use_sim_time_;
    tf::TransformListener *listener_;
    tf::MessageFilter<feature_detection::clusters> * laser_tf_filter_;
    tf::MessageFilter<sensing_on_road::pedestrian_vision_batch> * vision_tf_filter_;
    tf::MessageFilter<sensor_msgs::PointCloud> * vision_angular_tf_filter_;
    message_filters::Subscriber<feature_detection::clusters> pedClustSub_;
    message_filters::Subscriber<sensing_on_road::pedestrian_vision_batch> pedVisionSub_;
    message_filters::Subscriber<sensor_msgs::PointCloud> pedVisionAngularSub_;
    sensing_on_road::pedestrian_vision_batch lPedInView;
    double time_out_, poll_inc_, poll_dec_, threshold_;
    camera_project::camera_projector projector;
    std::vector<geometry_msgs::Point32> laser_latest_global_, laser_latest_local_;
    //double robotx_, roboty_, robotspeedx_;//pedx_, pedy_;

    //double robotspeedx_;
    //int simLen, simNum;
    //string  ped_id_file, policy_file, model_file;
    //void parse_simul_config( fstream& configfile);

    //int policy_initialize();
    //void pedInitPose();

    //void initPedMOMDP(ped_momdp_sarsop::ped_local_frame ped_local);


    //ofstream* foutStream;

	//struct POSE
	//{
		//double x;
		//double y;
		//double yaw;
	//};

	int latest_id;

    //ros::Timer timer_;
};

#endif /* DATA_ASSOC_H_ */
