#include <ros/ros.h>
#include <math.h>

#include <vision_opticalflow/Cluster.h>
#include <vision_opticalflow/Clusters.h>

class ClustersTrackingNode
{
public:
    ClustersTrackingNode();

private:
    ros::NodeHandle nhp_, nh_;
    ros::Subscriber feature_sub_;
    ros::Publisher tracker_pub_;

    float cluster_vel_bound_x_;
    float cluster_vel_bound_y_;
    float cluster_pos_bound_x_;
    float cluster_pos_bound_y_;

//     vision_opticalflow::Clusters clusters_;      //vector of cluster
    void clusterCallback(const vision_opticalflow::Feature::ConstPtr & features_msg);
};