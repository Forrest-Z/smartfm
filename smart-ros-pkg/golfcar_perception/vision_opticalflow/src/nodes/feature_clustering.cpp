#include <ros/ros.h>
#include <math.h>

#include <vision_opticalflow/Feature.h>
#include <vision_opticalflow/Cluster.h> 
#include <vision_opticalflow/Clusters.h>

class FeatureClusteringNode
{
public:
    FeatureClusteringNode();
    
private:
    ros::NodeHandle nhp_, nh_;
    ros::Subscriber feature_sub_;
    ros::Publisher cluster_pub_;

//     vision_opticalflow::Clusters clusters;      //vector of cluster
    void clusterCallback(const vision_opticalflow::Feature::ConstPtr & features_msg);
};

FeatureClusteringNode::FeatureClusteringNode()
{
    feature_sub_ = nh_.subscribe("features",10, &FeatureClusteringNode::clusterCallback, this);
    cluster_pub_ = nhp_.advertise<vision_opticalflow::Clusters>("clusters", 10);    
}

void FeatureClusteringNode::clusterCallback(const vision_opticalflow::Feature::ConstPtr & features_msg)
{
    geometry_msgs::Point feature_vel;
//     std::cout << "clusterCallback" << std::endl;
    
    float prev_feature_x,prev_feature_y,found_feature_x,found_feature_y;
    vision_opticalflow::Cluster cluster_msg;    //single cluster
    for (unsigned int i = 0; i < features_msg->prev_feature.size(); i++)
    {
        prev_feature_x = features_msg->prev_feature[i].x;
        prev_feature_y = features_msg->prev_feature[i].y;
        found_feature_x = features_msg->found_feature[i].x;
        found_feature_y = features_msg->found_feature[i].y;
        
//         feature_vel.z = sqrt(pow((found_feature_x - prev_feature_x),2) + pow((found_feature_y - prev_feature_y),2));
        feature_vel.x = found_feature_x - prev_feature_x;
        feature_vel.y = found_feature_y - prev_feature_y;
        
        cluster_msg.members_vel.push_back(feature_vel);
        cluster_msg.members.push_back(features_msg->found_feature[i]);
//         feature_vel_vec.push_back(sqrt(pow((found_feature_x - prev_feature_x),2) + pow((found_feature_y - prev_feature_y),2)));
    }
//     cluster_msg.prev_feature = fea   ture_vel_vec;

    vision_opticalflow::Clusters clusters;
    clusters.header = features_msg->header;
    clusters.clusters_info.push_back(cluster_msg);
    cluster_pub_.publish(clusters);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_clustering");
    FeatureClusteringNode node;
    ros::spin();
    return 0;
}
