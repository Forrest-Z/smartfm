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

    float cluster_vel_bound_x_;
    float cluster_vel_bound_y_;
    float cluster_pos_bound_x_;
    float cluster_pos_bound_y_;
    
//     vision_opticalflow::Clusters clusters_;      //vector of cluster
    void clusterCallback(const vision_opticalflow::Feature::ConstPtr & features_msg);
};

FeatureClusteringNode::FeatureClusteringNode()
{
    feature_sub_ = nh_.subscribe("features",10, &FeatureClusteringNode::clusterCallback, this);
    cluster_pub_ = nhp_.advertise<vision_opticalflow::Clusters>("clusters", 10);

    cluster_vel_bound_x_ = 3;
    cluster_vel_bound_y_ = 3;
    cluster_pos_bound_x_ = 50;
    cluster_pos_bound_y_ = 50;
}

void FeatureClusteringNode::clusterCallback(const vision_opticalflow::Feature::ConstPtr & features_msg)
{
    geometry_msgs::Point feature_vel;
//     vision_opticalflow::Cluster cluster_msg;    //single cluster
    vision_opticalflow::Cluster cluster_right;
    vision_opticalflow::Cluster cluster_left;
    vision_opticalflow::Clusters clusters_;     //vector of cluster

    clusters_.clusters_info.push_back(cluster_right);   //0 is right
    clusters_.clusters_info.push_back(cluster_left);    //1 is left
    
    clusters_.header = features_msg->header;
//     for (unsigned int i = 0; i < features_msg->found_feature.size(); i++)
//     {
//         if((features_msg->feature_vel[i].x < 2) && (features_msg->feature_vel[i].y < 2))
//         {
//             continue;
//         }
//         
//         if(clusters_.clusters_info.size() == 0)
//         {
//             //intialize clusters_
//             std::cout << " intialize clusters_ " << std::endl;
//             vision_opticalflow::Cluster cluster_msg;    //single cluster
// 
//             cluster_num = cluster_num + 1;
//             cluster_msg.id = cluster_num;
//             cluster_msg.centroid = zero_point;
//             cluster_msg.centroid_vel = zero_point;
//             cluster_msg.members_vel.push_back(zero_point);
//             cluster_msg.members.push_back(zero_point);
// 
//             clusters_.clusters_info.push_back(cluster_msg);
// //             return;
//         }
// 
//         bool need_new_cluster = true;
//         for(unsigned int j = 0; j < clusters_.clusters_info.size(); j++){
//             if((abs(features_msg->found_feature[i].x - clusters_.clusters_info[j].centroid.x) <= cluster_pos_bound_x_) &&
//                (abs(features_msg->found_feature[i].y - clusters_.clusters_info[j].centroid.y) <= cluster_pos_bound_y_)
//             )
//             {
//                 //This feature is in the same cluster
// //                 std::cout << " vel_x: " << features_msg->feature_vel[i].x << std::endl;
// //                 std::cout << " vel_y: " << features_msg->feature_vel[i].y << std::endl;
// 
//                 clusters_.clusters_info[j].members.push_back(features_msg->found_feature[i]);
//                 clusters_.clusters_info[j].members_vel.push_back(features_msg->feature_vel[i]);
//                 clusters_.clusters_info[j].centroid.x = (clusters_.clusters_info[j].centroid.x + features_msg->found_feature[i].x)/2;
//                 clusters_.clusters_info[j].centroid_vel.x = (clusters_.clusters_info[j].centroid_vel.x + features_msg->feature_vel[i].x)/2;
//                 clusters_.clusters_info[j].centroid.y = (clusters_.clusters_info[j].centroid.y + features_msg->found_feature[i].y)/2;
//                 clusters_.clusters_info[j].centroid_vel.y = (clusters_.clusters_info[j].centroid_vel.y + features_msg->feature_vel[i].y)/2;
//                 need_new_cluster = false;
//             }
//         }
// 
//         if(need_new_cluster == true)
//         {
//             vision_opticalflow::Cluster cluster_msg;    //single cluster
// 
//             cluster_num = cluster_num + 1;
//             cluster_msg.id = cluster_num;
//             cluster_msg.centroid = features_msg->found_feature[i];
//             cluster_msg.centroid_vel = features_msg->feature_vel[i];
//             cluster_msg.members_vel.push_back(features_msg->feature_vel[i]);
//             cluster_msg.members.push_back(features_msg->found_feature[i]);
// 
//             clusters_.clusters_info.push_back(cluster_msg);
//             need_new_cluster = false;
//         }
// 
//     }
    for (unsigned int i = 0; i < features_msg->found_feature.size(); i++)
    {
        if((abs(features_msg->feature_vel[i].x) < 2) && (abs(features_msg->feature_vel[i].y) < 2))
        {
            //ignore none moving feature
            continue;
        }

            if(features_msg->feature_vel[i].x > 0)
            {
                clusters_.clusters_info[0].members.push_back(features_msg->found_feature[i]);
                clusters_.clusters_info[0].members_vel.push_back(features_msg->feature_vel[i]);
                clusters_.clusters_info[0].centroid.x = (clusters_.clusters_info[0].centroid.x + features_msg->found_feature[i].x)/2;
                clusters_.clusters_info[0].centroid_vel.x = (clusters_.clusters_info[0].centroid_vel.x + features_msg->feature_vel[i].x)/2;
                clusters_.clusters_info[0].centroid.y = (clusters_.clusters_info[0].centroid.y + features_msg->found_feature[i].y)/2;
                clusters_.clusters_info[0].centroid_vel.y = (clusters_.clusters_info[0].centroid_vel.y + features_msg->feature_vel[i].y)/2;
            }else{
                clusters_.clusters_info[1].members.push_back(features_msg->found_feature[i]);
                clusters_.clusters_info[1].members_vel.push_back(features_msg->feature_vel[i]);
                clusters_.clusters_info[1].centroid.x = (clusters_.clusters_info[1].centroid.x + features_msg->found_feature[i].x)/2;
                clusters_.clusters_info[1].centroid_vel.x = (clusters_.clusters_info[1].centroid_vel.x + features_msg->feature_vel[i].x)/2;
                clusters_.clusters_info[1].centroid.y = (clusters_.clusters_info[1].centroid.y + features_msg->found_feature[i].y)/2;
                clusters_.clusters_info[1].centroid_vel.y = (clusters_.clusters_info[1].centroid_vel.y + features_msg->feature_vel[i].y)/2;
            }
    }
    cluster_pub_.publish(clusters_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_clustering");
    FeatureClusteringNode node;
    ros::spin();
    return 0;
}
