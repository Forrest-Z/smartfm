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

    cluster_vel_bound_x_ = 4;
    cluster_vel_bound_y_ = 8;
    cluster_pos_bound_x_ = 25;
    cluster_pos_bound_y_ = 50;
}

void FeatureClusteringNode::clusterCallback(const vision_opticalflow::Feature::ConstPtr & features_msg)
{
    geometry_msgs::Point feature_vel;
//     vision_opticalflow::Cluster cluster_msg;    //single cluster
    vision_opticalflow::Cluster cluster_right;
    vision_opticalflow::Cluster cluster_left;
    
    vision_opticalflow::Clusters clusters_vel;     //vector of clusters
    vision_opticalflow::Clusters clusters_pos;
    vision_opticalflow::Clusters clusters_pos_right, clusters_pos_left;     //vector of clusters
    
    clusters_vel.clusters_info.push_back(cluster_right);   //0 is right
    clusters_vel.clusters_info.push_back(cluster_left);    //1 is left
    
    clusters_vel.header = features_msg->header;
    clusters_pos_right.header = features_msg->header;
    clusters_pos_left.header = features_msg->header;
    clusters_pos.header = features_msg->header;
    unsigned int cluster_num = 0;

    for (unsigned int i = 0; i < features_msg->found_feature.size(); i++)
    {
        if((abs(features_msg->feature_vel[i].x) < 1) && (abs(features_msg->feature_vel[i].y) < 1))
        {
            //ignore none moving feature
            continue;
        }
        
        if(features_msg->direction[i] == 0)
        {
            //moving right cluster
            clusters_vel.clusters_info[0].members.push_back(features_msg->found_feature[i]);
            clusters_vel.clusters_info[0].members_vel.push_back(features_msg->feature_vel[i]);
            clusters_vel.clusters_info[0].centroid.x = (clusters_vel.clusters_info[0].centroid.x + features_msg->found_feature[i].x)/2;
            clusters_vel.clusters_info[0].centroid_vel.x = (clusters_vel.clusters_info[0].centroid_vel.x + features_msg->feature_vel[i].x)/2;
            clusters_vel.clusters_info[0].centroid.y = (clusters_vel.clusters_info[0].centroid.y + features_msg->found_feature[i].y)/2;
            clusters_vel.clusters_info[0].centroid_vel.y = (clusters_vel.clusters_info[0].centroid_vel.y + features_msg->feature_vel[i].y)/2;
        }else{
            //moving left cluster
            clusters_vel.clusters_info[1].members.push_back(features_msg->found_feature[i]);
            clusters_vel.clusters_info[1].members_vel.push_back(features_msg->feature_vel[i]);
            clusters_vel.clusters_info[1].centroid.x = (clusters_vel.clusters_info[1].centroid.x + features_msg->found_feature[i].x)/2;
            clusters_vel.clusters_info[1].centroid_vel.x = (clusters_vel.clusters_info[1].centroid_vel.x + features_msg->feature_vel[i].x)/2;
            clusters_vel.clusters_info[1].centroid.y = (clusters_vel.clusters_info[1].centroid.y + features_msg->found_feature[i].y)/2;
            clusters_vel.clusters_info[1].centroid_vel.y = (clusters_vel.clusters_info[1].centroid_vel.y + features_msg->feature_vel[i].y)/2;
//             std::cout << "left" << std::endl;
        }
    }
//     cluster_pub_.publish(clusters_vel);
        //right group
        for(unsigned int k = 0; k < clusters_vel.clusters_info[0].members.size(); k++)
        {
            if(clusters_pos_right.clusters_info.size() == 0)
            {
                //intialize clusters_
//                 std::cout << " intialize clusters_pos " << std::endl;
                vision_opticalflow::Cluster cluster_msg;    //single cluster

                cluster_num = cluster_num + 1;
                cluster_msg.id = cluster_num;
                cluster_msg.centroid = clusters_vel.clusters_info[0].members[k];
                cluster_msg.centroid_vel = clusters_vel.clusters_info[0].members_vel[k];
                cluster_msg.centroid_dir = 0;   // 0 -- moving in right direction
                cluster_msg.members_vel.push_back(clusters_vel.clusters_info[0].members_vel[k]);
                cluster_msg.members.push_back(clusters_vel.clusters_info[0].members[k]);

                clusters_pos_right.clusters_info.push_back(cluster_msg);
//             return;
            }

            bool need_new_cluster_right = true;
            for(unsigned int l = 0; l < clusters_pos_right.clusters_info.size(); l++){
                if((abs(clusters_vel.clusters_info[0].members[k].x - clusters_pos_right.clusters_info[l].centroid.x) <= cluster_pos_bound_x_) &&
                (abs(clusters_vel.clusters_info[0].members[k].y - clusters_pos_right.clusters_info[l].centroid.y) <= cluster_pos_bound_y_)
                )
                {
                    clusters_pos_right.clusters_info[l].members.push_back(clusters_vel.clusters_info[0].members[k]);
                    clusters_pos_right.clusters_info[l].members_vel.push_back(clusters_vel.clusters_info[0].members_vel[k]);
                    clusters_pos_right.clusters_info[l].centroid.x = (clusters_pos_right.clusters_info[l].centroid.x + clusters_vel.clusters_info[0].members[k].x)/2;
                    clusters_pos_right.clusters_info[l].centroid_vel.x = (clusters_pos_right.clusters_info[l].centroid_vel.x + clusters_vel.clusters_info[0].members_vel[k].x)/2;
                    clusters_pos_right.clusters_info[l].centroid.y = (clusters_pos_right.clusters_info[l].centroid.y + clusters_vel.clusters_info[0].members[k].y)/2;
                    clusters_pos_right.clusters_info[l].centroid_vel.y = (clusters_pos_right.clusters_info[l].centroid_vel.y + clusters_vel.clusters_info[0].members_vel[k].y)/2;
                    clusters_pos_right.clusters_info[l].centroid_dir = 0;
                    need_new_cluster_right = false;
                }
            }

            if(need_new_cluster_right == true)
            {
                vision_opticalflow::Cluster cluster_msg;    //single cluster

                cluster_num = cluster_num + 1;
                cluster_msg.id = cluster_num;
                cluster_msg.centroid = clusters_vel.clusters_info[0].members[k];
                cluster_msg.centroid_vel = clusters_vel.clusters_info[0].members_vel[k];
                cluster_msg.centroid_dir = 0;   // 0 -- moving in right direction
                cluster_msg.members_vel.push_back(clusters_vel.clusters_info[0].members_vel[k]);
                cluster_msg.members.push_back(clusters_vel.clusters_info[0].members[k]);

                clusters_pos_right.clusters_info.push_back(cluster_msg);
                need_new_cluster_right = false;
            }
            
        }

        //left group
        for(unsigned int k = 0; k < clusters_vel.clusters_info[1].members.size(); k++)
        {
            if(clusters_pos_left.clusters_info.size() == 0)
            {
                //intialize clusters_
//                 std::cout << " intialize clusters_pos " << std::endl;
                vision_opticalflow::Cluster cluster_msg;    //single cluster

                cluster_num = cluster_num + 1;
                cluster_msg.id = cluster_num;
                cluster_msg.centroid = clusters_vel.clusters_info[1].members[k];
                cluster_msg.centroid_vel = clusters_vel.clusters_info[1].members_vel[k];
                cluster_msg.centroid_dir = 1;   // 1 -- moving in left direction
                cluster_msg.members_vel.push_back(clusters_vel.clusters_info[1].members_vel[k]);
                cluster_msg.members.push_back(clusters_vel.clusters_info[1].members[k]);

                clusters_pos_left.clusters_info.push_back(cluster_msg);
//             return;
            }

            bool need_new_cluster_left = true;
            for(unsigned int l = 0; l < clusters_pos_left.clusters_info.size(); l++){
                if((abs(clusters_vel.clusters_info[1].members[k].x - clusters_pos_left.clusters_info[l].centroid.x) <= cluster_pos_bound_x_) &&
                (abs(clusters_vel.clusters_info[1].members[k].y - clusters_pos_left.clusters_info[l].centroid.y) <= cluster_pos_bound_y_)
                )
                {
                    clusters_pos_left.clusters_info[l].members.push_back(clusters_vel.clusters_info[1].members[k]);
                    clusters_pos_left.clusters_info[l].members_vel.push_back(clusters_vel.clusters_info[1].members_vel[k]);
                    clusters_pos_left.clusters_info[l].centroid.x = (clusters_pos_left.clusters_info[l].centroid.x + clusters_vel.clusters_info[1].members[k].x)/2;
                    clusters_pos_left.clusters_info[l].centroid_vel.x = (clusters_pos_left.clusters_info[l].centroid_vel.x + clusters_vel.clusters_info[1].members_vel[k].x)/2;
                    clusters_pos_left.clusters_info[l].centroid.y = (clusters_pos_left.clusters_info[l].centroid.y + clusters_vel.clusters_info[1].members[k].y)/2;
                    clusters_pos_left.clusters_info[l].centroid_vel.y = (clusters_pos_left.clusters_info[l].centroid_vel.y + clusters_vel.clusters_info[1].members_vel[k].y)/2;
                    clusters_pos_left.clusters_info[l].centroid_dir = 1;
                    need_new_cluster_left = false;
                }
            }

            if(need_new_cluster_left == true)
            {
                vision_opticalflow::Cluster cluster_msg;    //single cluster

                cluster_num = cluster_num + 1;
                cluster_msg.id = cluster_num;
                cluster_msg.centroid = clusters_vel.clusters_info[1].members[k];
                cluster_msg.centroid_vel = clusters_vel.clusters_info[1].members_vel[k];
                cluster_msg.centroid_dir = 1;   // 1 -- moving in left direction
                cluster_msg.members_vel.push_back(clusters_vel.clusters_info[1].members_vel[k]);
                cluster_msg.members.push_back(clusters_vel.clusters_info[1].members[k]);

                clusters_pos_left.clusters_info.push_back(cluster_msg);
                need_new_cluster_left = false;
            }
        }

    for (unsigned int n = 0; n < clusters_pos_right.clusters_info.size(); n++)
    {
        clusters_pos_right.clusters_info[n].centroid_dir = 0;
        clusters_pos.clusters_info.push_back(clusters_pos_right.clusters_info[n]);
    }
    for (unsigned int m = 0; m < clusters_pos_left.clusters_info.size(); m++)
    {
        clusters_pos_left.clusters_info[m].centroid_dir = 1;
        clusters_pos.clusters_info.push_back(clusters_pos_left.clusters_info[m]);
    }
//     std::cout << " cluster_num right: " << clusters_pos_right.clusters_info.size() << " cluster_num left: " << clusters_pos_left.clusters_info.size() << std::endl;
    cluster_pub_.publish(clusters_pos);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_clustering");
    FeatureClusteringNode node;
    ros::spin();
    return 0;
}
