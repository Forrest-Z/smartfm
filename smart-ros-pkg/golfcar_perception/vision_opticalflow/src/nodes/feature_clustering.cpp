#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <algorithm>

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

    float cluster_pos_bound_x_;
    float cluster_pos_bound_y_;
    /** new add **/
    float cluster_pos_bound_;
    float cluster_speed_bound_;          //speed as in magnitude of speed
    float cluster_direction_bound_;      //direction of movement in radian
//     vision_opticalflow::Clusters clusters_;      //vector of cluster
    int getNearestClusterToFeature(vision_opticalflow::Clusters clusters_pos, const vision_opticalflow::Feature::ConstPtr & input_feature, int feature_index);
    int min_cluster_id(std::vector<float> vecScoreToCluster);
    void clusterCallback(const vision_opticalflow::Feature::ConstPtr & features_msg);
};

FeatureClusteringNode::FeatureClusteringNode()
{
    feature_sub_ = nh_.subscribe("features",10, &FeatureClusteringNode::clusterCallback, this);
    cluster_pub_ = nhp_.advertise<vision_opticalflow::Clusters>("clusters", 10);

    cluster_pos_bound_x_ = 50.0;
    cluster_pos_bound_y_ = 240.0;
    /** new add **/
    cluster_pos_bound_ = 50.0;
    cluster_speed_bound_ = 10.0;   //magnitude of velocity
    cluster_direction_bound_ = 1.0;  //0.1 rad is approx. 5 degree
}
int FeatureClusteringNode::min_cluster_id(std::vector<float> vecScoreToCluster)
{
    int min_id = 0;
    for(unsigned int i = 1; i < vecScoreToCluster.size(); i++)
    {
        if(vecScoreToCluster.at(i) < vecScoreToCluster.at(min_id))
        {
            min_id = i;
        }
    }
    return min_id;
}
int FeatureClusteringNode::getNearestClusterToFeature(vision_opticalflow::Clusters clusters_pos, const vision_opticalflow::Feature::ConstPtr & input_feature, int feature_index)
{
    //return index number of the nearet cluster, if don't have any -> return -1
    int clusterIndex_out;
    float distToCluster_x, distToCluster_y,speedToCluster,dirToCluster,scoreToCluster;
    float distToCluster;
    std::vector<float> vecDistToCluster;
    std::vector<float> vecDistToCluster_x;
    std::vector<float> vecDistToCluster_y;
    std::vector<float> vecSpeedToCluster;
    std::vector<float> vecDirToCluster;
    std::vector<float> vecScoreToCluster;

    for(unsigned int l = 0; l < clusters_pos.clusters_info.size(); l++) //search in all cluster in clusters
    {
        //distance in x-direction
        distToCluster_x = input_feature->found_feature[feature_index].x - clusters_pos.clusters_info[l].centroid.x;
        vecDistToCluster_x.push_back(distToCluster_x);
        //distance in y-direction
        distToCluster_y = input_feature->found_feature[feature_index].y - clusters_pos.clusters_info[l].centroid.y;
        vecDistToCluster_y.push_back(distToCluster_y);
        //absolute distance
        distToCluster = sqrt(pow(distToCluster_x,2) + pow(distToCluster_y,2));
        vecDistToCluster.push_back(distToCluster);
        
        //speed difference to cluster
        speedToCluster = fabs(input_feature->feature_speed_mag[feature_index] - clusters_pos.clusters_info[l].centroid_speed_mag);
        vecSpeedToCluster.push_back(speedToCluster);
        //direction difference to cluster
        dirToCluster = fabs(input_feature->feature_speed_dir[feature_index]- clusters_pos.clusters_info[l].centroid_speed_dir);
        vecDirToCluster.push_back(dirToCluster);
        //calculate score
        scoreToCluster = distToCluster + speedToCluster + dirToCluster;
//         scoreToCluster = distToCluster_x + distToCluster_y + speedToCluster + dirToCluster;
//         scoreToCluster = distToCluster_x + distToCluster_y + speedToCluster;
        vecScoreToCluster.push_back(scoreToCluster);
    }
    //get cluster index with lowest score
    clusterIndex_out = min_cluster_id(vecScoreToCluster);

    //Check boundary value
    if(
//         vecDistToCluster_x.at(clusterIndex_out) <= cluster_pos_bound_x_ &&
//         vecDistToCluster_y.at(clusterIndex_out) <= cluster_pos_bound_y_ &&
        vecDistToCluster.at(clusterIndex_out) <= cluster_pos_bound_ &&
        vecSpeedToCluster.at(clusterIndex_out) <= cluster_speed_bound_ &&
        vecDirToCluster.at(clusterIndex_out) <= cluster_direction_bound_
    ){
        clusterIndex_out = clusterIndex_out;
    }else
    {
//         std::cout << "cluster_pos_bound_x_: " << vecDistToCluster_x.at(clusterIndex_out) << std::endl;
//         std::cout << "cluster_pos_bound_y_: " << vecDistToCluster_y.at(clusterIndex_out) << std::endl;
//         std::cout << "cluster_speed_bound_: " << vecSpeedToCluster.at(clusterIndex_out) << std::endl;
//         std::cout << "cluster_direction_bound_: " << vecDirToCluster.at(clusterIndex_out) << std::endl;
//         std::cout << "###--------------------------------###" << std::endl;

        clusterIndex_out = -1;
    }

    //return value
    return clusterIndex_out;
}

void FeatureClusteringNode::clusterCallback(const vision_opticalflow::Feature::ConstPtr & features_msg)
{
    int cluster_index;
    vision_opticalflow::Clusters clusters_pos,clusters_out;
    clusters_out.header = features_msg->header;
    unsigned int cluster_num = 0;

    /** new add**/
    if(features_msg->found_feature.size() != 0){
        //go through each feature
        for (unsigned int i = 0; i < features_msg->found_feature.size(); i++)
        {
            //filter out some feature, ignore too low and too high feature
            if(features_msg->feature_speed_mag[i] < 2 ||features_msg->feature_speed_mag[i] > 20 || features_msg->feature_speed_dir[i] == 0) continue;
            if(clusters_pos.clusters_info.size() == 0)
            {
                //intialize first cluster
                vision_opticalflow::Cluster cluster_msg;    //single cluster
                //set id
                cluster_num = cluster_num + 1;
                cluster_msg.id = cluster_num;
                //set first feature member
                cluster_msg.centroid.x = features_msg->found_feature[i].x;
                cluster_msg.centroid.y = features_msg->found_feature[i].y;
                cluster_msg.centroid_vel.x = features_msg->feature_vel[i].x;
                cluster_msg.centroid_vel.y = features_msg->feature_vel[i].y;
                cluster_msg.members.push_back(features_msg->found_feature[i]);
                cluster_msg.members_vel.push_back(features_msg->feature_vel[i]);
                cluster_msg.centroid_speed_mag = features_msg->feature_speed_mag[i];
                cluster_msg.centroid_speed_dir = features_msg->feature_speed_dir[i];
                cluster_msg.members_speed_mag.push_back(features_msg->feature_speed_mag[i]);   //have only one feature for now
                cluster_msg.members_speed_dir.push_back(features_msg->feature_speed_dir[i]);   //have only one feature for now
                //put this first cluster into clusters
                clusters_pos.clusters_info.push_back(cluster_msg);
                //nothing else to do next
                continue;
            }

            //start clustering each feature into cluster in clusters
            cluster_index = getNearestClusterToFeature(clusters_pos, features_msg, i);
            if(cluster_index != -1)
            {
                int clusterSize = clusters_pos.clusters_info[cluster_index].members.size();
                //Add feature to availiable clusture
                clusters_pos.clusters_info[cluster_index].centroid.x = ((clusterSize*clusters_pos.clusters_info[cluster_index].centroid.x) + features_msg->found_feature[i].x)/(clusterSize + 1);
                clusters_pos.clusters_info[cluster_index].centroid.y = ((clusterSize*clusters_pos.clusters_info[cluster_index].centroid.y) + features_msg->found_feature[i].y)/(clusterSize + 1);
                clusters_pos.clusters_info[cluster_index].centroid_vel.x = ((clusterSize*clusters_pos.clusters_info[cluster_index].centroid_vel.x) + features_msg->feature_vel[i].x)/(clusterSize + 1);
                clusters_pos.clusters_info[cluster_index].centroid_vel.y = ((clusterSize*clusters_pos.clusters_info[cluster_index].centroid_vel.y) + features_msg->feature_vel[i].y)/(clusterSize + 1);
                clusters_pos.clusters_info[cluster_index].centroid_dir = ((clusterSize*clusters_pos.clusters_info[cluster_index].centroid_dir) + features_msg->feature_speed_dir[i])/(clusterSize + 1);
                clusters_pos.clusters_info[cluster_index].members.push_back(features_msg->found_feature[i]);
                clusters_pos.clusters_info[cluster_index].members_vel.push_back(features_msg->feature_vel[i]);

                clusters_pos.clusters_info[cluster_index].centroid_speed_mag = ((clusterSize*clusters_pos.clusters_info[cluster_index].centroid_speed_mag) + features_msg->feature_speed_mag[i])/(clusterSize + 1);
                clusters_pos.clusters_info[cluster_index].centroid_speed_dir = ((clusterSize*clusters_pos.clusters_info[cluster_index].centroid_speed_dir) + features_msg->feature_speed_dir[i])/(clusterSize + 1);
                clusters_pos.clusters_info[cluster_index].members_speed_mag.push_back(features_msg->feature_speed_mag[i]);
                clusters_pos.clusters_info[cluster_index].members_speed_dir.push_back(features_msg->feature_speed_dir[i]);
            }else{
                //create and initialize new cluster
                vision_opticalflow::Cluster cluster_msg;    //single cluster
                //set id
                cluster_num = cluster_num + 1;
                cluster_msg.id = cluster_num;
                //set first feature member
                cluster_msg.centroid.x = features_msg->found_feature[i].x;
                cluster_msg.centroid.y = features_msg->found_feature[i].y;
                cluster_msg.centroid_vel.x = features_msg->feature_vel[i].x;
                cluster_msg.centroid_vel.y = features_msg->feature_vel[i].y;
                cluster_msg.members.push_back(features_msg->found_feature[i]);
                cluster_msg.members_vel.push_back(features_msg->feature_vel[i]);
                cluster_msg.centroid_speed_mag = features_msg->feature_speed_mag[i];
                cluster_msg.centroid_speed_dir = features_msg->feature_speed_dir[i];
                cluster_msg.members_speed_mag.push_back(features_msg->feature_speed_mag[i]);   //have only one feature for new cluster
                cluster_msg.members_speed_dir.push_back(features_msg->feature_speed_dir[i]);   //have only one feature for new cluster
                //put this first cluster into clusters
                clusters_pos.clusters_info.push_back(cluster_msg);
            }
            
        }
        //after cluster every feature into a cluster, check if which cluster is too small then discard
        std::cout << "clusters_pos size: " << clusters_pos.clusters_info.size() << std::endl;
        for(unsigned int i = 0; i < clusters_pos.clusters_info.size(); i++)
        {
            
            if(clusters_pos.clusters_info[i].members.size() >= 0)
            {
                clusters_out.clusters_info.push_back(clusters_pos.clusters_info[i]);
//                 std::cout << "erase: " << i << std::endl;
//                 clusters_pos.clusters_info.erase(clusters_pos.clusters_info.begin()+i);
//                 std::cout << "cluster size after: " << clusters_pos.clusters_info[i].size() << std::endl;

            }else{
//                 std::cout << "cluster size: " << clusters_pos.clusters_info[i].members.size() << std::endl;
            }
        }
        std::cout << "clusters_out size: " << clusters_out.clusters_info.size() << std::endl;
        std::cout << "###------------------------------------###" << std::endl;
    }
    cluster_pub_.publish(clusters_out);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_clustering");
    FeatureClusteringNode node;
    ros::spin();
    return 0;
}
