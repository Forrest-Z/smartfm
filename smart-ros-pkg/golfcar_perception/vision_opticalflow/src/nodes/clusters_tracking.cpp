#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>

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

    std::vector<vision_opticalflow::Clusters> clusters_prev_;
    vision_opticalflow::Clusters clusters_curr_;

    //Write to file
    std::ofstream logFile;
    ros::Time begin;
    
    struct minFloat_out{
        float min_value;
        unsigned int id;
    };
    void trackerCallback(const vision_opticalflow::Clusters::ConstPtr & clusters_msg);
    float centroid_distance(geometry_msgs::Point cen1,geometry_msgs::Point cen2);
    minFloat_out minFloat(std::vector<float> vector_float);
    
    /** new add **/
    int min_cluster_id(std::vector<float> vecScoreToCluster);
    int getNearestClusterToCluster(std::vector<vision_opticalflow::Clusters> clusters_prev_, vision_opticalflow::Cluster clusters_curr_, int hist_index);
    float cluster_dist_bound_x_;
    float cluster_dist_bound_y_;
    float cluster_pos_bound_;
    float cluster_speed_bound_;          //speed as in magnitude of speed
    float cluster_direction_bound_;      //direction of movement in radian
};

ClustersTrackingNode::ClustersTrackingNode()
{
    feature_sub_ = nh_.subscribe("clusters", 10, &ClustersTrackingNode::trackerCallback, this);
    tracker_pub_  = nhp_.advertise<vision_opticalflow::Clusters>("tracker", 10);

    begin = ros::Time::now();
//     logFile.open ("/home/livingmachine/Documents/DataRecord/logFile.txt");

    /** new add **/
    cluster_pos_bound_ = 10.0;
    cluster_dist_bound_x_ = 25.0;
    cluster_dist_bound_y_ = 100.0;
    cluster_speed_bound_ = 10.0;   //magnitude of velocity
    cluster_direction_bound_ = 1.0;  //0.1 rad is approx. 5 degree
}

int ClustersTrackingNode::min_cluster_id(std::vector<float> vecScoreToCluster)
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

int ClustersTrackingNode::getNearestClusterToCluster(std::vector<vision_opticalflow::Clusters> clusters_prev_, vision_opticalflow::Cluster cluster_curr_, int hist_index)
{
    int clusterIndex_out;
    float distToCluster_x, distToCluster_y,speedToCluster,dirToCluster,scoreToCluster;
    float distToCluster;
    std::vector<float> vecDistToCluster;
    std::vector<float> vecDistToCluster_x;
    std::vector<float> vecDistToCluster_y;
    std::vector<float> vecSpeedToCluster;
    std::vector<float> vecDirToCluster;
    std::vector<float> vecScoreToCluster;

    for(unsigned int k = 0; k < clusters_prev_[hist_index].clusters_info.size(); k++)
    {
        //distance in x-direction
        distToCluster_x = fabs(cluster_curr_.centroid.x - clusters_prev_[hist_index].clusters_info[k].centroid.x);
        vecDistToCluster_x.push_back(distToCluster_x);
        //distance in y-direction
        distToCluster_y = fabs(cluster_curr_.centroid.y - clusters_prev_[hist_index].clusters_info[k].centroid.y);
        vecDistToCluster_y.push_back(distToCluster_y);
        //absolute distance
        distToCluster = sqrt(pow(distToCluster_x,2) + pow(distToCluster_y,2));
        vecDistToCluster.push_back(distToCluster);
        
        //speed difference to cluster
        speedToCluster = fabs(cluster_curr_.centroid_speed_mag - clusters_prev_[hist_index].clusters_info[k].centroid_speed_mag);
        vecSpeedToCluster.push_back(speedToCluster);
        //direction difference to cluster
        dirToCluster = fabs(cluster_curr_.centroid_speed_dir - clusters_prev_[hist_index].clusters_info[k].centroid_speed_dir);
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
//         vecDistToCluster_x.at(clusterIndex_out) <= cluster_dist_bound_x_ &&
//         vecDistToCluster_y.at(clusterIndex_out) <= cluster_dist_bound_y_ &&
        vecDistToCluster.at(clusterIndex_out) <= cluster_pos_bound_ &&
        vecSpeedToCluster.at(clusterIndex_out) <= cluster_speed_bound_ &&
        vecDirToCluster.at(clusterIndex_out) <= cluster_direction_bound_
    ){
        clusterIndex_out = clusterIndex_out;
    }else{
        clusterIndex_out = -1;
    }

    //return value
    return clusterIndex_out;
}

void ClustersTrackingNode::trackerCallback(const vision_opticalflow::Clusters::ConstPtr & clusters_msg)
{
    if(clusters_msg->clusters_info.empty()) return;
    clusters_curr_ = *clusters_msg;
    
    if(clusters_prev_.empty())
    {   //for first call
        std::cout << "clusters_prev_ is empty " << std::endl;
        clusters_prev_.push_back(clusters_curr_);
        return;
    }

    for(unsigned int j = 0; j < clusters_curr_.clusters_info.size(); j++)
    {
        //For each current cluster
        unsigned int max_id;
        int cluster_index;
        max_id = clusters_curr_.clusters_info.size() + 1;
        bool match_found = false;
        int hist_index = clusters_prev_.size()-1;
        
        cluster_index = getNearestClusterToCluster(clusters_prev_, clusters_curr_.clusters_info[j], hist_index);
        if(cluster_index != -1)
        {
            clusters_curr_.clusters_info[j].id = clusters_prev_[hist_index].clusters_info[cluster_index].id;
        }else if(cluster_index == -1 && hist_index != 0){
            hist_index = hist_index - 1;
        }else{
            clusters_curr_.clusters_info[j].id = max_id;
            max_id = max_id + 1;
        }
//         //Write data to file
//         logFile << ros::Time::now()-begin << " " << clusters_msg->header.seq << " " << j << " " << clusters_curr_.clusters_info[j].centroid.x << " " << clusters_curr_.clusters_info[j].centroid.y
//         << " " << clusters_curr_.clusters_info[j].centroid_vel.x << " " << clusters_curr_.clusters_info[j].centroid_vel.y << std::endl;
//         std::cout << clusters_curr_.clusters_info[j].id << " :: " << clusters_prev_.back().clusters_info[nearest_centroid.id].id << " , " << nearest_centroid.id << std::endl;
    }

    tracker_pub_.publish(clusters_curr_);
    //update
    if(clusters_prev_.size() >= 2) clusters_prev_.erase(clusters_prev_.begin());
    clusters_prev_.push_back(clusters_curr_);
//     std::cout << "clusters_prev_.size(): " << clusters_prev_.size() << std::endl;
}

float ClustersTrackingNode::centroid_distance(geometry_msgs::Point cen1,geometry_msgs::Point cen2)
{
    float distance;
    distance = sqrt(pow((cen1.x - cen2.x),2)+pow((cen1.y - cen2.y),2));

    return distance;
}

ClustersTrackingNode::minFloat_out ClustersTrackingNode::minFloat(std::vector<float> vector_float)
{
    minFloat_out output_temp;
    output_temp.min_value = vector_float[0];
    output_temp.id = 0;
    for(unsigned int i = 0; i < vector_float.size(); i++)
    {
        if(vector_float[i] < output_temp.min_value)
        {
            output_temp.min_value = vector_float[i];
            output_temp.id = i;
        }
    }
    return output_temp;
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clusters_tracking");
    ClustersTrackingNode node;
    ros::spin();
    return 0;
}