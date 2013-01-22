#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <math.h>
#include <vector>
#include <algorithm>

#include <vision_opticalflow/Feature.h>
#include <vision_opticalflow/Cluster.h>
#include <vision_opticalflow/Clusters.h>

class DBClusteringNode
{
//For reference :: Density-Based Spatial Clustering of Application with Noise -> http://en.wikipedia.org/wiki/DBSCAN
public:
    DBClusteringNode();

private:
    ros::NodeHandle nhp_, nh_;
    ros::Subscriber feature_sub_;
    ros::Publisher cluster_pub_;

    int esp;         //neighborhood distance for search
    int MinPts;      //minimum number of points required to form a cluster
    int cluster_inx_;
    vision_opticalflow::Clusters clustersDB;
    vision_opticalflow::Feature featureDB;

    float dir_diff(float dir1, float dir2);
    int minimum_id_vec(std::vector<float> &input_vector);
    void finalizeCluster_stage1(void);
    void finalizeCluster_stage2(void);
    void updateCluster(int p_index);
    void expandCluster(geometry_msgs::Point p, int p_index, std::vector<int> &neighbor_points);
    bool checkMembership(geometry_msgs::Point p);
    void regionQuery(vision_opticalflow::Feature input_feature, geometry_msgs::Point p, float npoint_dir, std::vector<int> &pointInRegion);
    float distance(geometry_msgs::Point p1,geometry_msgs::Point p2);
    void clusterCallback(const vision_opticalflow::Feature::ConstPtr & features_msg);
};

DBClusteringNode::DBClusteringNode()
{
    feature_sub_ = nh_.subscribe("features",10, &DBClusteringNode::clusterCallback, this);
    cluster_pub_ = nhp_.advertise<vision_opticalflow::Clusters>("clusters", 10);

    //set up parameters
    esp = 10;
    MinPts = 10;
    cluster_inx_ = 0;
}

float DBClusteringNode::dir_diff(float dir1, float dir2)
{
    float delta_dir;
    delta_dir = std::fabs(dir1 - dir2);
    return delta_dir;
}

int DBClusteringNode::minimum_id_vec(std::vector<float> &input_vector)
{
    int min_id = 0;
    for(int i = 0; i < input_vector.size(); i++)
    {
        if(input_vector[i] < input_vector[min_id])
        {
            min_id = i;
        }
    }
    return min_id;
}

void DBClusteringNode::finalizeCluster_stage1(void)
{
    std::vector<float> sort_dir;
    //Stage 1 part 1
    for(int i = 0; i < clustersDB.clusters_info.size(); i++)
    {
        vision_opticalflow::Cluster sorted_Cluster;
        sorted_Cluster.id = i;
        int min_id;
        for(int j = 0; j < clustersDB.clusters_info[i].members.size(); j++)
        {
            //why I cannot access clustersDB.clusters_info[i].members_speed_dir vector directly??
            sort_dir.clear();
            for(int k = 0; k < clustersDB.clusters_info[i].members.size(); k++) sort_dir.push_back(clustersDB.clusters_info[i].members_speed_dir[k]);
            min_id = minimum_id_vec(sort_dir);
            //sort it
            sorted_Cluster.members.push_back(clustersDB.clusters_info[i].members[min_id]);
            sorted_Cluster.members_vel.push_back(clustersDB.clusters_info[i].members_vel[min_id]);
            sorted_Cluster.members_speed_mag.push_back(clustersDB.clusters_info[i].members_speed_mag[min_id]);
            sorted_Cluster.members_speed_dir.push_back(clustersDB.clusters_info[i].members_speed_dir[min_id]);
            //erase entry
            clustersDB.clusters_info[i].members.erase(clustersDB.clusters_info[i].members.begin()+min_id);
            clustersDB.clusters_info[i].members_vel.erase(clustersDB.clusters_info[i].members_vel.begin()+min_id);
            clustersDB.clusters_info[i].members_speed_mag.erase(clustersDB.clusters_info[i].members_speed_mag.begin()+min_id);
            clustersDB.clusters_info[i].members_speed_dir.erase(clustersDB.clusters_info[i].members_speed_dir.begin()+min_id);
            
        }

        clustersDB.clusters_info[i].to_be_erase = true;
        clustersDB.clusters_info.push_back(sorted_Cluster);
    }
    
    //Stage 1 part 2
    for(int i = 0; i < clustersDB.clusters_info.size(); i++)
    {
        vision_opticalflow::Cluster tmp_Cluster;
        int inx_tmp = 0;
        float diff = 0;
        //for the first feature
//         clustersDB.clusters_info.push_back(tmp_Cluster);
        tmp_Cluster.members.push_back(clustersDB.clusters_info[i].members[0]);
        tmp_Cluster.members_vel.push_back(clustersDB.clusters_info[i].members_vel[0]);
        tmp_Cluster.members_speed_mag.push_back(clustersDB.clusters_info[i].members_speed_mag[0]);
        tmp_Cluster.members_speed_dir.push_back(clustersDB.clusters_info[i].members_speed_dir[0]);
        for(int j = 0; j < clustersDB.clusters_info[i].members_speed_dir.size()-1; j++)
        {
            diff = std::fabs(clustersDB.clusters_info[i].members_speed_dir[inx_tmp] - clustersDB.clusters_info[i].members_speed_dir[inx_tmp+1]);
            if(diff >= 3.414)
            {
                std::cout << "diff: " << diff << std::endl;
                clustersDB.clusters_info.push_back(tmp_Cluster);
                //and begin new cluster
                tmp_Cluster.members.clear();
                tmp_Cluster.members_vel.clear();
                tmp_Cluster.members_speed_mag.clear();
                tmp_Cluster.members_speed_dir.clear();
                
                tmp_Cluster.members.push_back(clustersDB.clusters_info[i].members[inx_tmp+1]);
                tmp_Cluster.members_vel.push_back(clustersDB.clusters_info[i].members_vel[inx_tmp+1]);
                tmp_Cluster.members_speed_mag.push_back(clustersDB.clusters_info[i].members_speed_mag[inx_tmp+1]);
                tmp_Cluster.members_speed_dir.push_back(clustersDB.clusters_info[i].members_speed_dir[inx_tmp+1]);
            }else{
                //at the feature to cluster
                tmp_Cluster.members.push_back(clustersDB.clusters_info[i].members[inx_tmp+1]);
                tmp_Cluster.members_vel.push_back(clustersDB.clusters_info[i].members_vel[inx_tmp+1]);
                tmp_Cluster.members_speed_mag.push_back(clustersDB.clusters_info[i].members_speed_mag[inx_tmp+1]);
                tmp_Cluster.members_speed_dir.push_back(clustersDB.clusters_info[i].members_speed_dir[inx_tmp+1]);
            }             
            inx_tmp = inx_tmp + 1;
        }
        clustersDB.clusters_info.push_back(tmp_Cluster);
        //kill old cluster
        clustersDB.clusters_info[i].to_be_erase = true;
    }
  
}

void DBClusteringNode::finalizeCluster_stage2(void)
{
    //Stage 2 of 2 for finalizeCluster
    for(int i = 0; i < clustersDB.clusters_info.size(); i++)
    {
        //temp variable
        float sum_centroid_x = 0;
        float sum_centroid_y = 0;
        float sum_centroid_vel_x = 0;
        float sum_centroid_vel_y = 0;
        float sum_centroid_dir = 0;
        
        geometry_msgs::Point centroid;
        geometry_msgs::Point centroid_vel;
        int centroid_dir;
        for(int j = 0; j < clustersDB.clusters_info[i].members.size(); j++)
        {
            sum_centroid_x = sum_centroid_x + clustersDB.clusters_info[i].members[j].x;
            sum_centroid_y = sum_centroid_y + clustersDB.clusters_info[i].members[j].y;

            sum_centroid_vel_x = sum_centroid_vel_x + clustersDB.clusters_info[i].members_vel[j].x;
            sum_centroid_vel_y = sum_centroid_vel_y + clustersDB.clusters_info[i].members_vel[j].y;
        }
        //id
        clustersDB.clusters_info[i].id = i;
        //centroid
        centroid.x = sum_centroid_x/clustersDB.clusters_info[i].members.size();
        centroid.y = sum_centroid_y/clustersDB.clusters_info[i].members.size();
        clustersDB.clusters_info[i].centroid = centroid;
        //centroid_vel
        centroid_vel.x = sum_centroid_vel_x/clustersDB.clusters_info[i].members.size();
        centroid_vel.y = sum_centroid_vel_y/clustersDB.clusters_info[i].members.size();
        clustersDB.clusters_info[i].centroid_vel = centroid_vel;
        //centroid_dir --> do we using this??
        clustersDB.clusters_info[i].centroid_dir = atan2(centroid_vel.y,centroid_vel.x);
        //centroid_speed_mag
        clustersDB.clusters_info[i].centroid_speed_mag = sqrt(pow(centroid_vel.x,2) + pow(centroid_vel.y,2));
        //centroid_speed_dir
        if((centroid_vel.x != 0) && (centroid_vel.y != 0))
        {
            clustersDB.clusters_info[i].centroid_speed_dir = atan2(centroid_vel.y,centroid_vel.x);
        }else{
            std::cout << "centroid_vel.x or centroid_vel.y = 0" << std::endl;
            clustersDB.clusters_info[i].centroid_speed_dir = 0;
        }
    }

    if(clustersDB.clusters_info.size() == 0)
    {
        //in this case, there are some features, but unable to form a cluster
        vision_opticalflow::Cluster dummy_cluster;
        geometry_msgs::Point dummy_p;
        dummy_p.x = 0.0;
        dummy_p.y = 0.0;

        dummy_cluster.id = 0;
        dummy_cluster.centroid = dummy_p;
        dummy_cluster.centroid_vel = dummy_p;
        dummy_cluster.centroid_dir = 0;
        dummy_cluster.members.push_back(dummy_p);
        dummy_cluster.members_vel.push_back(dummy_p);
        dummy_cluster.centroid_speed_mag = 0.0;
        dummy_cluster.centroid_speed_dir = 0.0;
        dummy_cluster.members_speed_mag.push_back(0.0);
        dummy_cluster.members_speed_dir.push_back(0.0);

        clustersDB.clusters_info.push_back(dummy_cluster);
    }
}

void DBClusteringNode::updateCluster(int p_index)
{
    clustersDB.clusters_info[cluster_inx_].members.push_back(featureDB.found_feature[p_index]);
    clustersDB.clusters_info[cluster_inx_].members_vel.push_back(featureDB.feature_vel[p_index]);
    clustersDB.clusters_info[cluster_inx_].members_speed_mag.push_back(featureDB.feature_speed_mag[p_index]);
    clustersDB.clusters_info[cluster_inx_].members_speed_dir.push_back(featureDB.feature_speed_dir[p_index]);
}

void DBClusteringNode::expandCluster(geometry_msgs::Point p, int p_index, std::vector<int> &neighbor_points)
{
    updateCluster(p_index);
    
    std::vector<int> NeighborPts_;
    geometry_msgs::Point npoint_tmp;
    float npoint_dir;
    int i = -1;
    while(true)
    {
        if((i+1) < neighbor_points.size())
        {
            i = i + 1;
        }else{
            break;
        }

        npoint_tmp = featureDB.found_feature[neighbor_points[i]];
        npoint_dir = featureDB.feature_speed_dir[neighbor_points[i]];
        if(featureDB.visited[neighbor_points[i]] != true)
        {
            featureDB.visited[neighbor_points[i]] = true;
            regionQuery(featureDB, npoint_tmp, npoint_dir, NeighborPts_);
            if(NeighborPts_.size() >= MinPts)
            {
                for(int j = 0; j < NeighborPts_.size(); j++)
                {
                    neighbor_points.push_back(NeighborPts_[j]);
                }
            }
        }
        
        if(checkMembership(npoint_tmp) != true)
        {
            updateCluster(neighbor_points[i]);
        }else{
            //do not thing
        }
    }
}

bool DBClusteringNode::checkMembership(geometry_msgs::Point p)
{
    bool ismember = false;
    for(int i = 0; i < clustersDB.clusters_info.size(); i++)
    {
        for(int j = 0; j < clustersDB.clusters_info[i].members.size(); j++)
        {
            if((p.x == clustersDB.clusters_info[i].members[j].x) && (p.y == clustersDB.clusters_info[i].members[j].y))
            {
                ismember = true;
            }
        }
    }
    return ismember;
}

void DBClusteringNode::regionQuery(vision_opticalflow::Feature featureDB, geometry_msgs::Point p, float npoint_dir, std::vector<int> &pointInRegion)
{
    pointInRegion.clear();
    geometry_msgs::Point p_tmp;
    float p_tmp_dir;
    for(int i = 0; i < featureDB.found_feature.size(); i++)
    {
        p_tmp = featureDB.found_feature[i];
        p_tmp_dir = featureDB.feature_speed_dir[i];
        if((distance(p,p_tmp) < esp) && (p.x != p_tmp.x) && (p.y != p_tmp.y) && dir_diff(npoint_dir,p_tmp_dir) < 3.414)
        {
            pointInRegion.push_back(i);
        }
    }
}

float DBClusteringNode::distance(geometry_msgs::Point p1,geometry_msgs::Point p2)
{
    float dx,dy;
    //return distance between two point
    dx = (p1.x - p2.x);
    dy = (p1.y - p2.y);
    
    return sqrt(pow(dx,2) + pow(dy,2));
}


void DBClusteringNode::clusterCallback(const vision_opticalflow::Feature::ConstPtr &features_msg)
{
    //Features
    featureDB = *features_msg;
    //Clusters
    clustersDB.clusters_info.clear();
    clustersDB.header = featureDB.header;
    //index
    cluster_inx_ = -1;
    
    geometry_msgs::Point p_tmp;
    float p_tmp_dir;
    vision_opticalflow::Cluster dummy_cluster;
    if(featureDB.found_feature.size() > 0){
        std::cout << "Feature receives: " << featureDB.found_feature.size() << std::endl;
        //initialize visited and isnoise
        for(int i = 0; i < featureDB.found_feature.size(); i++)
        {
            featureDB.visited.push_back(false);
            featureDB.isnoise.push_back(false);
        }
        //end initialize
        
        for(int i = 0; i < featureDB.found_feature.size(); i++)
        {
            p_tmp = featureDB.found_feature[i];
            p_tmp_dir = featureDB.feature_speed_dir[i];
            if(featureDB.visited[i] != true)
            {
                std::vector<int> NeighborPts_;
                featureDB.visited[i] = true;
                regionQuery(featureDB, p_tmp, p_tmp_dir, NeighborPts_);
                if(NeighborPts_.size() < MinPts)
                {
                    featureDB.isnoise[i] = true;
                }else{
                    cluster_inx_ = cluster_inx_ + 1;
                    dummy_cluster.id = cluster_inx_;
                    clustersDB.clusters_info.push_back(dummy_cluster);
                    expandCluster(p_tmp, i, NeighborPts_);
                }
            }
        }
//         finalizeCluster_stage1();
        finalizeCluster_stage2();
    }else{
        //no features input
        //what can you do? --> put zero cluster into clustersDB
        geometry_msgs::Point dummy_p;
        dummy_p.x = 0.0;
        dummy_p.y = 0.0;

        dummy_cluster.id = 0;
        dummy_cluster.centroid = dummy_p;
        dummy_cluster.centroid_vel = dummy_p;
        dummy_cluster.centroid_dir = 0;
        dummy_cluster.members.push_back(dummy_p);
        dummy_cluster.members_vel.push_back(dummy_p);
        dummy_cluster.centroid_speed_mag = 0.0;
        dummy_cluster.centroid_speed_dir = 0.0;
        dummy_cluster.members_speed_mag.push_back(0.0);
        dummy_cluster.members_speed_dir.push_back(0.0);
        
        clustersDB.clusters_info.push_back(dummy_cluster);
    }
    std::cout << "number of cluster: " << clustersDB.clusters_info.size() << std::endl;
    std::cout << "###--------------------------###"<< std::endl;
    cluster_pub_.publish(clustersDB);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_clustering_DB");
    DBClusteringNode node;
    ros::spin();
    return 0;
}