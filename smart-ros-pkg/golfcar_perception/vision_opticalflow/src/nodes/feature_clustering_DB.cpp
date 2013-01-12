#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <algorithm>

#include <vision_opticalflow/Feature.h>
#include <vision_opticalflow/Cluster.h>
#include <vision_opticalflow/Clusters.h>

class DBClusteringNode
{
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
    
    void expandCluster(geometry_msgs::Point p, std::vector<int> &neighbor_points);
    bool checkMembership(geometry_msgs::Point p);
    void regionQuery(vision_opticalflow::Feature input_feature, geometry_msgs::Point p, std::vector<int> &pointInRegion);
    float distance(geometry_msgs::Point p1,geometry_msgs::Point p2);
    void clusterCallback(const vision_opticalflow::Feature::ConstPtr & features_msg);
};

DBClusteringNode::DBClusteringNode()
{
    feature_sub_ = nh_.subscribe("features",10, &DBClusteringNode::clusterCallback, this);
    cluster_pub_ = nhp_.advertise<vision_opticalflow::Clusters>("clusters", 10);

    //set up parameters
    esp = 4;
    MinPts = 2;
    cluster_inx_ = -1;
}

void DBClusteringNode::expandCluster(geometry_msgs::Point p, std::vector<int> &neighbor_points)
{
    clustersDB.clusters_info[cluster_inx_].members.push_back(p);
    
    std::vector<int> NeighborPts_;
    geometry_msgs::Point npoint_tmp;

    int i = 0;
    while(true)
    {
        if(i <= neighbor_points.size())
        {
            i = i + 1;
        }else{
            break;
        }

        npoint_tmp = featureDB.found_feature[neighbor_points[i]];
        if(featureDB.visited[neighbor_points[i]] != true)
        {
            featureDB.visited[neighbor_points[i]] = true;
            regionQuery(featureDB, npoint_tmp, NeighborPts_);
            if(NeighborPts_.size() >= MinPts)
            {
                for(int j = 0; j <= NeighborPts_.size(); j++)
                {
                    neighbor_points.push_back(NeighborPts_[j]);
                }
            }
        }
        if(checkMembership(npoint_tmp) != true)
        {
            clustersDB.clusters_info[cluster_inx_].members.push_back(npoint_tmp);
        }else{
            //do not thing
        }

    }

}

bool DBClusteringNode::checkMembership(geometry_msgs::Point p)
{
    bool ismember = false;
    for(int i = 0; i <= clustersDB.clusters_info.size(); i++)
    {
        for(int j = 0; j <= clustersDB.clusters_info[i].members.size(); j++)
        {
            if((p.x == clustersDB.clusters_info[i].members[j].x) && (p.x == clustersDB.clusters_info[i].members[j].x))
            {
                ismember = true;
            }
        }
    }
    return ismember;
}

void DBClusteringNode::regionQuery(vision_opticalflow::Feature featureDB, geometry_msgs::Point p, std::vector<int> &pointInRegion)
{
    pointInRegion.clear();
    geometry_msgs::Point p_tmp;
    for(int i = 0; i <= featureDB.found_feature.size(); i++)
    {
        p_tmp = featureDB.found_feature[i];
        if((distance(p,p_tmp) < esp) && (p.x != p_tmp.x) && (p.y != p_tmp.y))
        {
            //pointInRegion.push_back(p_tmp);
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

//     def DBSCAN(self):
//         for i in range(len(self.DB)):
//             p_tmp = self.DB[i]
//             if (not p_tmp.visited):
//                 #for each unvisited point P in dataset
//                 p_tmp.visited = True
//                 NeighborPts = self.regionQuery(p_tmp)
//                 if(len(NeighborPts) < self.MinPts):
//                     #that point is a noise
//                     p_tmp.isnoise = True
//                     print p_tmp.show(), 'is a noise'
//                 else:
//                     self.cluster.append([])
//                     self.cluster_inx = self.cluster_inx + 1
//                     self.expandCluster(p_tmp, NeighborPts)

void DBClusteringNode::clusterCallback(const vision_opticalflow::Feature::ConstPtr &features_msg)
{
    featureDB = *features_msg;

    geometry_msgs::Point p_tmp;
    vision_opticalflow::Cluster dummy_cluster;
    for(int i = 0; i <= featureDB.found_feature.size(); i++)
    {
        p_tmp = featureDB.found_feature[i];
        if(featureDB.visited[i] != true)
        {
            std::vector<int> NeighborPts_;
            
            featureDB.visited[i] = true;
            regionQuery(featureDB, p_tmp, NeighborPts_);
            if(NeighborPts_.size() < MinPts)
            {
                featureDB.isnoise[i] = true;
            }else{
                cluster_inx_ = cluster_inx_ + 1;
                dummy_cluster.id = cluster_inx_;
                clustersDB.clusters_info.push_back(dummy_cluster);
                expandCluster(p_tmp, NeighborPts_);
            }
        }
    }
    cluster_pub_.publish(clustersDB);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_clustering_DB");
    DBClusteringNode node;
    ros::spin();
    return 0;
}