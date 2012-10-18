#include <ros/ros.h>
#include <math.h>
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

    std::vector<vision_opticalflow::Clusters> clusters_prev;
    vision_opticalflow::Clusters clusters_curr;

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
//     float minFloat(std::vector<float> vector_float);
};

ClustersTrackingNode::ClustersTrackingNode()
{
    feature_sub_ = nh_.subscribe("clusters", 10, &ClustersTrackingNode::trackerCallback, this);
    tracker_pub_  = nhp_.advertise<vision_opticalflow::Clusters>("tracker", 10);

    begin = ros::Time::now();
    logFile.open ("/home/livingmachine/Documents/DataRecord/logFile.txt");
}

void ClustersTrackingNode::trackerCallback(const vision_opticalflow::Clusters::ConstPtr & clusters_msg)
{
//     std::cout << " Got clusters message " << std::endl;
    vision_opticalflow::Clusters clusters_temp;
    if(clusters_msg->clusters_info.empty()) return;
    
    for(unsigned int i = 0; i < clusters_msg->clusters_info.size(); i++)
    {
        vision_opticalflow::Cluster cluster_temp;

        cluster_temp.id = clusters_msg->clusters_info[i].id;
        cluster_temp.centroid = clusters_msg->clusters_info[i].centroid;
        cluster_temp.centroid_vel = clusters_msg->clusters_info[i].centroid_vel;
        clusters_temp.header = clusters_msg->header;
        clusters_temp.clusters_info.push_back(cluster_temp);
//         std::cout << "centroid : " << cluster_temp.centroid << std::endl;
    }
    clusters_curr = clusters_temp;
    
    if(clusters_prev.empty())
    {   //for first call
        std::cout << " clusters_prev is empty " << std::endl;
        clusters_prev.push_back(clusters_curr);
        return;
    }

    for(unsigned int j = 0; j < clusters_curr.clusters_info.size(); j++)
    {
        //For each current cluster
        unsigned int max_id;
        max_id = clusters_curr.clusters_info.size() + 1;
        bool match_found = false;
        int clusters_index = clusters_prev.size()-1;
        
        while((match_found != true) && (clusters_index != 0)){
            std::vector<float> centroid_diff;
            
            for(unsigned int k = 0; k < clusters_prev[clusters_index].clusters_info.size(); k++)
            {
                float distance_temp;
                distance_temp = centroid_distance(clusters_curr.clusters_info[j].centroid,clusters_prev[clusters_index].clusters_info[k].centroid);
    //             std::cout << distance_temp << std::endl;
                centroid_diff.push_back(distance_temp);
            }
            minFloat_out nearest_centroid = minFloat(centroid_diff);

            if(nearest_centroid.min_value < 30)
            {
                clusters_curr.clusters_info[j].id = clusters_prev[clusters_index].clusters_info[nearest_centroid.id].id;
                match_found = true;
            }else{
                clusters_index = clusters_index - 1;
//                 std::cout << "clusters_index: " << clusters_index << std::endl;
            }
        }
        
        if(match_found == false)
        {
            clusters_curr.clusters_info[j].id = max_id;
            max_id = max_id + 1;
        }
//         //Write data to file
//         logFile << ros::Time::now()-begin << " " << clusters_msg->header.seq << " " << j << " " << clusters_curr.clusters_info[j].centroid.x << " " << clusters_curr.clusters_info[j].centroid.y
//         << " " << clusters_curr.clusters_info[j].centroid_vel.x << " " << clusters_curr.clusters_info[j].centroid_vel.y << std::endl;
//         std::cout << clusters_curr.clusters_info[j].id << " :: " << clusters_prev.back().clusters_info[nearest_centroid.id].id << " , " << nearest_centroid.id << std::endl;
    }

    tracker_pub_.publish(clusters_curr);
    //update
    if(clusters_prev.size() >= 8) clusters_prev.erase(clusters_prev.begin());
    clusters_prev.push_back(clusters_curr);
//     std::cout << "clusters_prev.size(): " << clusters_prev.size() << std::endl;
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