#include <ros/ros.h>

#include <vision_motion_detection/Tracks.h>
#include <vision_opticalflow/Cluster.h>
#include <vision_opticalflow/Clusters.h>

class ClustersToTracksNode
{
public:
    ClustersToTracksNode();

private:
    ros::NodeHandle nhp_, nh_;
    ros::Subscriber clusters_sub_;
    ros::Publisher tracker_pub_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "clusters_to_tracks");
    ClustersToTracksNode node;
    ros::spin();
    return 0;
}