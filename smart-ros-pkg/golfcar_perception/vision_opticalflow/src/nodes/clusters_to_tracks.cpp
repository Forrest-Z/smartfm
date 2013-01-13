#include <ros/ros.h>

#include "vision_motion_detection/Tracks.h"
#include "vision_motion_detection/Track.h"
#include "vision_motion_detection/Blob.h"

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

    void trackerCallback(const vision_opticalflow::Clusters::ConstPtr & tracker_msg);
};

ClustersToTracksNode::ClustersToTracksNode()
{
    clusters_sub_ = nh_.subscribe("tracker", 10, &ClustersToTracksNode::trackerCallback, this);
    tracker_pub_  = nhp_.advertise<vision_motion_detection::Tracks>("tracks", 10);
}

void ClustersToTracksNode::trackerCallback(const vision_opticalflow::Clusters::ConstPtr & tracker_msg)
{
//     std::cout << "ClustersToTracksNode is recieving... " << std::endl;
    vision_motion_detection::Tracks tracks_msg;
    tracks_msg.header = tracker_msg->header;

    for(unsigned int i = 0; i < tracker_msg->clusters_info.size(); i++)
    {
        vision_motion_detection::Track track_tmp;
        vision_motion_detection::Blob blob_tmp;
        
        track_tmp.id = tracker_msg->clusters_info[i].id;
        track_tmp.xvel = tracker_msg->clusters_info[i].centroid_vel.x;
        track_tmp.yvel = tracker_msg->clusters_info[i].centroid_vel.y;

        blob_tmp.centroid = tracker_msg->clusters_info[i].centroid;
        track_tmp.blob = blob_tmp;

        tracks_msg.tracks.push_back(track_tmp);
    }
    
    tracker_pub_.publish(tracks_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clusters_to_tracks");
    ClustersToTracksNode node;
    ros::spin();
    return 0;
}