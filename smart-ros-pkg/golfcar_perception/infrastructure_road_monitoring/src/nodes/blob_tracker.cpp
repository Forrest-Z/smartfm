#include <ros/ros.h>

#include <infrastructure_road_monitoring/Tracks.h>
#include <infrastructure_road_monitoring/Blobs.h>

#include "BlobTracker.h"
#include "data_msg_conv.h"

class BlobTrackerNode : public BlobTracker
{
public:
    BlobTrackerNode()
    {
        blobs_sub_ = nh_.subscribe("blobs", 10, &BlobTrackerNode::blobsCallback, this);
        tmnnt.match_threshold = &amt;
        matcher = &tmnnt;
        tracks_pub_ = nh_.advertise<infrastructure_road_monitoring::Tracks>("tracks", 10);
        unobserved_threshold_remove = 10;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher tracks_pub_;
    ros::Subscriber blobs_sub_;

    AdaptiveMatcherThreshold amt;
    TrackMatcherNNT tmnnt;

    void blobsCallback(const infrastructure_road_monitoring::Blobs & blobs_msg)
    {
        std::vector<Blob> blobs;
        for( unsigned i=0; i<blobs_msg.blobs.size(); i++ )
            blobs.push_back( blobMsgToData(blobs_msg.blobs[i], blobs_msg.header.stamp.toSec()) );

        this->update(blobs);

        infrastructure_road_monitoring::Tracks tracks_msg;
        tracks_msg.header = blobs_msg.header;
        for( Tracks::const_iterator it=tracks.begin(); it!=tracks.end(); ++it )
            tracks_msg.tracks.push_back( trackDataToMsg(*it) );

        tracks_pub_.publish(tracks_msg);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_tracker");
    BlobTrackerNode node;
    ros::spin();
    return 0;
}
