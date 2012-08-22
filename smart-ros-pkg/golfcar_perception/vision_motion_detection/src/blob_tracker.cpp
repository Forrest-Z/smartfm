#include <ros/ros.h>

#include <vision_motion_detection/Tracks.h>
#include <vision_motion_detection/Blobs.h>
#include <vision_motion_detection/TrackMatcher.h>
#include <vision_motion_detection/data_types.h>
#include <vision_motion_detection/Tracker.h>

class BlobTrackerNode
{
public:
    BlobTrackerNode() : matcher_(amt_), tracker_(matcher_)
    {
        blobs_sub_ = nh_.subscribe("blobs", 10, &BlobTrackerNode::blobsCallback, this);
        tracks_pub_ = nh_.advertise<vision_motion_detection::Tracks>("tracks", 10);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher tracks_pub_;
    ros::Subscriber blobs_sub_;

    AdaptiveMatcherThreshold amt_;
    TrackMatcherNNT matcher_;
    Tracker tracker_;

    void blobsCallback(const vision_motion_detection::Blobs & blobs_msg);
};


void BlobTrackerNode::blobsCallback(const vision_motion_detection::Blobs & blobs_msg)
{
    std::vector<Blob> blobs;
    for( unsigned i=0; i<blobs_msg.blobs.size(); i++ )
        blobs.push_back( blobMsgToData(blobs_msg.blobs[i], blobs_msg.header.stamp.toSec()) );

    tracker_.update(blobs);

    vision_motion_detection::Tracks tracks_msg;
    tracks_msg.header = blobs_msg.header;
    for( Tracks::const_iterator it=tracker_.tracks.begin(); it!=tracker_.tracks.end(); ++it )
        tracks_msg.tracks.push_back( trackDataToMsg(*it) );

    tracks_pub_.publish(tracks_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_tracker");
    BlobTrackerNode node;
    ros::spin();
    return 0;
}
