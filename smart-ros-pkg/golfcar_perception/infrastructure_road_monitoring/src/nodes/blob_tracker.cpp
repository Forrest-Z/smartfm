#include <ros/ros.h>

#include <infrastructure_road_monitoring/Tracks.h>
#include <infrastructure_road_monitoring/Blobs.h>
#include <infrastructure_road_monitoring/BlobTracker.h>

#include "data_msg_conv.h"

using namespace std;

class BlobTrackerNode
{
public:
    BlobTrackerNode()
    : matcher(0), unobserved_threshold_remove(10)
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

    Tracks tracks;

    TrackMatcher * matcher;
    unsigned unobserved_threshold_remove;

    AdaptiveMatcherThreshold amt;
    TrackMatcherNNT tmnnt;


    void blobsCallback(const infrastructure_road_monitoring::Blobs & blobs_msg);

    void update(const vector<Blob> & in_blobs);
};


void BlobTrackerNode::blobsCallback(const infrastructure_road_monitoring::Blobs & blobs_msg)
{
	std::vector<Blob> blobs;
	for( unsigned i=0; i<blobs_msg.blobs.size(); i++ )
		blobs.push_back( blobMsgToData(blobs_msg.blobs[i], blobs_msg.header.stamp.toSec()) );

	update(blobs);

	infrastructure_road_monitoring::Tracks tracks_msg;
	tracks_msg.header = blobs_msg.header;
	for( Tracks::const_iterator it=tracks.begin(); it!=tracks.end(); ++it )
		tracks_msg.tracks.push_back( trackDataToMsg(*it) );

	tracks_pub_.publish(tracks_msg);
}

void BlobTrackerNode::update(const vector<Blob> & in_blobs)
{
    if( tracks.empty() && in_blobs.empty() ) return;

    /* cout <<"--- Processing " <<in_blobs.size() <<" blobs. Following " <<tracks.size() <<" tracks" <<endl;
    for( unsigned i=0; i<in_blobs.size(); i++ ) {
        cout <<"Blob " <<i <<": centroid=(" <<in_blobs[i].centroid.x
                <<"," <<in_blobs[i].centroid.y <<"), timestamp="
                <<in_blobs[i].timestamp <<endl;
    } */

    if( tracks.empty() ) {
        //cout <<"Adding blobs to tracker: creating tracks [";
        for( unsigned i=0; i<in_blobs.size(); i++ ) {
            tracks.push_back( Track::newTrack(in_blobs[i]) );
            //if( i!=0 ) cout <<", ";
            //cout <<tracks.back().id;
        }
        //cout <<']' <<endl;
        return;
    }

    // For each blob in in_blobs, check whether it is already tracked.
    //  - yes: update
    //  - no : create a new track
    // updatedTracks holds the iterator to the tracks that have been updated
    vector<Tracks::iterator> updatedTracks;
    for( unsigned i=0; i<in_blobs.size(); i++ ) {
        //cout <<"Matching blob " <<i <<endl;
        Tracks::iterator it = matcher->match(tracks, in_blobs[i]);
        if( it!=tracks.end() ) {
            //cout <<"blob " <<i <<" matched with track " <<it->id <<endl;
            it->addObservation(in_blobs[i]);
            updatedTracks.push_back(it);
        }
        else {
            it = tracks.insert( tracks.end(), Track::newTrack(in_blobs[i]) );
            updatedTracks.push_back(it);
            //cout <<"blob " <<i <<" cannot be matched. Creating a new track: " <<tracks.back().id <<endl;
        }
    }

    // Wherever there was no update, create an unobserved observation
    for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); ++it )
    {
        if( find(updatedTracks.begin(), updatedTracks.end(), it)==updatedTracks.end() )
        {
            //cout <<"Track " <<it->id <<" unobserved" <<endl;
            Observation obs;
            obs.timestamp = in_blobs.empty() ? 0 : in_blobs.front().timestamp;
            it->observations.push_back( obs );
        }
    }

    // remove tracks that haven't been updated in a long time
    for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); )
    {
        if( it->latestObserved() - it->observations.rbegin() > (int)unobserved_threshold_remove ) {
            //cout <<"Removing track " <<it->id <<endl;
            it = tracks.erase(it);
        }
        else {
            ++it;
        }
    }

    /*
    // Display pos and vel of each track
    for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); ++it ) {
    	const cv::Point & p = it->latestObserved()->centroid;
    	double vx=0, vy=0;
    	try {
    		vx = it->vel_x.value();
    		vy = it->vel_y.value();
    	} catch (runtime_error & e) {
    	}
    	ROS_INFO("Track %d: pos=(%03d, %03d), vel=(%+.3f, %+.3f)",
    			it->id, p.x, p.y, vx, vy);
    }
    ROS_INFO("---");
    */
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_tracker");
    BlobTrackerNode node;
    ros::spin();
    return 0;
}
