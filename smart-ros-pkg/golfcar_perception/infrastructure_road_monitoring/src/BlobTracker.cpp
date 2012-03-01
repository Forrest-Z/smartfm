#include <infrastructure_road_monitoring/BlobTracker.h>

using namespace std;

FixedMatcherThreshold::FixedMatcherThreshold()
: th( std::numeric_limits<double>::infinity() )
{

}

FixedMatcherThreshold::FixedMatcherThreshold(double t) : th(t)
{

}

double FixedMatcherThreshold::threshold(const Track & track, const Blob & b)
{
    return th;
}

double AdaptiveMatcherThreshold::threshold(const Track & track, const Blob & b)
{
    Observations::const_reverse_iterator rit;
    for( rit=track.observations.rbegin(); rit!=track.observations.rend(); ++rit )
        if( rit->observed && rit->timestamp < b.timestamp )
            break;
    double dt = b.timestamp - rit->timestamp;

    double alpha_y = 1 + (double)b.centroid.y / 720; //frame height is 720
    double th = dt * 150 * pow(alpha_y,5);

    cout <<"dt=" <<dt <<", th=" <<th <<endl;
    assert(dt>0);
    return th;
}

TrackMatcherNNT::TrackMatcherNNT()
: match_threshold(0)
{

}

Tracks::iterator TrackMatcherNNT::match(Tracks & tracks, const Blob & blob)
{
    assert( match_threshold!=0 );

    match_distance = numeric_limits<double>::infinity();
    Tracks::iterator matched_track_it = tracks.end();

    for( Tracks::iterator it = tracks.begin(); it!=tracks.end(); ++it )
    {
        double d = it->distance(blob);
        cout <<"distance to track " <<it->id <<": " <<d <<endl;
        if( d < match_distance && d < match_threshold->threshold(*it, blob) )
        {
            match_distance = d;
            matched_track_it = it;
        }
    }

    return matched_track_it;
}





BlobTracker::BlobTracker()
: matcher(0), unobserved_threshold_remove(10)
{

}

void BlobTracker::update(const vector<Blob> & in_blobs)
{
    if( tracks.empty() && in_blobs.empty() ) return;

    cout <<"--- Processing " <<in_blobs.size() <<" blobs. Following " <<tracks.size() <<" tracks" <<endl;
    for( unsigned i=0; i<in_blobs.size(); i++ ) {
        cout <<"Blob " <<i <<": centroid=(" <<in_blobs[i].centroid.x
                <<"," <<in_blobs[i].centroid.y <<"), timestamp="
                <<in_blobs[i].timestamp <<endl;
    }

    if( tracks.empty() ) {
        cout <<"Adding blobs to tracker: creating tracks [";
        for( unsigned i=0; i<in_blobs.size(); i++ ) {
            tracks.push_back( Track::newTrack(in_blobs[i]) );
            if( i!=0 ) cout <<", ";
            cout <<tracks.back().id;
        }
        cout <<']' <<endl;
        return;
    }

    // For each blob in in_blobs, check whether it is already tracked.
    //  - yes: update
    //  - no : create a new track
    // updatedTracks holds the iterator to the tracks that have been updated
    vector<Tracks::iterator> updatedTracks;
    for( unsigned i=0; i<in_blobs.size(); i++ ) {
        cout <<"Matching blob " <<i <<endl;
        Tracks::iterator it = matcher->match(tracks, in_blobs[i]);
        if( it!=tracks.end() ) {
            cout <<"blob " <<i <<" matched with track " <<it->id <<endl;
            it->addObservation(in_blobs[i]);
            updatedTracks.push_back(it);
        }
        else {
            it = tracks.insert( tracks.end(), Track::newTrack(in_blobs[i]) );
            updatedTracks.push_back(it);
            cout <<"blob " <<i <<" cannot be matched. Creating a new track: " <<tracks.back().id <<endl;
        }
    }

    // Wherever there was no update, create an unobserved observation
    for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); ++it )
    {
        if( find(updatedTracks.begin(), updatedTracks.end(), it)==updatedTracks.end() )
        {
            cout <<"Track " <<it->id <<" unobserved" <<endl;
            Observation obs;
            obs.timestamp = in_blobs.empty() ? 0 : in_blobs.front().timestamp;
            it->observations.push_back( obs );
        }
    }

    // remove tracks that haven't been updated in a long time
    for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); )
    {
        if( it->latestObserved() - it->observations.rbegin() > (int)unobserved_threshold_remove ) {
            cout <<"Removing track " <<it->id <<endl;
            it = tracks.erase(it);
        }
        else {
            ++it;
        }
    }
}

void BlobTracker::display(cv::Mat & displayFrame, cv::Scalar color)
{
    for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); ++it )
        it->display(displayFrame, color);
}
