#include "BlobTracker.h"

#include <stdexcept>
#include <limits>
#include <cassert>

using namespace std;

Observation::Observation() : observed(false)
{

}

Observation::Observation(const Blob & blob)
: Blob(blob),
  observed(true),
  disp(INT_MAX,INT_MAX)
{

}

Observation::Observation(const Blob & blob, const Observation & prevObs)
: Blob(blob),
  observed(true),
  disp(INT_MAX,INT_MAX)
{
    if( prevObs.observed )
    {
        disp.x = centroid.x - prevObs.centroid.x;
        disp.y = centroid.y - prevObs.centroid.y;
    }
}




unsigned Track::counter = 1;
double Track::vel_filter_tau = 1;

Track::Track() : id(0), vel_x(vel_filter_tau), vel_y(vel_filter_tau)
{

}

Track::Track(unsigned i) : id(i), vel_x(vel_filter_tau), vel_y(vel_filter_tau)
{

}

Track Track::newTrack()
{
    return Track(counter++);
}

Track Track::newTrack(const Blob & blob)
{
    Track track = Track::newTrack();
    track.addObservation(blob);
    return track;
}

double Track::distance(const Blob & blob) const
{
    const Observation & o = latestObserved();
    return sqrt( pow(o.centroid.x - blob.centroid.x, 2)
                + pow(o.centroid.y - blob.centroid.y, 2) );
}

Observation & Track::latestObserved()
{
    vector<Observation>::reverse_iterator rit;
    for( rit=observations.rbegin(); rit!=observations.rend(); ++rit )
        if( rit->observed )
            return *rit;
    throw runtime_error("no observed observation in track");
    return observations[0];
}

const Observation & Track::latestObserved() const
{
    vector<Observation>::const_reverse_iterator rit;
    for( rit=observations.rbegin(); rit!=observations.rend(); ++rit )
        if( rit->observed )
            return *rit;
    throw runtime_error("no observed observation in track");
    return observations[0];
}

void Track::addObservation(const Blob & blob)
{
    if( observations.empty() ) {
        observations.push_back(Observation(blob));
    }
    else {
        Observation & latest = latestObserved();
        Observation obs(blob, latest);
        observations.push_back(obs);
        double dt = obs.timestamp - latest.timestamp;
        vel_x.filter_dt(dt, obs.disp.x);
        vel_y.filter_dt(dt, obs.disp.y);
    }
    latest_blob_info = blob;
}






TrackMatcher::TrackMatcher(Tracks & t)
: tracks(t), matched_track_it(t.end())
{

}

TrackMatcherNNT::FixedThreshold::FixedThreshold()
: th( std::numeric_limits<double>::infinity() )
{

}

TrackMatcherNNT::FixedThreshold::FixedThreshold(double t) : th(t)
{

}

double TrackMatcherNNT::FixedThreshold::operator() (const Track & track, const Blob & b)
{
    return th;
}

TrackMatcherNNT::TrackMatcherNNT(Tracks & t)
: TrackMatcher(t), match_threshold(TrackMatcherNNT::FixedThreshold())
{

}

TrackMatcherNNT::TrackMatcherNNT(Tracks & t, double threshold)
: TrackMatcher(t), match_threshold(TrackMatcherNNT::FixedThreshold(threshold))
{

}

TrackMatcherNNT::TrackMatcherNNT(Tracks & t, TrackMatcherNNT::ThresholdFn & f)
: TrackMatcher(t), match_threshold(f)
{

}

bool TrackMatcherNNT::match(const Blob & blob)
{
    if( tracks.empty() )
        return false;

    match_distance = numeric_limits<double>::infinity();
    matched_track_it = tracks.end();

    for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); ++it )
    {
        double d = it->distance(blob);
        cout <<"distance to track " <<it->id <<": " <<d <<endl;
        if( d < match_distance && d < match_threshold(*it, blob) )
        {
            match_distance = d;
            matched_track_it = it;
        }
    }

    return matched_track_it != tracks.end();
}





BlobTracker::BlobTracker(Tracks & t)
: tracks(t),
  unobserved_threshold_remove(UINT_MAX)
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
    for( unsigned i=0; i<in_blobs.size(); i++ ) {
        cout <<"Matching blob " <<i <<endl;
        if( matcher->match(in_blobs[i]) ) {
            cout <<"blob " <<i <<" matched with track " <<matcher->matched_track_it->id <<endl;
            matcher->matched_track_it->addObservation(in_blobs[i]);
        }
        else {
            tracks.push_back( Track::newTrack(in_blobs[i]) );
            cout <<"blob " <<i <<" cannot be matched. Creating a new track: " <<tracks.back().id <<endl;
        }
    }

    /// Wherever there was no update, create an unobserved observation
    /// and delete the track if necessary
    for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); )
    {
        if( it->observations.back().timestamp < in_blobs.front().timestamp )
        {
            cout <<"Track " <<it->id <<" unobserved" <<endl;
            Observation obs;
            obs.timestamp = in_blobs.front().timestamp;
            it->observations.push_back( obs );

            vector<Observation>::const_reverse_iterator rit;
            for( rit=it->observations.rbegin(); rit!=it->observations.rend() && ! rit->observed; ++rit );

            if( rit - it->observations.rbegin() > (int)unobserved_threshold_remove ) {
                cout <<"Removing track " <<it->id <<endl;
                it = tracks.erase(it);
                continue;
            }
        }
        ++it;
    }
}

void BlobTracker::display(cv::Mat & displayFrame, cv::Scalar color)
{
    for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); ++it )
    {
        stringstream ss;
        ss << it->id;
        cv::putText(displayFrame, ss.str(), it->latestObserved().centroid,
                        cv::FONT_HERSHEY_COMPLEX_SMALL, 2,
                        color, 1, CV_AA);
    }
}
