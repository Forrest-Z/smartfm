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

    if( rit==track.observations.rend() )
        return -1;

    double dt = b.timestamp - rit->timestamp;

    double alpha_y = 1 + (double)b.centroid.y / 720; //frame height is 720
    double th = dt * 150 * pow(alpha_y,5);

    //cout <<"dt=" <<dt <<", th=" <<th <<endl;
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
        cout <<"   distance to track " <<it->id <<": " <<d <<endl;
        if( d < match_distance && d < match_threshold->threshold(*it, blob) )
        {
            match_distance = d;
            matched_track_it = it;
        }
    }

    return matched_track_it;
}
