#include <infrastructure_road_monitoring/Tracker.h>


Tracker::Tracker(TrackMatcher & matcher) : matcher_(matcher)
{
    unobserved_threshold_remove = 10;
}

void Tracker::update(const std::vector<Blob> & in_blobs)
{
    bool debug = false;

    if( tracks.empty() && in_blobs.empty() ) return;

    if( debug )
    {
        std::cout <<"--- Processing " <<in_blobs.size() <<" blobs."
                <<" Following " <<tracks.size() <<" tracks" <<std::endl;
        for( unsigned i=0; i<in_blobs.size(); i++ )
        {
            std::cout <<"Blob " <<i <<": centroid=(" <<in_blobs[i].centroid.x
                    <<"," <<in_blobs[i].centroid.y <<"), timestamp="
                    <<in_blobs[i].timestamp <<std::endl;
        }
    }

    if( tracks.empty() )
    {
        if(debug) std::cout <<"Adding blobs to tracker: creating tracks [";
        for( unsigned i=0; i<in_blobs.size(); i++ )
        {
            tracks.push_back( Track::newTrack(in_blobs[i]) );
            if(debug) {
                if( i!=0 ) std::cout <<", ";
                std::cout <<tracks.back().id;
            }
        }
        if(debug) std::cout <<']' <<std::endl;
        return;
    }

    // For each blob in in_blobs, check whether it is already tracked.
    //  - yes: update
    //  - no : create a new track
    // updatedTracks holds the iterator to the tracks that have been updated
    // newTracks holds the new tracks. These must be added later. If not,
    // next blob matching will bug (dt<0)
    std::vector<Tracks::iterator> updatedTracks;
    std::vector<Track> newTracks;
    for( unsigned i=0; i<in_blobs.size(); i++ )
    {
        if(debug) std::cout <<"Matching blob " <<i <<std::endl;
        Tracks::iterator it = matcher_.match(tracks, in_blobs[i]);
        if( it!=tracks.end() )
        {
            if(debug) std::cout <<"blob " <<i <<" matched with track " <<it->id <<std::endl;
            it->addObservation(in_blobs[i]);
            updatedTracks.push_back(it);
        }
        else
        {
            newTracks.push_back( Track::newTrack(in_blobs[i]) );
            if(debug) std::cout <<"blob " <<i <<" cannot be matched. Creating a new track: " <<tracks.back().id <<std::endl;
        }
    }

    // Wherever there was no update, create an unobserved observation
    for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); ++it )
    {
        if( find(updatedTracks.begin(), updatedTracks.end(), it)==updatedTracks.end() )
        {
            if(debug) std::cout <<"Track " <<it->id <<" unobserved" <<std::endl;
            Observation obs;
            obs.timestamp = in_blobs.empty() ? 0 : in_blobs.front().timestamp;
            it->observations.push_back( obs );
        }
    }

    // remove tracks that haven't been updated in a long time
    for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); )
    {
        if( it->latestObserved() - it->observations.rbegin() > (int)unobserved_threshold_remove )
        {
            if(debug) std::cout <<"Removing track " <<it->id <<std::endl;
            it = tracks.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // add new tracks
    tracks.insert(tracks.end(), newTracks.begin(), newTracks.end());

    if(debug)
    {
        // Display pos and vel of each track
        for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); ++it )
        {
            const cv::Point & p = it->latestObserved()->centroid;
            double vx=0, vy=0;
            try
            {
                vx = it->vel_x.value();
                vy = it->vel_y.value();
            }
            catch (std::runtime_error & e) { }
            ROS_INFO("Track %d: pos=(%03d, %03d), vel=(%+.3f, %+.3f)",
                    it->id, p.x, p.y, vx, vy);
        }
        ROS_INFO("---");
    }
}