#ifndef TRACKER_H_
#define TRACKER_H_

#include <utility>
#include <vector>
#include <list>

/** A track based on class BASE.
 *
 * T is expected to define the following methods:
 * - copy constructor
 * - assignment operator
 * - vector<unsigned> update(const vector<BASE> & obs)
 *     which updates the status of the track given the set of observations. It
 *     returns the observations that have been used in the update.
 * - string to_str() const
 *     which returns a string describing the state of the track (pos, vel, etc.)
 */
template <class BASE>
class Track : public BASE
{
    /// ID counter
    static unsigned id_counter;
    unsigned id_;
    unsigned unobserved_;

    Track(const BASE & obs) : BASE(obs) {}

public:
    Track(const Track &);
    Track & operator= (const Track &);

    /// Creates a new Track with next available ID, and fill with
    /// an observation from the observation
    static Track newTrack(const BASE & obs);

    unsigned get_id() const { return id_; }

    unsigned get_unobserved() const { return unobserved_; }

    /// update and reset the unobserved counter
    vector<unsigned> update(const vector<BASE> & obs)
    {
        vector<unsigned> i = BASE::update(obs);
        unobserved_ = i.empty() ? unobserved_+1 : 0;
        return i;
    }
};

template <class BASE>
static unsigned Track<BASE>::id_counter = 0;

template <class BASE>
Track<BASE>::Track(const Track & t)
: BASE(t), id_(t.id_), unobserved_(t.unobserved_)
{

}

template <class BASE>
Track & Track<BASE>::operator= (const Track & t)
{
    if( &t==this ) return *this;
    BASE::operator=(t);
    id_ = t.id_;
    return *this;
}

template <class BASE>
Track Track<BASE>::newTrack(const BASE & obs)
{
    Track track(obs);
    track.id_ = Track::id_counter++;
    track.unobserved_ = 0;
    return track;
}

#define Tracks std::list< Track<BASE> >

template <class BASE>
class Tracker
{
    Tracks tracks;
    std::list<unsigned> unmatched_;

    void update_unmatched(const std::vector<unsigned> & used_obs);

public:
    void update(const std::vector<BASE> &);
};

template <class BASE>
void Tracker<BASE>::update(const std::vector<BASE> & observations)
{
    static const bool debug = false;

    if( tracks.empty() && observations.empty() ) return;

    if( debug )
    {
        std::cout <<"--- Processing " <<observations.size() <<" observations."
                <<" Following " <<tracks.size() <<" tracks" <<std::endl;
        for( unsigned i=0; i<observations.size(); i++ )
        {
            std::cout <<"observation " <<i <<": " <<observations[i] <<std::endl;
        }
    }

    if( tracks.empty() )
    {
        if(debug) std::cout <<"Adding observations to tracker: creating tracks [";
        for( unsigned i=0; i<observations.size(); i++ )
        {
            tracks.push_back( Track::newTrack(observations[i]) );
            if(debug) {
                if( i!=0 ) std::cout <<", ";
                std::cout <<tracks.back().id;
            }
        }
        if(debug) std::cout <<']' <<std::endl;
        return;
    }


    // keep a list of observation indices that have not been used to update a track
    unmatched_.clear();
    for( unsigned i=0; i<observations.size(); i++ )
        unmatched_.push_back(i);

    // For each track, update it with the set of observations. Keep a record of
    // observations that were used
    for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); )
    {
        if(debug) std::cout <<"Updating track " <<it->get_id() <<std::endl;
        std::vector<unsigned> used_obs = it->update(observations);
        if( used_obs.empty() )
        {
            if(debug) std::cout <<"Track " <<it->get_id() <<" unobserved" <<std::endl;
            if( it->get_unobserved() > unobserved_threshold_remove )
            {
                if(debug) std::cout <<"Removing track " <<it->get_id() <<std::endl;
                it = tracks.erase(it);
                continue;
            }
        }
        else
        {
            update_unmatched(used_obs);
        }
        ++it;
    }

    for( std::list<unsigned>::iterator unmatched_it = unmatched_.begin(); unmatched_it!=unmatched_.end(); ++unmatched_it )
    {
        tracks.push_back( Track<BASE>::newTrack(observations[*unmatched_it]) );
        if(debug) std::cout <<"Observation " <<*unmatched_it <<" cannot be matched. Creating a new track: " <<tracks.back().get_id() <<std::endl;
    }

    if(debug)
    {
        // Display info about each track
        for( Tracks::iterator it=tracks.begin(); it!=tracks.end(); ++it )
            std::cout <<"Track " <<it->get_id() <<": " <<it->to_str() <<std::endl;
        std::cout <<"---" <<std::endl;
    }
}

template <class BASE>
void Tracker<BASE>::update_unmatched(const std::vector<unsigned> & used_obs)
{
    std::vector<unsigned>::iterator used_obs_it = used_obs.begin();
    std::list<unsigned>::iterator unmatched_it = unmatched_.begin();
    for( ; used_obs_it!=used_obs.end() && unmatched_it!=unmatched_.end(); )
    {
        if( *used_obs_it == *unmatched_it )
        {
            unmatched_it = unmatched_.erase(unmatched_it);
            ++used_obs_it;
        }
        else
        {
            ++unmatched_it;
        }
    }
}

#endif /* TRACKER_H_ */
