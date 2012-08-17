#ifndef __TRACKER_H__
#define __TRACKER_H__

#include <infrastructure_road_monitoring/Tracks.h>
#include <infrastructure_road_monitoring/Blobs.h>
#include <infrastructure_road_monitoring/TrackMatcher.h>
#include <infrastructure_road_monitoring/data_types.h>


class Tracker
{
public:
    Tracker(TrackMatcher &);

    void update(const std::vector<Blob> & in_blobs);

    /// the collection of tracks
    Tracks tracks;

    /// the track matcher
    TrackMatcher & matcher_;

    /// delete a track if it has not been observed for this number of consecutive rounds
    unsigned unobserved_threshold_remove;
};

#endif // __TRACKER_H__
