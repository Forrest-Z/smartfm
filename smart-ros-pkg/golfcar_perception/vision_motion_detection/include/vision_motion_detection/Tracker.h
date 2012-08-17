#ifndef __TRACKER_H__
#define __TRACKER_H__

#include <vision_motion_detection/Tracks.h>
#include <vision_motion_detection/Blobs.h>
#include <vision_motion_detection/TrackMatcher.h>
#include <vision_motion_detection/data_types.h>


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
