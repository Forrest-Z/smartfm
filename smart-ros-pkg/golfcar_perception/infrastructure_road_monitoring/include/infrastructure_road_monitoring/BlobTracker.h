#ifndef BLOBTRACKER_H_
#define BLOBTRACKER_H_

#include <limits>

#include <boost/function.hpp>

#include <fmutil/fm_filter.h>

#include <infrastructure_road_monitoring/data_def.h>


/// Generates a threshold for the nearest neighbours matcher, given a blob and
/// a track. Abstract base class.
class MatcherThreshold
{
public:
    virtual double threshold(const Track & track, const Blob & b) = 0;
};

/// A threshold functor that returns a fixed value
class FixedMatcherThreshold : public MatcherThreshold
{
public:
    /// The threshold returned by the functor
    double th;

    /// Default constructor: sets the threshold value to infinity
    FixedMatcherThreshold();

    /// Constructs with a give threshold value
    explicit FixedMatcherThreshold(double t);

    /// Returns the set threshold
    double threshold(const Track & track, const Blob & b);
};

class AdaptiveMatcherThreshold : public MatcherThreshold
{
public:
    double threshold(const Track & track, const Blob & b);
};

/// Search all tracks for one that matches a given Blob. Abstract base class.
class TrackMatcher
{
public:
    /// Searches all tracks for one that matches a given Blob. If one is found
    /// then returns an iterator to it, otherwise returns the end iterator.
    virtual Tracks::iterator match(Tracks & tracks, const Blob & blob) = 0;
};



/// Nearest neighbour threshold track matcher
class TrackMatcherNNT : public TrackMatcher
{
public:
    typedef boost::function<double (const Track & track, const Blob & blob)> ThresholdFn;



public:
    /// Matching between an existing track and an blob occurs if the distance
    /// is smaller than this threshold
    MatcherThreshold * match_threshold;

    /// Once a match occurred, this is the distance to the closest match
    double match_distance;

    TrackMatcherNNT();

    /// Searches all tracks for one that matches a given Blob. If one is found
    /// then returns an iterator to it, otherwise returns the end iterator.
    /// A blob is matched to the nearest track, if the distance is smaller
    /// than a threshold (see match_treshold).
    Tracks::iterator match(Tracks & tracks, const Blob & blob);
};

#endif /* BLOBTRACKER_H_ */
