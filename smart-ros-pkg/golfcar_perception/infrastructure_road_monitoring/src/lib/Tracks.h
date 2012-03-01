#ifndef TRACKS_H_
#define TRACKS_H_

#include <vector>
#include <list>

#include <cv.h>

#include "Blob.h"

/// A class to hold Observation data
class Observation : public Blob
{
public:
    bool observed;  ///< was it observed
    cv::Point disp; ///< displacement w.r.t. prev position

    /// Creates an unobserved Observation
    Observation();

    /// Creates an observation from a Blob
    Observation(const Blob & blob);

    /// Creates an Observation from current Blob information and previous observation
    Observation(const Blob & blob, const Observation & pre);
};

typedef std::vector<Observation> Observations;

/// A class to hold history of observations
class Track
{
    /// ID counter
    static unsigned counter;

public:
    static double vel_filter_tau;

public:
    Observations observations;
    unsigned id;
    fmutil::LowPassFilter vel_x, vel_y;


public:
    /// Creates a Track with an ID of 0
    Track();

    /// Creates a Track with a given ID
    Track(unsigned);

    /// Creates a new Track with next available ID
    static Track newTrack();

    /// Creates a new Track with next available ID, and fill with
    // an observation from the blob
    static Track newTrack(const Blob & blob);

    /// computes the distance between a blob and the latest observed Observation in the track
    double distance(const Blob & blob) const;

    void addObservation(const Blob & blob);

    Observations::reverse_iterator latestObserved();
    Observations::const_reverse_iterator latestObserved() const;

    void display(cv::Mat displayFrame, cv::Scalar color=CV_RGB(255,0,0));
};


/// A typedef for collection of tracks
typedef std::list<Track> Tracks;


#endif /* TRACKS_H_ */
