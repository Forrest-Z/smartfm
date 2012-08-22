#ifndef DATA_DEF_H_
#define DATA_DEF_H_

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>

#include <fmutil/fm_filter.h>

#include <vision_motion_detection/Blobs.h>
#include <vision_motion_detection/Tracks.h>


class Blob
{
public:
    std::vector<cv::Point> contour;
    cv::Point centroid;
    double timestamp; ///< time in seconds when this blob was extracted

    Blob();

    void drawContour(cv::Mat & displayFrame, cv::Scalar color) const;
    void drawCentroid(cv::Mat & displayFrame, cv::Scalar color, int radius=5, int thickness=-1) const;
};


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


vision_motion_detection::Blob blobDataToMsg(const Blob &);
void blobDataToMsg(vision_motion_detection::Blob *, const Blob &);

Blob blobMsgToData(const vision_motion_detection::Blob &, double time=0.0);
void blobMsgToData(Blob *, const vision_motion_detection::Blob &, double time=0.0);



vision_motion_detection::Track trackDataToMsg(const Track &);
void trackDataToMsg(vision_motion_detection::Track *, const Track &);

Track trackMsgToData(const vision_motion_detection::Track &);
void trackMsgToData(Track *, const vision_motion_detection::Track &);


#endif /* DATA_DEF_H_ */
