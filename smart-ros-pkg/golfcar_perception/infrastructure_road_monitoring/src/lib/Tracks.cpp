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
    const Observation & latest = * latestObserved();
    return sqrt( pow(latest.centroid.x - blob.centroid.x, 2)
               + pow(latest.centroid.y - blob.centroid.y, 2) );
}

Observations::reverse_iterator Track::latestObserved()
{
    Observations::reverse_iterator rit;
    for( rit=observations.rbegin(); rit!=observations.rend(); ++rit )
        if( rit->observed )
            return rit;
    cerr <<"Track::latestObserved(): No observed observation in Track." <<endl;
    abort();
    return observations.rend();
}

Observations::const_reverse_iterator Track::latestObserved() const
{
    Observations::const_reverse_iterator rit;
    for( rit=observations.rbegin(); rit!=observations.rend(); ++rit )
        if( rit->observed )
            return rit;
    cerr <<"Track::latestObserved(): No observed observation in Track." <<endl;
    abort();
    return observations.rend();
}

void Track::addObservation(const Blob & blob)
{
    if( observations.empty() ) {
        observations.push_back(Observation(blob));
    }
    else {
        Observation & latest = * latestObserved();
        Observation obs(blob, latest);
        observations.push_back(obs);
        double dt = obs.timestamp - latest.timestamp;
        vel_x.filter_dt(dt, obs.disp.x);
        vel_y.filter_dt(dt, obs.disp.y);
    }
}

void Track::display(cv::Mat displayFrame, cv::Scalar color)
{
    stringstream ss;
    ss << id;
    cv::putText(displayFrame, ss.str(), latestObserved()->centroid,
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 2,
                    color, 1, CV_AA);
}
