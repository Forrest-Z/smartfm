#include <stdexcept>
#include <limits>
#include <cassert>

#include <vision_motion_detection/data_types.h>

using namespace std;


Blob::Blob() : timestamp(0.0)
{

}

void Blob::drawContour(cv::Mat & displayFrame, cv::Scalar color) const
{
    std::vector< std::vector<cv::Point> > v;
    v.push_back( contour );
    cv::drawContours( displayFrame, v, -1, color, 2, 8 );
}

void Blob::drawCentroid(cv::Mat & displayFrame, cv::Scalar color, int radius, int thickness) const
{
    cv::circle(displayFrame, centroid, radius, color, thickness);
}



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




vision_motion_detection::Blob blobDataToMsg(const Blob & data)
{
    vision_motion_detection::Blob msg;
    blobDataToMsg(&msg, data);
    return msg;
}

void blobDataToMsg(vision_motion_detection::Blob * msg, const Blob & data)
{
    msg->contour.clear();
    for(unsigned i=0; i<data.contour.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = data.contour[i].x;
        p.y = data.contour[i].y;
        msg->contour.push_back(p);
    }
    msg->centroid.x = data.centroid.x;
    msg->centroid.y = data.centroid.y;
    msg->centroid.z = 0;
}

Blob blobMsgToData(const vision_motion_detection::Blob & msg, double time)
{
    Blob data;
    blobMsgToData(&data, msg, time);
    return data;
}

void blobMsgToData(Blob * data, const vision_motion_detection::Blob & msg, double time)
{
    data->contour.clear();
    for(unsigned i=0; i<msg.contour.size(); i++)
    {
        cv::Point p;
        p.x = (int) msg.contour[i].x;
        p.y = (int) msg.contour[i].y;
        data->contour.push_back(p);
    }
    data->centroid.x = (int) msg.centroid.x;
    data->centroid.y = (int) msg.centroid.y;
    data->timestamp = time;
}




vision_motion_detection::Track trackDataToMsg(const Track & data)
{
    vision_motion_detection::Track msg;
    trackDataToMsg(&msg, data);
    return msg;
}

void trackDataToMsg(vision_motion_detection::Track *msg, const Track & data)
{
    msg->id = data.id;
    try { msg->xvel = data.vel_x.value(); } catch(std::runtime_error & e) { msg->xvel=0; }
    try { msg->yvel = data.vel_y.value(); } catch(std::runtime_error & e) { msg->yvel=0; }
    blobDataToMsg(&(msg->blob), *(data.latestObserved()));
}

Track trackMsgToData(const vision_motion_detection::Track & msg)
{
    Track data;
    trackMsgToData(&data, msg);
    return data;
}

void trackMsgToData(Track * data, const vision_motion_detection::Track & msg)
{
    data->id = msg.id;
    data->vel_x.value(msg.xvel);
    data->vel_y.value(msg.yvel);
    data->observations.push_back( blobMsgToData(msg.blob, 0) );
}