#include "CarDetector.h"

using namespace std;


CarDetector::CarDetector()
: areaFilter(500)

{
    background.alpha = 0.005;
    detector.diff_thresh = 70;
    detector.dilate_size = 40;
    detector.erode_size  = 40;
    trackMatcher.match_threshold = new AdaptiveMatcherThreshold();
    tracker.matcher = &trackMatcher;
    tracker.unobserved_threshold_remove = 10;
}



bool CarDetector::update(cv::Mat frame, double time)
{
    detector.diff(frame, background.getImg());

    blob_extractor.extract(detector.diffImg, time);
    areaFilter.filter(blob_extractor.blobs);

    tracker.update(blob_extractor.blobs);

    bool road_is_clear = true;
    for(Tracks::iterator it=tracker.tracks.begin(); it!=tracker.tracks.end(); ++it ) {
        try {
            double vx = it->vel_x.value(), vy = it->vel_y.value();
            cout <<"TrackInfo: " <<it->id <<": pos=(" <<it->latestObserved().centroid.x
                    <<"," <<it->latestObserved().centroid.y <<"), vel=("
                    <<vx <<"," <<vy <<"), "
                    <<"observed: " <<it->observations.back().observed;

            if( vx<0 && (vy<0 || it->latestObserved().centroid.y<180) )
                cout <<", green";
            else
                road_is_clear = false;

            cout <<endl;

        } catch( runtime_error & e ) {

        }

    }

    return road_is_clear;
}

void CarDetector::display(cv::Mat & displayImg)
{
    blob_extractor.display(displayImg, CV_RGB(255, 0, 0));
    tracker.display(displayImg, CV_RGB(255,0,0));
    for(Tracks::iterator it=tracker.tracks.begin(); it!=tracker.tracks.end(); ++it ) {
        try {
            double vx = it->vel_x.value(), vy = it->vel_y.value();
            if( vx<0 && (vy<0 || it->latestObserved().centroid.y<180) ) {
                it->latestObserved().drawContour(displayImg, CV_RGB(0, 255, 0)); //green
            }
        } catch( runtime_error & e ) {
        }
    }
}
