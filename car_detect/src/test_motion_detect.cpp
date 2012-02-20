/* code from
 * http://stackoverflow.com/questions/8354037/opencv-findcontours-issue
 */

#include <stdio.h>

#include <iostream>
#include <vector>
#include <sstream>

#include <cv.h>
#include <highgui.h>

#include "GlobalClock.h"
#include "MovingObjectDetector.h"
#include "BlobExtractor.h"
#include "BlobFilter.h"
#include "BlobTracker.h"

using namespace std;


string format_time_frame(double t);
string format_time_file(double t);

int frame_width, frame_height;


double adaptiveThresholdFn(const Track & track, const Blob & b)
{
    vector<Observation>::const_reverse_iterator rit;
    for( rit=track.observations.rbegin(); rit!=track.observations.rend(); ++rit )
        if( rit->observed && rit->timestamp < b.timestamp )
            break;
    double dt = b.timestamp - rit->timestamp;

    double alpha_y = 1 + (double)b.centroid.y / frame_height;
    double th = dt * 150 * pow(alpha_y,5);



    cout <<"dt=" <<dt <<", th=" <<th <<endl;
    assert(dt>0);
    return th;
}


int main( int argc, char **argv )
{
    /******************************/
    /* Objects used for detection */
    /******************************/

    Background background;
    background.alpha = 0.005;

    MovingObjectDetector detector;
    detector.diff_thresh = 70;
    detector.dilate_size = 40;
    detector.erode_size  = 40;

    BlobExtractor blob_extractor;
    BlobFilterArea areaFilter(500);

    Tracks tracks;
    TrackMatcherNNT::ThresholdFn f = adaptiveThresholdFn;
    TrackMatcherNNT trackMatcher(tracks, f);
    BlobTracker tracker(tracks);
    tracker.matcher = &trackMatcher;
    tracker.unobserved_threshold_remove = 10;


    /******************************/

    cv::VideoCapture cap(argv[1]);
    if ( !cap.isOpened() ) return 1;

    double fps = cap.get(CV_CAP_PROP_FPS);
    cout <<"FPS: " <<fps <<endl;
    int speedFactor = 0;
    double inv_fps = 1.0/fps;
    double time=0;

    frame_width = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
    frame_height = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    bool quit = false;

    cv::namedWindow("car detection", CV_WINDOW_NORMAL);
    cv::namedWindow("background", CV_WINDOW_NORMAL);
    cv::namedWindow("diff", CV_WINDOW_NORMAL);

    cv::Mat frame, displayImg;

    // init the background with some frames
    for( unsigned i=0; i<fps; i++ ) {
        cap >> frame;
        background.add(frame);
    }


    while( ! quit )
    {

        // always capture one frame. If fast forwarding (i.e. speedFactor>0)
        // then capture more. Update background each time.
        // cannot rely on cap.get(CV_CAP_PROP_FPS) to get the time....
        for( unsigned i=0; 1; i++ ) {
            cap >> frame;
            background.add(frame);
            time += inv_fps;
            if( speedFactor<=0 || i>=pow(speedFactor,2) )
                break;
        }
        displayImg = frame.clone();

        // add timestamp
        //cout <<"Current camera time: " <<time <<endl;
        cv::putText(displayImg, format_time_frame(time), cv::Point(30,50),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 2,
                CV_RGB(0,255,555), 1, CV_AA);


        detector.diff(frame, background.getImg());

        blob_extractor.extract(detector.diffImg, time);
        areaFilter.filter(blob_extractor.blobs);
        blob_extractor.display(displayImg, CV_RGB(255, 0, 0));

        tracker.update(blob_extractor.blobs);
        tracker.display(displayImg, CV_RGB(255,0,0));


        for(Tracks::iterator it=tracker.tracks.begin(); it!=tracker.tracks.end(); ++it ) {
            try {
                double vx = it->vel_x.value(), vy = it->vel_y.value();
                cout <<"TrackInfo: " <<it->id <<": pos=(" <<it->latest_blob_info.centroid.x
                        <<"," <<it->latest_blob_info.centroid.y <<"), vel=("
                        <<vx <<"," <<vy <<"), "
                        <<"observed: " <<it->observations.back().observed;
                if( vx<0 && (vy<0 || it->latest_blob_info.centroid.y<180) ) {
                    it->latest_blob_info.drawContour(displayImg, CV_RGB(0, 255, 0)); //green
                    cout <<", green";
                }
                cout <<endl;
            } catch( runtime_error & e ) {

            }

        }


        cv::imshow("car detection", displayImg);
        cv::imshow("background", background.getImg());
        cv::imshow("diff", detector.diffImg);


        // listen to user input; control playback speed
        double pre = GlobalClock::time();
        bool wait = true;
        while( ! quit && wait )
        {
            int key = cv::waitKey((int)(inv_fps*1000));

            if( key=='q' ) quit = true;
            else if( key=='f' ) speedFactor++;
            else if( key=='s' ) speedFactor--;
            else if( key=='d' ) speedFactor = 0;
            else if( key=='c' ) {
                string t = format_time_file(time);
                cv::imwrite("images/" + t + "_background.png", background.getImg());
                cv::imwrite("images/" + t +"_frame.png", frame);
            }

            double twait = inv_fps;
            if( speedFactor<0 ) twait *= pow(2,-speedFactor);
            if( GlobalClock::time() > pre+twait ) wait = false;
            //else cout <<"Waiting" <<endl;
        }
    }

    return 0;
}

string format_time_frame(double t)
{
    int mins = t/60;
    t -= mins*60;
    int secs = t;
    t -= secs;
    int ms = t*1000;
    char buf[100];
    snprintf(buf, 99, "%02d:%02d.%03d", mins, secs, ms);
    return string(buf);
}

string format_time_file(double t)
{
    int mins = t/60;
    t -= mins*60;
    int secs = t;
    t -= secs;
    int ms = t*1000;
    char buf[100];
    snprintf(buf, 99, "%02d_%02d_%03d", mins, secs, ms);
    return string(buf);
}

