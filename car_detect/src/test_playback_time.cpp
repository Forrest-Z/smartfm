#include <stdio.h>

#include <cv.h>
#include <highgui.h>

#include "GlobalClock.h"

int main( int argc, char **argv )
{
    cv::VideoCapture cap(argv[1]);
    if ( !cap.isOpened() ) return 1;
    double fps = cap.get(CV_CAP_PROP_FPS);
    printf("FPS: %f\n", fps);

    cv::Mat frame;
    double time, prev_time=0;

    cv::namedWindow("movie", CV_WINDOW_NORMAL);

    while(1)
    {
        cap >> frame;
        cv::imshow("movie", frame);
        time = cap.get(CV_CAP_PROP_POS_MSEC)/1000;
        double dt  = time-prev_time;
        double wt = GlobalClock::time();
        printf("camera time: %.02fs, dt=%dms, wall time=%.02fs, diff=%dms\n",
                time, (int)(dt*1000), wt, (int)((wt-time)*1000) );
        prev_time = time;
        //assert(dt>0);
        if( cv::waitKey(1000.0/fps*0.67)=='q' ) break;
        //sleep(1);
    }

    return 0;
}
