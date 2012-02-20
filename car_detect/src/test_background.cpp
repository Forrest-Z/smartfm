#include <cv.h>
#include <highgui.h>

#include "MovingAvgBackground.h"

using namespace cv;

int main(int argc, char **argv)
{
    VideoCapture cap(argv[1]);
    if(!cap.isOpened())  // check if we succeeded
        return 1;

    Mat blurred;
    MovingAvgBackground * background;
    background = new ManualMovingAvgBackground(50);
    //background = new WeightedMovingAvgBackground(0.002);

    namedWindow("movie",1);
    namedWindow("background",1);
    namedWindow("diff",1);

    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from video

        cvtColor(frame, blurred, CV_BGR2GRAY);
        GaussianBlur(blurred, blurred, Size(7,7), 1.5, 1.5);
        background->add(blurred);

        Mat diff;
        absdiff(blurred, background->getBackgroundImage(), diff);
        threshold(diff, diff, 70, 255, CV_THRESH_BINARY);

        imshow("movie", frame);
        imshow("background", background->getBackgroundImage() );
        imshow("diff", diff);
        if(waitKey(30) >= 0) break;
    }


    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
