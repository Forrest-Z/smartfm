#include "GlobalClock.h"
#include "CarDetector.h"

using namespace std;


int main( int argc, char **argv )
{
    cv::VideoCapture cap(0);
    if ( !cap.isOpened() ) throw runtime_error("Could not open camera");

    CarDetector detector;

    detector.frame_width = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
    detector.frame_height = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    cv::namedWindow("camera 0", CV_WINDOW_NORMAL);
    //cv::namedWindow("background", CV_WINDOW_NORMAL);
    //cv::namedWindow("diff", CV_WINDOW_NORMAL);


    cv::Mat frame;

    while( 1 )
    {
        cap >> frame;
        cv::Mat displayImg = frame.clone();

        detector.background.add(frame);
        detector.update(frame, GlobalClock::time());

        detector.display(displayImg);
        cv::imshow("camera 0", displayImg);

        int key = cv::waitKey(10);
        if( key=='q' ) return 0;
    }

    return 0;
}
