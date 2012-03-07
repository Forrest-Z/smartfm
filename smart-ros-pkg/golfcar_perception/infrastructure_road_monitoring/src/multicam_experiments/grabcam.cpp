// Grab images from one camera and display. Using OpenCV for frame grabbing.

// compile with:
// gcc -Wall -g grabcam.cpp -o grabcam -I/usr/include/opencv-2.3.1 -lopencv_core -lopencv_highgui

#include <iostream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main(int argc, char **argv)
{
    int camid = 0;
    if( argc>1 ) camid = atoi(argv[1]);

    cv::VideoCapture cap(camid);
    if ( !cap.isOpened() ) {
        cerr <<"Error opening the source (" <<camid <<")." <<endl;
        return 1;
    }

    cv::Mat frame;
    stringstream ss;
    ss <<"camera" <<camid;
    string name = ss.str();
    cv::namedWindow(name, CV_WINDOW_NORMAL);

    while(1)
    {
        cap >> frame;
        cv::imshow(name, frame);
        if( cv::waitKey(30) == 'q' )
            break;
    }

    return 0;
}
