// Grab images alternatively from several cameras and display. Using OpenCV for
// frame grabbing.

// compile with:
// gcc -Wall -g multicam.cpp -o multicam -I/usr/include/opencv-2.3.1 -lopencv_core -lopencv_highgui

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class Camera
{
public:
    Camera(int id) : id_(id), cap_(id)
    {
        stringstream ss;
        ss <<"camera " <<id;
        wname_ = ss.str();
        if( ! cap_.isOpened() )
            throw runtime_error(string("Error opening ") + wname_);
        cv::namedWindow(wname_, CV_WINDOW_NORMAL);
    }

    cv::Mat & grab()
    {
        cap_ >>frame_;
        cv::imshow(wname_, frame_);
        return frame_;
    }

private:
    int id_;
    cv::VideoCapture cap_;
    string wname_;
    cv::Mat frame_;
};

int main(int argc, char **argv)
{
    vector<Camera *> cams;
    cams.push_back( new Camera(1) );
    cams.push_back( new Camera(2) );

    int counter = 0;

    while(1)
    {
        cams[counter++ % cams.size()] -> grab();
        if( cv::waitKey(15) == 'q' )
            break;
    }

    return 0;
}
