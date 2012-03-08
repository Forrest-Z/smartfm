// Grab images alternatively from several cameras and display. Using OpenCV for
// frame grabbing. Grab and close the device.

// compile with:
// gcc -Wall -g multicamclose.cpp -o multicamclose -I/usr/include/opencv-2.3.1 -lopencv_core -lopencv_highgui

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
    Camera(int id) : id_(id)
    {
        stringstream ss;
        ss <<"camera " <<id;
        wname_ = ss.str();
        cv::namedWindow(wname_, CV_WINDOW_NORMAL);
    }

    cv::Mat & grab()
    {
        cv::VideoCapture cap(id_);
        if( ! cap.isOpened() )
            throw runtime_error(string("Error opening ") + wname_);
        cap >>frame_;
        cv::imshow(wname_, frame_);
        return frame_;
    }

private:
    int id_;
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
