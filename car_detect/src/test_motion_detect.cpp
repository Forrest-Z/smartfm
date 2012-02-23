#include "GlobalClock.h"
#include "CarDetector.h"

using namespace std;


string format_time_frame(double t);
string format_time_file(double t);

int main( int argc, char **argv )
{
    CarDetector detector;

    cv::VideoCapture cap(argv[1]);
    if ( !cap.isOpened() )
        throw runtime_error(string("Could not open ") + string(argv[1]));

    double fps = cap.get(CV_CAP_PROP_FPS);
    cout <<"FPS: " <<fps <<endl;
    int speedFactor = 0;
    double inv_fps = 1.0/fps;
    double time=0;

    detector.frame_width = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
    detector.frame_height = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    bool quit = false;

    cv::namedWindow("car detect", CV_WINDOW_NORMAL);
    //cv::namedWindow("background", CV_WINDOW_NORMAL);
    //cv::namedWindow("diff", CV_WINDOW_NORMAL);

    cv::Mat frame;

    // init the background with some frames
    for( unsigned i=0; i<fps; i++ ) {
        cap >> frame;
        detector.background.add(frame);
    }


    while( ! quit )
    {

        // always capture one frame. If fast forwarding (i.e. speedFactor>0)
        // then capture more. Update background each time.
        // cannot rely on cap.get(CV_CAP_PROP_POS_MSEC) to get the time....
        for( unsigned i=0; 1; i++ ) {
            cap >> frame;
            detector.background.add(frame);
            time += inv_fps;
            if( speedFactor<=0 || i>=pow(speedFactor,2) )
                break;
        }

        cv::Mat displayImg = frame.clone();

        detector.update(frame, time);

        // add timestamp
        //cout <<"Current camera time: " <<time <<endl;
        cv::putText(displayImg, format_time_frame(time), cv::Point(30,50),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 2,
                CV_RGB(0,255,555), 1, CV_AA);

        detector.display(displayImg);
        cv::imshow("car detect", displayImg);
        //cv::imshow("background", background.getImg());
        //cv::imshow("diff", detector.diffImg);



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
                cv::imwrite("images/" + t + "_background.png", detector.background.getImg());
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

