#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

CascadeClassifier vehicle_cascade;
string window_name = "vehicle detection";

void detectAndDisplay( Mat frame );

int main(int argc, char** argv)
{
  CvCapture *capture;
  Mat frame;
  int input_resize_percent = 100;
  
  if( !vehicle_cascade.load( std::string(argv[1]))){ printf("--(!)Error loading\n"); return -1; };
   
  if(argc < 3)
  {
    std::cout << "Usage " << argv[0] << " cascade.xml video.avi" << std::endl;
    return 0;
  }

  capture = cvCaptureFromAVI(argv[2]);
  assert(capture);
  
  const int KEY_SPACE  = 32;
  const int KEY_ESC    = 27;

  int key = 0;
  do
  {
	 frame = cvQueryFrame( capture );
    if(frame.empty()) break;
    detectAndDisplay(frame); 

    key = waitKey(10);
    if(key == KEY_SPACE)
      key = waitKey(0);
    if(key == KEY_ESC)
      break;
  }while(1);

  cvDestroyAllWindows();
  return 0;
}

void detectAndDisplay( Mat frame )
{
	std::vector<Rect> faces;
	Mat frame_gray;
	cvtColor( frame, frame_gray, CV_BGR2GRAY );
	equalizeHist( frame_gray, frame_gray );
	
	std::cout <<"go to detect"<< std::endl;
	vehicle_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0, Size(100, 40) );
	
	for( int i = 0; i < faces.size(); i++ )
	{
	 Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
	 ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
	}
	
	//-- Show what you got
	imshow( window_name, frame );
}
