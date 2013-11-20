/*
 * reference: http://opencv-srf.blogspot.sg/2010/09/object-detection-using-color-seperation.html
 * Hue range:
    Orange  0-22
    Yellow 22- 38
    Green 38-75
    Blue 75-130
    Violet 130-160
    Red 160-179
 */

#include "color_tracking.h"

using namespace std;
using namespace cv;

char key;

IplImage* imgTracking;
int lastX = -1;
int lastY = -1;

//This function threshold the HSV image and create a binary image
IplImage* GetThresholdedImage(IplImage* imgHSV, CvScalar lowerBound, CvScalar upperBound){       
    IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
    //cvInRangeS(imgHSV, cvScalar(160,160,60), cvScalar(180,2556,256), imgThresh); 
    cvInRangeS(imgHSV, lowerBound, upperBound, imgThresh);
    return imgThresh;
}

CvPoint trackObject(IplImage* imgThresh){
    // Calculate the moments of 'imgThresh'
    CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
    cvMoments(imgThresh, moments, 1);
    double moment10 = cvGetSpatialMoment(moments, 1, 0);
    double moment01 = cvGetSpatialMoment(moments, 0, 1);
    double area = cvGetCentralMoment(moments, 0, 0);

    int posX,posY;
     // if the area<1000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
    if(area>1000){
        // calculate the position of the ball
        posX = moment10/area;
        posY = moment01/area;        
        /*
       if(lastX>=0 && lastY>=0 && posX>=0 && posY>=0)
        {
            // Draw a yellow line from the previous point to the current point
            cvLine(imgTracking, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,0,255), 4);
        }

         lastX = posX;
        lastY = posY;

      */
    }

     free(moments);
     
     return cvPoint(posX, posY);
}

int main( int argc, char *argv[] )
{
  cvNamedWindow("Camera_Output", 1);    //Create window
  cvNamedWindow("Object1", 1);    //Create window
  cvNamedWindow("Object2", 1);    //Create window
  
  CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);  //Capture using any camera connected to your system
  if(!capture){
    std::cout << "Capture failure!" << std::endl;
    return -1;
  }
  //Initial first frame
  IplImage* frame=0;
  CvPoint trackingPoint1 = cvPoint(0,0);
  CvPoint lastPoint1 = cvPoint(-1,-1);
  
  CvPoint trackingPoint2 = cvPoint(0,0);
  CvPoint lastPoint2 = cvPoint(-1,-1);
  
  frame = cvQueryFrame(capture);           
  if(!frame) return -1;
      
  imgTracking=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U, 3);
  cvZero(imgTracking); //covert the image, 'imgTracking' to black
  //end initialise
  
  while(1){ //Create infinte loop for live streaming

    IplImage* frame = cvQueryFrame(capture); //Create image frames from capture
    /* 
     * Input code here
     */
    //Clone and Smoothen raw input frame
    frame = cvCloneImage(frame);
    //cvSmooth(frame, frame, CV_GAUSSIAN,3,3); //smooth the original image using Gaussian kernel
    
    //Create HSV image and thresholding
    IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3); 
    cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
    
    //Do color 1
    IplImage* imgThresh1 = GetThresholdedImage(imgHSV,cvScalar(160,100,60), cvScalar(180,256,256));
    trackingPoint1 = trackObject(imgThresh1);	//track the possition of the object
    /*
    if(lastPoint1.x>0 && lastPoint1.y>0 && trackingPoint1.x>0 && trackingPoint1.y>0)
    {
	// Draw a line from the previous point to the current point
	cvLine(imgTracking, trackingPoint1, lastPoint1, cvScalar(0,0,255), 4);
    }
    cvAdd(frame, imgTracking, frame);	// Add the tracking image and the frame
    */
    
    //Do color 2
    IplImage* imgThresh2 = GetThresholdedImage(imgHSV,cvScalar(38,45,170), cvScalar(75,256,256));
    trackingPoint2 = trackObject(imgThresh2);	//track the possition of the object
    /*
    if(lastPoint2.x>0 && lastPoint2.y>0 && trackingPoint2.x>0 && trackingPoint2.y>0)
    {
	// Draw a line from the previous point to the current point
	cvLine(imgTracking, trackingPoint2, lastPoint2, cvScalar(0,255,0), 4);
    }
    cvAdd(frame, imgTracking, frame);	    // Add the tracking image and the frame
    */
    if(trackingPoint1.x>0 && trackingPoint1.y>0 && trackingPoint2.x>0 && trackingPoint2.y>0)
    {
	// Draw a line from the previous point to the current point
	cvLine(imgTracking, trackingPoint1, trackingPoint2, cvScalar(0,255,0), 1);
    }
    cvAdd(frame, imgTracking, frame);
    
    //Show result
    cvShowImage("Object1", imgThresh1);
    cvShowImage("Object2", imgThresh2);
    cvShowImage("Camera_Output", frame);   //Show image frames on created window
    
    //Keep lastPoint
    lastPoint1 = trackingPoint1;
    lastPoint2 = trackingPoint2;
    
    cvZero(imgTracking);
    //Clean up used images
    cvReleaseImage(&imgHSV);
    cvReleaseImage(&imgThresh1);
    cvReleaseImage(&imgThresh2);
    cvReleaseImage(&frame);
    
    //wait for exit signal
    key = cvWaitKey(10);     //Capture Keyboard stroke
    if (char(key) == 27){
	break;      //If you hit ESC key loop will break.
    }
  }
  
  //Destroy on exit
  cvDestroyAllWindows() ;
  cvReleaseCapture(&capture); //Release capture.
  //cvDestroyWindow("Camera_Output"); //Destroy Window
  //cvDestroyWindow("Object"); //Destroy Window

  return 0;
}