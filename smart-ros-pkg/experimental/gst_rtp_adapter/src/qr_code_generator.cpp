#include <ros/ros.h>
#include "addTimeStamp.h"

using namespace std;

int main(int argc, char** argv){
  string text = string(argv[1]);
  cv::Mat qr_img_pad = getQRCode(text);
  cv::imwrite("qr_img2.png", qr_img_pad);
  while(1){
    cv::imshow("qr_img", qr_img_pad);
    cvWaitKey(1);
  }
}