#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <qrencode.h>
#include <zbar.h>
using namespace std;

cv::Mat getQRCode(string text){
  QRcode *qrcode = QRcode_encodeData(text.size(), (unsigned char *)text.c_str(), 0, QR_ECLEVEL_M);
//   cout<<"Version "<<qrcode->version<<" width "<<qrcode->width<<endl;
  int scale = 2;
  int img_size = qrcode->width*scale;
  cv::Mat qr_img(img_size, img_size, CV_8UC3);
  cv::Mat qr_img_pad(img_size+4, img_size+4, CV_8UC3, CV_RGB(255,255,255));
  cv::Vec3b black(0,0,0);
  cv::Vec3b white(255,255,255);
  for(int i=0; i<qrcode->width; i++){
    for(int j=0; j<qrcode->width; j++){
      unsigned char data = qrcode->data[j*qrcode->width+i];
      int x_init = i*scale;
      int y_init = j*scale;
      for(int x=x_init; x<x_init+scale; x++){
	for(int y=y_init; y<y_init+scale; y++){
	  if(data&1) qr_img.at<cv::Vec3b>(y, x) = black;
	  else qr_img.at<cv::Vec3b>(y, x) = white;
	}
      }
    }
  }
  cv::Mat roi = qr_img_pad(cv::Rect(2,2, img_size, img_size));
  qr_img.copyTo(roi);
  QRcode_free(qrcode);
  return qr_img_pad;
}

cv::Mat getQRCode(ros::Time time_now){
  stringstream ss;
  ss<<time_now.toNSec();
  return getQRCode(ss.str());
}

cv_bridge::CvImagePtr sensorMsgToMat(sensor_msgs::Image &msg){
  cv_bridge::CvImagePtr cv_image;
  try {
      cv_image = cv_bridge::toCvCopy(msg, "rgb8");
  }
  catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  return cv_image;
}

uint64_t retriveQrCode(cv::Mat &img_in){
  cv::Mat img;
  cvtColor(img_in, img, CV_RGB2GRAY);
  cv::resize(img, img, cv::Size(), 2, 2, cv::INTER_NEAREST);
  cv::imwrite("retrieved_img.png", img);
  zbar::ImageScanner scanner;  
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);  
  int width = img.cols;  
  int height = img.rows;  
  uchar *raw = (uchar *)img.data;  
   // wrap image data  
   zbar::Image image(width, height, "Y800", raw, width * height);  
   // scan the image for barcodes  
   scanner.scan(image);  
   // extract results  
   uint64_t symbol_int=0;
   for(zbar::Image::SymbolIterator symbol = image.symbol_begin();  
     symbol != image.symbol_end();  
     ++symbol) {
      std::istringstream ss(symbol->get_data());
      ss >> symbol_int;
//      cout << "decoded " << symbol->get_type_name()  
//         << " symbol \"" << symbol->get_data() << '"' <<" "<< endl;  
   }  
  return symbol_int;
}

cv::Mat addTimeStamp(ros::Time time_now)
{
  ROS_DEBUG("SensorMsgs to CV");
  cv::Mat image(10, 250, CV_8UC3, cvScalar(255,255,255));
  int epochSec = time_now.toSec();
  time_t raw_time = epochSec;
  struct tm* timeinfo;
  timeinfo = localtime(&raw_time);
  string time_str = asctime(timeinfo);
  stringstream ss;
  ss<<time_str<<" "<<(time_now.toSec() - epochSec)*1000;
  cv::putText(image, ss.str(), cv::Point(0, 10), 
	      cv::FONT_HERSHEY_PLAIN, 0.8,cvScalar(0,0,0), 1, 8);
  return image;
};