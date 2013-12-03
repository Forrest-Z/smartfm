#include <cv.h>
#include <highgui.h>
#include <iostream> 
#include <fstream> 
using namespace std;
using namespace cv;

struct Data{
  double x;
  double y;
  double t;
};

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

int main(int argc, char** argv)
{
  Mat img_mat = imread(argv[1]);
  if(img_mat.rows == 0 || img_mat.cols == 0)
  {
    cout<<"Image is not loaded. Make sure parameter img_file is provided correctly"<<endl;
    return 0;
  }
  cout<<"Image "<<img_mat.rows<<"x"<<img_mat.cols<<" loaded."<<endl;
  
  ifstream fp_in;
  fp_in.open(argv[2], ios::in);
  vector<Data> datas;
  string data_str;
  int i=0;
  while(!(getline(fp_in, data_str)).eof()){
    if(i++ == 0) continue;
    vector<string> split_str = split(data_str, ',');
    Data data;
    data.t = atof(split_str[0].c_str());
    data.x = atof(split_str[1].c_str());
    data.y = atof(split_str[2].c_str());
    datas.push_back(data);
  } 
  fp_in.close();
  
  cout<<datas.size()<<" pts."<<endl;
  double res = 0.05;
  double x_offset = -100.000000;
  double y_offset = -173.600000;
  for(size_t i=0; i<datas.size(); i++){
    
    int img_y_ori = (datas[i].x-x_offset)/res;
    int img_x_ori = img_mat.rows -  (datas[i].y-y_offset)/res;
    for(int y = -3; y< 4; y++){
      for(int x = -3; x<4; x++){
	int img_y = img_y_ori + y;
	int img_x = img_x_ori + x;
	img_mat.data[img_mat.step[0]*img_x + img_mat.step[1]*img_y + 0] = 0;
	img_mat.data[img_mat.step[0]*img_x + img_mat.step[1]*img_y + 1] = 0;
	img_mat.data[img_mat.step[0]*img_x + img_mat.step[1]*img_y + 2] = 255;
      }
    }
  }
  imwrite(argv[3], img_mat);
  //imshow("read_img", img_mat);
  //waitKey(0);
  /*
  for (int i=0; i<img_mat.rows; i++)
  {
    for(int j=0; j<img_mat.cols; j++)
    {
      pcl::PointXYZRGB p;
      p.b=(int)img_mat.at<Vec3b>( i , j )[0];
      p.g=(int)img_mat.at<Vec3b>( i , j )[1];
      p.r=(int)img_mat.at<Vec3b>( i , j )[2];
      p.data[0]=(float)j*0.1;
      p.data[1]=(img_mat.rows-i)*0.1;
      p.data[2]=-0.02;
      if(p.b==111 && p.g==111 && p.r==111) continue;
      msg->points.push_back (p);
    }
  }*/
  return 0;
}
