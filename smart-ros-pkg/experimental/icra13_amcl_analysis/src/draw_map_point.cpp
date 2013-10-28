#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <fstream>
using namespace std;

struct Pose2d
{
  double x,y,t;
};

bool readFile(istream& file, Pose2d &p)
{
  if(!(file >> p.x )) return false;
  if(!(file >> p.y )) return false;
  return true;
}

void getData(istream& file, vector<Pose2d> &p)
{
  Pose2d p_t;
  while(readFile(file, p_t))
  {
    p.push_back(p_t);
  }
  cout<<p.size()<<" data obtained"<<endl;
}

int main(int argc, char** argv)
{
  double res = 0.05;
  cv::Mat map_img = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  istream *data_in1=NULL, *data_in2=NULL;         // input for data points
	ifstream dataStreamSrc1, dataStreamSrc2;
	dataStreamSrc1.open(argv[2], ios::in);// open data file
  dataStreamSrc2.open(argv[3], ios::in);
	if (!dataStreamSrc1 || !dataStreamSrc2) {
		cerr << "Cannot open data file\n";
		exit(1);
	}
	data_in1 = &dataStreamSrc1;
  data_in2 = &dataStreamSrc2;
  
  vector<vector<Pose2d> > poses;
  poses.resize(2);
  getData(*data_in1, poses[0]);
  getData(*data_in2, poses[1]);
  vector<cv::Scalar> colors;
  colors.push_back(cv::Scalar(255,0,0));
  colors.push_back(cv::Scalar(0,0,255));
  for(size_t i=0; i<poses.size(); i++)
  {
    cv::Point pre_pt(poses[i][0].x/res, map_img.rows - poses[i][0].y/res);
    for(size_t j=0; j<poses[i].size(); j++)
    {
      cv::Point pt_img(poses[i][j].x / res, map_img.rows - poses[i][j].y / res);
      cv::line(map_img, pre_pt, pt_img, colors[i], 1, CV_AA);
      pre_pt = pt_img;
    }
  }
  cv::imwrite("map_with_pts.png", map_img);
}
