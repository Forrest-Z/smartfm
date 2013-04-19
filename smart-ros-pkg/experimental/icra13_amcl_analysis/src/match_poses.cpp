#include <fstream>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

struct PoseAndFrameIdx
{
  double x,y,t,dist;
  int frame_idx, score;
  int matching_frame_idx;
};

bool readFile(istream& file, PoseAndFrameIdx &pfi)
{
  if(!(file >> pfi.frame_idx )) return false;
  if(!(file >> pfi.x )) return false;
  if(!(file >> pfi.y )) return false;
  if(!(file >> pfi.t )) return false;
  return true;
}

bool readResult(istream& file, PoseAndFrameIdx &pfi)
{
  if(!(file >> pfi.frame_idx )) return false;
  if(!(file >> pfi.matching_frame_idx )) return false;
  if(!(file >> pfi.x)) return false;
  if(!(file >> pfi.y)) return false;
  if(!(file >> pfi.t)) return false;
  if(!(file >> pfi.dist)) return false;
  if(!(file >> pfi.score)) return false;
  return true;
}

void getData(istream& file, vector<PoseAndFrameIdx> &pfi)
{
  PoseAndFrameIdx pfi_temp;
  while(readFile(file, pfi_temp))
  {
    pfi.push_back(pfi_temp);
  }
  cout<<pfi.size()<<" data obtained"<<endl;
}

void getResult(istream& file, vector<PoseAndFrameIdx> &pfi)
{
  PoseAndFrameIdx pfi_temp;
   while(readResult(file, pfi_temp))
  {
    pfi.push_back(pfi_temp);
  }
  cout<<pfi.size()<<" data obtained"<<endl;
}

int main(int argc, char** argv)
{
    if(argc<3)
    {
      cout<<"command:"<<endl;
      cout<<"match_poses log1 log2"<<endl;
      cout<<"match_poses log1 log2 match_result"<<endl;
    }
    istream *data_in1=NULL, *data_in2=NULL;         // input for data points
		ifstream dataStreamSrc1, dataStreamSrc2;
		dataStreamSrc1.open(argv[1], ios::in);// open data file
    dataStreamSrc2.open(argv[2], ios::in);
		if (!dataStreamSrc1 || !dataStreamSrc2) {
			cerr << "Cannot open data file\n";
			exit(1);
		}
		data_in1 = &dataStreamSrc1;
    data_in2 = &dataStreamSrc2;

    vector<PoseAndFrameIdx> pfi1, pfi2, pfi_result;
		getData(*data_in1, pfi1);
    getData(*data_in2, pfi2);
    
    string folder1 = argv[1];
    folder1 = folder1.substr(0, folder1.find_last_of("/")+1);
    string folder2 = argv[2];
    folder2 = folder2.substr(0, folder2.find_last_of("/")+1);
    if(argc == 4)
    {
      istream *data_result;
      ifstream dataStreamResult;
      dataStreamResult.open(argv[3], ios::in);
      data_result = &dataStreamResult;
      getResult(*data_result, pfi_result);
    }
      cv::VideoWriter record("RobotVideo.avi", CV_FOURCC('D','I','V','X'), 30, cv::Size(960, 360), true);
    for(size_t i=0; i<pfi1.size(); i++)
    {
      PoseAndFrameIdx pfi_nearest;
      double best_distance=99;
      double theta_diff, dist_diff=99, x_diff, y_diff;
      pfi_nearest = pfi2[0];
      stringstream image_file1, image_file2;
      if(argc ==3)
      {
        for(size_t j=0; j<pfi2.size(); j++)
        {
          double x = pfi1[i].x - pfi2[j].x;
          double y = pfi1[i].y - pfi2[j].y;
          double t = pfi1[i].t - pfi2[j].t;
          double dist = x*x + y*y + t*t;
          if(best_distance > dist)
          {
            best_distance = dist;
            pfi_nearest = pfi2[j];
            theta_diff = fabs(t);
            dist_diff = x*x + y*y;
            x_diff = fabs(x);
            y_diff = fabs(y);
          }
        }
        image_file1<<folder1<<pfi1[i].frame_idx<<".png";
        image_file2<<folder2<<pfi_nearest.frame_idx<<".png";
      }
      else
      {
        image_file1<<folder1<<pfi_result[i].frame_idx<<".png";
        image_file2<<folder2<<pfi_result[i].matching_frame_idx<<".png";    
      }
      
      cv::Mat img1, img2, img_diff;
      img1 = cv::imread(image_file1.str(), CV_LOAD_IMAGE_GRAYSCALE);
      img2 = cv::imread(image_file2.str(), CV_LOAD_IMAGE_GRAYSCALE);
      cv::normalize(img1, img1, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
      cv::normalize(img2, img2, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );
      int blurring_size_ = 2;
      cv::Size s( 2*blurring_size_ + 1, 2*blurring_size_+1 );
      //cv::GaussianBlur( img1, img1, s, 0 );
      //cv::GaussianBlur( img2, img2, s, 0 );
      cv::absdiff(img1, img2, img_diff);
      int sum_error = cv::sum(img_diff)[0];
      if(argc == 3)
        cout <<pfi1[i].frame_idx <<" "<<pfi_nearest.frame_idx << " "<<x_diff<<" "<<y_diff<<" "<<theta_diff/M_PI*180<<" "<<sqrt(dist_diff) <<" "<<sum_error<<endl;
      else
        cout <<pfi_result[i].frame_idx<<" "<<pfi_result[i].matching_frame_idx<<" "<<pfi_result[i].dist<<" "<<pfi_result[i].t<<" "<<pfi_result[i].score<<endl;
      cv::resize(img1, img1, cv::Size(0,0), 0.5, 0.5);
      cv::resize(img2, img2, cv::Size(0,0), 0.5, 0.5);
      cv::imshow("img1", img1);
      cv::imshow("img2", img2);
//src.copyTo(dst(Rect(left, top, src.cols, src.rows));
      cv::Mat ext_img = cv::Mat::ones(360, 960, img1.type());
      img_diff = (255-img_diff);
      img_diff.copyTo(ext_img(cv::Rect(320,0,img_diff.cols, img_diff.rows)));
      img1.copyTo(ext_img(cv::Rect(0,0,img1.cols, img1.rows)));
      img2.copyTo(ext_img(cv::Rect(0,180,img2.cols, img2.rows)));
      cv::imshow("img_diff", ext_img);
      stringstream save_img;
      //save_img<<pfi_result[i].frame_idx<<"_"<<pfi_result[i].matching_frame_idx<<".png";
      save_img<<pfi1[i].frame_idx<<"_result.png";
//      cv::imwrite(save_img.str(), ext_img);
        record << ext_img; 
      cv::waitKey(1);
    }
    return 0;
}
