#include <vision_opticalflow/FeatureExtractor.h>

FeatureExtractor::FeatureExtractor() : changed_(true), alpha_target_(0.005), alpha_(1.0)
{
  MAX_CORNERS = 500;
  qualityLevel = 0.1;
  minDistance = 0.1;

  win_size = cv::Size(12,12);
  maxLevel = 3;
  criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
  flag = 1;
  minEigThreshold = 1e-2;
}

void FeatureExtractor::goodFeatures(cv::Mat frame)
{
    cv::Mat curr_frame_gray_;
    
    disp_frame_ = frame.clone();
    frame.convertTo(curr_frame_, CV_8U);
    
    cv::cvtColor(curr_frame_, curr_frame_gray_, CV_BGR2GRAY);

    //Get good feature to track of the current frame, store it in curr_frame_feature_ vector (vector of type float)
    cv::goodFeaturesToTrack(curr_frame_gray_, curr_frame_feature_, MAX_CORNERS, qualityLevel, minDistance, cv::noArray());
}

void FeatureExtractor::opticalFlow()
{
    if(prev_frame_.empty() == true)
    {
        // if no previous frame then skip
        return;
    }

    //Do optical flow, return found_frame_feature_ :: feature from time n-1 found at time n
    calcOpticalFlowPyrLK(prev_frame_, curr_frame_, prev_frame_feature_, found_frame_feature_, opticalFlow_status_, err_, win_size, maxLevel, criteria, flag, minEigThreshold);
}

void FeatureExtractor::update_frame()
{
    prev_frame_ = curr_frame_.clone();
    prev_frame_feature_ = curr_frame_feature_;
}

void FeatureExtractor::reset()
{
    cv::Mat empty;
    empty.copyTo(curr_frame_);
    empty.copyTo(prev_frame_);
    changed_ = true;
    alpha_ = 1.0;
}

void FeatureExtractor::set_alpha(double a)
{
    alpha_target_ = a;
}

cv::Mat FeatureExtractor::getImg()
{
//     for(unsigned int i = 0; i < prev_frame_feature_.size(); i++)
//     {
//         cv::circle(disp_frame_, prev_frame_feature_[i], 2, cv::Scalar(0,0,255));
//     }
// 
//     for(unsigned int i = 0; i < found_frame_feature_.size(); i++)
//     {
//         cv::circle(disp_frame_, found_frame_feature_[i], 2, cv::Scalar(255,0,0));
//     }
//     
    for(unsigned int i = 0; i < found_frame_feature_.size(); i++)
    {
        if(opticalFlow_status_[i] == 0){
            continue;
        }else{
            cv::line(disp_frame_, prev_frame_feature_[i], found_frame_feature_[i], cv::Scalar(0,255,0));
        }
    }
    
    disp_frame_.convertTo(disp_frame_, CV_8U);
    return disp_frame_;
}