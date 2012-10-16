#include <vision_opticalflow/FeatureExtractor.h>

FeatureExtractor::FeatureExtractor() : changed_(true), alpha_target_(0.005), alpha_(1.0)
{
  MAX_CORNERS = 500;
  qualityLevel = 0.01;
  minDistance = 0.1;

  win_size = cv::Size(12,12);
  maxLevel = 3;
  criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
  flag = 1;
  minEigThreshold = 1e-2;
}

void FeatureExtractor::goodFeatures(cv::Mat frame)
{
    frame.convertTo(curr_frame_, CV_8U);
//     curr_frame_ = frame.clone();
    disp_frame_ = frame.clone();
    
    cv::cvtColor(curr_frame_, curr_frame_gray_, CV_BGR2GRAY);        //convert BGR to grayscale image
//     std::cout << "goodFeatures" <<std::endl;
    std::cout << curr_frame_gray_.depth() << "," << curr_frame_gray_.channels() <<std::endl;
    
    cv::goodFeaturesToTrack(curr_frame_gray_, curr_frame_feature_, MAX_CORNERS, qualityLevel, minDistance, cv::noArray());
//     std::cout << "goodFeatures: " << curr_frame_feature_.size() << std::endl;
}

void FeatureExtractor::opticalFlow()
{
//     cv::vector<cv::Point2f> found_frame_feature_;

    if(prev_frame_.empty() == true)
    {
        prev_frame_ = curr_frame_.clone();
        prev_frame_feature_ = curr_frame_feature_;
        return;
    }
//     prev_frame_gray_ = curr_frame_gray_.clone();
//     prev_frame_feature_ = curr_frame_feature_;
    cv::cvtColor(curr_frame_, curr_frame_gray_, CV_BGR2GRAY);
    cv::cvtColor(prev_frame_, prev_frame_gray_, CV_BGR2GRAY);
    
    calcOpticalFlowPyrLK(prev_frame_gray_, curr_frame_gray_, prev_frame_feature_, found_frame_feature_, opticalFlow_status_, err_, win_size, maxLevel, criteria, flag, minEigThreshold);
    prev_frame_ = curr_frame_.clone();
//     prev_frame_feature_ = curr_frame_feature_;
    

//     for(unsigned int i = 0; i < found_frame_feature_.size(); i++)
//     {
//         std::cout << "(" << prev_frame_feature_[i].x << " , " << prev_frame_feature_[i].y << ")  " << "(" << found_frame_feature_[i].x << " , " << found_frame_feature_[i].y << ")" << std::endl;
//     }
//     std::cout << "### ------------------------------ ###" << std::endl;

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
//     for(unsigned int i = 0; i < curr_frame_feature_.size(); i++)
//     {
//         cv::circle(disp_frame_, curr_frame_feature_[i], 2, cv::Scalar(0,255,0));
//     }
    
    for(unsigned int i = 0; i < found_frame_feature_.size(); i++)
    {
        if(opticalFlow_status_[i] == 0){
            continue;
        }else{
            std::cout << "(" << prev_frame_feature_[i].x << " , " << prev_frame_feature_[i].y << ")  " << "(" << found_frame_feature_[i].x << " , " << found_frame_feature_[i].y << ")" << std::endl;
            cv::line(disp_frame_, prev_frame_feature_[i], found_frame_feature_[i], cv::Scalar(0,255,0));
        }
    }
    
    prev_frame_feature_ = curr_frame_feature_;
//     prev_frame_feature_ = found_frame_feature_;
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
//     for(unsigned int i = 0; i < found_frame_feature_.size(); i++)
//     {
//         if(opticalFlow_status_[i] == 0){
//             continue;
//         }else{
//             cv::line(disp_frame_, prev_frame_feature_[i], found_frame_feature_[i], cv::Scalar(0,255,0));
//         }
//     }
    disp_frame_.convertTo(disp_frame_, CV_8U);
    return disp_frame_;
}