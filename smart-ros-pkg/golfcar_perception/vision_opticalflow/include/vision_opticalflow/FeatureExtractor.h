#ifndef FEATUREEXTRACTOR_H_
#define FEATUREEXTRACTOR_H_

#include <opencv2/opencv.hpp>
#include <vision_opticalflow/Feature.h>

class FeatureExtractor
{
    //class variables for goodFeatures()
    int MAX_CORNERS;
    double qualityLevel;
    double minDistance;
    cv::Mat mask_roi;
    cv::Point roi_start_pt;
    cv::Point roi_end_pt;
    
    //class variables for opticalFlow
    std::vector<uchar> opticalFlow_status_;
    std::vector<float> err_;
    cv::Size win_size;
    int maxLevel;
    cv::TermCriteria criteria;
    int flag;   //0 = OPTFLOW_USE_INITIAL_FLOW, 1 = OPTFLOW_LK_GET_MIN_EIGENVALS
    double minEigThreshold;

    cv::Mat curr_frame_;
    cv::Mat prev_frame_;
    cv::Mat disp_frame_;  //color image clone from input image

    std::vector<cv::Point2f> curr_frame_feature_;       //feature at time n
    std::vector<cv::Point2f> prev_frame_feature_;       //feature at time n-1
    std::vector<cv::Point2f> found_frame_feature_;      //feature from time n-1 found at time n
    
    bool changed_;
    double alpha_target_;
    double alpha_;
  
public:
    FeatureExtractor(); //class constructor
    void goodFeatures(cv::Mat frame);
    void opticalFlow();
    void update_frame();
    
    cv::Mat getImg();
    vision_opticalflow::Feature getFeatureToMsg(std_msgs::Header header);
    void set_alpha(double);
    void reset();
};
  
#endif //FEATUREEXTRACTOR_H_