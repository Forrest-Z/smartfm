#ifndef FEATUREEXTRACTOR_H_
#define FEATUREEXTRACTOR_H_

#include <opencv2/opencv.hpp>
#include <vision_opticalflow/Feature.h>
#include <vision_opticalflow/Polygon.h>

class FeatureExtractor
{
public:
    FeatureExtractor(); //class constructor
//     void goodFeatures(cv::Mat frame);
//     void opticalFlow();
//     void update_frame();

    cv::Mat getImg();
    vision_opticalflow::Feature getFeatureToMsg(std_msgs::Header header);

    unsigned diff_thresh_; ///< threshold used when converting diff image to binary
    int dilate_size_; ///< how much dilation should be applied
    int erode_size_;  ///< how much erosion should be applied
    int blurring_size_; ///< size of the blurring kernel

    bool view_intermediate_images_; ///< display images resulting from blurring, erosion, dilation, threshold, etc.

    /// Computes the difference between frame and background and extracts feature
    vision_opticalflow::Feature extract(cv::Mat frame_in, cv::Mat background_in, double time);
    
private:
    cv::Mat diffImg_; ///< the resulting grey-scale image.

    /// Compute the difference between frame and background (result goes to diffImg_)
    void diff(cv::Mat frame, cv::Mat background);

    void dilate(cv::Mat &);
    void erode(cv::Mat &);
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

//     cv::Mat curr_frame_;
    cv::Mat prev_frame_;
    cv::Mat disp_frame_;  //color image clone from input image

    std::vector<cv::Point2f> curr_frame_feature_;       //feature at time n
    std::vector<cv::Point2f> prev_frame_feature_;       //feature at time n-1
    std::vector<cv::Point2f> found_frame_feature_;      //feature from time n-1 found at time n

    /// extract blobs from the gray-scale frame
    vision_opticalflow::Feature extract(cv::Mat inFrameBin, double time);
    
    /// Region of interest defined as a polygon
    Polygon roi_poly_;
    
    /// Masks corresponding to the ROI
    cv::Mat bin_mask_, rgb_mask_;

    /// Creates the masks bin_mask_ and rgb_mask_ corresponding to roi_poly_
    void setMask();
};
  
#endif //FEATUREEXTRACTOR_H_