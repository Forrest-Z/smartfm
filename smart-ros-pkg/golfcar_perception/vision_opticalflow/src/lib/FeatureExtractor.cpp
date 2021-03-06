#include <vision_opticalflow/FeatureExtractor.h>

#define _USE_MATH_DEFINES
#include <math.h>

FeatureExtractor::FeatureExtractor()
: diff_thresh_(30),
  dilate_size_(5),
  erode_size_(5),
  blurring_size_(2),
  view_intermediate_images_(false) ///display images resulting from blurring, erosion, dilation, threshold, etc.
{
    //goodFeaturesToTrack
    MAX_CORNERS = 500;
    qualityLevel = 0.2;
    minDistance = 0.1; //0.1
    roi_start_pt = cv::Point(0,380); //cv::Point(0,200)
    roi_end_pt = cv::Point(960,480); //cv::Point(120,240)
    
    win_size = cv::Size(21,21);
    //Opticalflow
    maxLevel = 3;
    criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    flag = 1;
    minEigThreshold = 1e-2;
}

cv::Mat FeatureExtractor::getImg()
{
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

vision_opticalflow::Feature FeatureExtractor::extract(cv::Mat frame_in, cv::Mat background_in, double time)
{
    // Apply the ROI mask
    if( bin_mask_.empty() || rgb_mask_.empty() )
    {
        // first call, create a white mask (all ones)
        bin_mask_ = cv::Mat(frame_in.size(), CV_8U);
        rgb_mask_ = cv::Mat(frame_in.size(), CV_8UC3);
        setMask();
    }

    diff(frame_in, background_in);
    return extract(diffImg_, time);
}

void FeatureExtractor::setMask()
{
    // the mask is only defined once we know the dimension of the image, i.e.
    // after the image callback has been called at least once.
    if( bin_mask_.empty() || rgb_mask_.empty() )
    {
        //ROS_INFO("empty masks, skipping");
    }

    else if( roi_poly_.empty() )
    {
        //ROS_INFO("No ROI defined, use white masks");
        bin_mask_.setTo(255);
        rgb_mask_.setTo(cv::Scalar(255,255,255));
    }

    else
    {
        //ROS_INFO("Creating masks");
        rgb_mask_.setTo( cv::Scalar(0,0,0) );
        std::vector< std::vector<cv::Point> > polys;
        polys.push_back(roi_poly_);
        cv::fillPoly(rgb_mask_, polys, cv::Scalar(255,255,255), 8);
        cv::Mat tmp;
        cvtColor(rgb_mask_, tmp, CV_BGR2GRAY);
        cv::threshold(tmp, bin_mask_, 125, 255, CV_THRESH_BINARY);
    }

//     cv::imshow("bin mask", bin_mask_);
//     cv::imshow("rgb mask", rgb_mask_);
//     cv::waitKey(10);

}

void FeatureExtractor::diff(cv::Mat frame_in, cv::Mat background_in)
{
    cv::Mat frame, background;
    cv::bitwise_and(frame_in, rgb_mask_, frame);
    cv::bitwise_and(background_in, rgb_mask_, background);

    // first blur the input image and the background. This helps reducing
    // problems due to small camera motions (wind, vibration, auto focus, etc.)
    if( blurring_size_>0 ) {
        cv::Size s( 2*blurring_size_ + 1, 2*blurring_size_+1 );
        cv::GaussianBlur( frame, frame, s, 0 );
        cv::GaussianBlur( background, background, s, 0 );
    }

    if( view_intermediate_images_ )
        cv::imshow(ros::this_node::getName()+"/1blurred", frame);

    // compute difference between current image and background
    cv::Mat tmp;
    cv::absdiff(background, frame, tmp);

    // convert to gray scale
    cvtColor(tmp, diffImg_, CV_BGR2GRAY);

    if( view_intermediate_images_ )
        cv::imshow(ros::this_node::getName()+"/2diff", diffImg_);

    // dilate and erode to get rid of isolated pixels and holes

    dilate(diffImg_);
    if( view_intermediate_images_ )
        cv::imshow(ros::this_node::getName()+"/3dilate", diffImg_);

    erode(diffImg_);
    if( view_intermediate_images_ ) {
        cv::imshow(ros::this_node::getName()+"/4erode", diffImg_);
        cv::waitKey(10);
    }

}

void FeatureExtractor::dilate(cv::Mat & img)
{
    //MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE
    int s = dilate_size_;
    if( s<=0 ) return;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 2*s + 1, 2*s+1 ),
                                       cv::Point( s, s ) );
    cv::dilate( img, img, element );
}

void FeatureExtractor::erode(cv::Mat & img)
{
    //MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE
    int s = erode_size_;
    if( s<=0 ) return;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 2*s + 1, 2*s+1 ),
                                       cv::Point( s, s ) );
    cv::erode( img, img, element );
}

vision_opticalflow::Feature FeatureExtractor::extract(cv::Mat gray_input, double time)
{
//     cv::Mat gray_input_32F;
//     gray_input.convertTo(gray_input_32F,CV_32F);
    curr_frame_feature_.clear();
    /** Set roi_mask_ **/
    roi_mask_ = cv::Mat(gray_input.size(),CV_8U);
    roi_mask_.setTo(cv::Scalar(0,0,0));
    cv::rectangle(roi_mask_,roi_start_pt,roi_end_pt,cv::Scalar(255,255,255),-1,8);
//     std::cout << "elemSize: " << gray_input.elemSize() << " depth: " << gray_input.depth() << " channels :" << gray_input.channels() << std::endl;
    //Get good feature to track of the current frame, store it in curr_frame_feature_ vector (vector of type float)
//     cv::goodFeaturesToTrack(gray_input, curr_frame_feature_, MAX_CORNERS, qualityLevel, minDistance, roi_mask_);
    cv::goodFeaturesToTrack(gray_input, curr_frame_feature_, MAX_CORNERS, qualityLevel, minDistance);     //not using ROI

    if(prev_frame_.empty() || prev_frame_feature_.size() <= 0)
    {
        std::cout << "empty prev_frame" << std::endl;
        vision_opticalflow::Feature msg_out;
        prev_frame_ = gray_input.clone();
        prev_frame_feature_ = curr_frame_feature_;
        
        return msg_out;
    }else{
        //Do optical flow, return found_frame_feature_ :: feature from time n-1 found at time n
        //prev_frame_ and gray_input have to be 8 bit image, but prev_frame_feature_ and found_frame_feature_ have to be floating point
        calcOpticalFlowPyrLK(prev_frame_, gray_input, prev_frame_feature_, found_frame_feature_, opticalFlow_status_, err_, win_size, maxLevel, criteria, flag, minEigThreshold);

        prev_frame_ = gray_input.clone();

//         std::cout << "prev_frame_feature_ size: " << prev_frame_feature_.size() << std::endl;
//         for(unsigned int i = 0; i < prev_frame_feature_.size(); i++){
//             std::cout << " (" << (float)prev_frame_feature_[i].x << "," << (float)prev_frame_feature_[i].y << ") ,";
//         }
//         std::cout << std::endl;
//         
//         std::cout << "found_frame_feature_ size: " << found_frame_feature_.size() << std::endl;
//         for(unsigned int i = 0; i < found_frame_feature_.size(); i++){
//             std::cout << " (" << (float)found_frame_feature_[i].x << "," << (float)found_frame_feature_[i].y << ") ,";
//         }
//         std::cout << std::endl;
//         
//         std::cout << "opticalFlow_status_ size: " << opticalFlow_status_.size(); 
//         for(unsigned int i = 0; i < opticalFlow_status_.size(); i++){
//             if(opticalFlow_status_[i] == 0) std::cout << " ok";
//             if(opticalFlow_status_[i] == 1) std::cout << " miss";
//         }
//         std::cout << std::endl;
//         std::cout << "-------------------------------" << std::endl;

        for(unsigned int i = 0; i < opticalFlow_status_.size(); i++){
            if(opticalFlow_status_[i] != 0){
                //match is found, check for flow
                if(prev_frame_feature_[i].x == found_frame_feature_[i].x &&
                   prev_frame_feature_[i].y == found_frame_feature_[i].y
                ){
                    prev_frame_feature_.erase(prev_frame_feature_.begin() + i);
                    found_frame_feature_.erase(found_frame_feature_.begin() + i);
                }
            }else{
                //match not found
                prev_frame_feature_.erase(prev_frame_feature_.begin() + i);
                found_frame_feature_.erase(found_frame_feature_.begin() + i);
            }
            
        }

        std_msgs::Header dummy;
        vision_opticalflow::Feature msg_out = getFeatureToMsg(dummy);

        prev_frame_feature_ = found_frame_feature_;
        prev_frame_feature_.insert(prev_frame_feature_.end(),curr_frame_feature_.begin(),curr_frame_feature_.end());
//         prev_frame_feature_ = curr_frame_feature_;
        return msg_out;
    }
}

vision_opticalflow::Feature FeatureExtractor::getFeatureToMsg(std_msgs::Header header)
{
    geometry_msgs::Point feature_temp;
    geometry_msgs::Point feature_vel;
    vision_opticalflow::Feature feature_msg;
    float feature_speed_mag;
    float feature_speed_dir;
    
    feature_msg.header = header;
    for(unsigned int i = 0; i < prev_frame_feature_.size(); i++)
    {
        //position of previous frame feature
        feature_temp.x = prev_frame_feature_[i].x;
        feature_temp.y = prev_frame_feature_[i].y;
        feature_msg.prev_feature.push_back(feature_temp);

        //position of found frame feature
        feature_temp.x = found_frame_feature_[i].x;
        feature_temp.y = found_frame_feature_[i].y;
        feature_msg.found_feature.push_back(feature_temp);

        //velocity of each feature
        feature_vel.x = found_frame_feature_[i].x - prev_frame_feature_[i].x;
        feature_vel.y = found_frame_feature_[i].y - prev_frame_feature_[i].y;
        feature_msg.feature_vel.push_back(feature_vel);

        //velocity in magnitude
        feature_speed_mag = sqrt(pow(feature_vel.x,2) + pow(feature_vel.y,2));
        feature_msg.feature_speed_mag.push_back(feature_speed_mag);
        
        //direction of feature in radian
        feature_speed_dir = atan2(feature_vel.y,feature_vel.x);
        feature_msg.feature_speed_dir.push_back(feature_speed_dir);       
//         std::cout << "direction in radian: " << feature_speed_dir << std::endl;
        
//         direction of each feature, didn't use anymore
        if(feature_speed_dir <= M_PI/2 && feature_speed_dir >= -M_PI/2){
            feature_msg.direction.push_back(0);         //0 -- moving right, 1 -- moving left
        }else{
            feature_msg.direction.push_back(1);         //0 -- moving right, 1 -- moving left
        }
    }

    return feature_msg;
}