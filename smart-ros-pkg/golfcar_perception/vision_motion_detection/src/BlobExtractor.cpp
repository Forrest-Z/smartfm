#include <vision_motion_detection/BlobExtractor.h>

BlobExtractor::BlobExtractor()
: diff_thresh_(30),
  dilate_size_(5),
  erode_size_(5),
  blurring_size_(2),
  view_intermediate_images_(false)
{

}

void BlobExtractor::set_roi(Polygon p)
{
    roi_poly_ = p;
    setMask();
}

std::vector<Blob> BlobExtractor::extract(cv::Mat frame_in, cv::Mat background_in, double time)
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

void BlobExtractor::setMask()
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

    /*
    cv::imshow("bin mask", bin_mask_);
    cv::imshow("rgb mask", rgb_mask_);
    cv::waitKey(10);
    */
}

void BlobExtractor::diff(cv::Mat frame_in, cv::Mat background_in)
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

    // convert to gray scale and threshold
    cvtColor(tmp, tmp, CV_BGR2GRAY);
    cv::threshold(tmp, diffImg_, diff_thresh_, 255, CV_THRESH_BINARY);

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

void BlobExtractor::dilate(cv::Mat & img)
{
    //MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE
    int s = dilate_size_;
    if( s<=0 ) return;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 2*s + 1, 2*s+1 ),
                                       cv::Point( s, s ) );
    cv::dilate( img, img, element );
}

void BlobExtractor::erode(cv::Mat & img)
{
    //MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE
    int s = erode_size_;
    if( s<=0 ) return;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 2*s + 1, 2*s+1 ),
                                       cv::Point( s, s ) );
    cv::erode( img, img, element );
}

std::vector<Blob> BlobExtractor::extract(cv::Mat bin_input, double time)
{
    cv::Mat image;
    cv::bitwise_and(bin_input, bin_mask_, image);

    // Connected points
    std::vector< std::vector<cv::Point> > v;

    /* CV_RETR_EXTERNAL retrives only the extreme outer contours
     * CV_RETR_LIST retrieves all of the contours and puts them in the list
     * CV_RETR_CCOMP retrieves all of the contours and organizes them into a two-level hierarchy: on the top level are the external boundaries of the components, on the second level are the boundaries of the holes
     * CV_RETR_TREE retrieves all of the contours and reconstructs the full hierarchy of nested contours
     */
    /* CV_CHAIN_CODE outputs contours in the Freeman chain code. All other methods output polygons (sequences of vertices)
     * CV_CHAIN_APPROX_NONE translates all of the points from the chain code into points
     * CV_CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments and leaves only their end points
     * CV_CHAIN_APPROX_TC89_L1,CV_CHAIN_APPROX_TC89_KCOS applies one of the flavors of the Teh-Chin chain approximation algorithm.
     * CV_LINK_RUNS uses a completely different contour retrieval algorithm by linking horizontal segments of 1’s. Only the CV_RETR_LIST retrieval mode can be used with this method.
     */
    cv::findContours(image, v, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    std::vector<Blob> blobs;

    // Approximate each contour by a polygon
    for( unsigned i=0; i<v.size(); i++ )
    {
        Blob blob;
        cv::Mat vi(v[i]);
        //cv::approxPolyDP( vi, blob.contour, 10, true );
        blob.contour = v[i];
        cv::Moments m = cv::moments(vi);
        blob.centroid = cv::Point(m.m10/m.m00, m.m01/m.m00);
        blob.timestamp = time;
        blobs.push_back(blob);
    }

    return blobs;
}
