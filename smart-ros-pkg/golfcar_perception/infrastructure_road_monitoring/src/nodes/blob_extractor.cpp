/**
 * Extract blobs from current frame and background image. Publishes extracted
 * blobs as an infrastructure_road_monitoring/Blobs message on topic "blobs".
 * Also publishes the diff image on topic "diff".
 * It is possible to control the output rate by skipping some frames by either
 * specifying the desired period (parameter "period") or the number of frames
 * to skip (parameter "skip").
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/server.h>
#include <infrastructure_road_monitoring/BlobExtractorConfig.h>

#include <infrastructure_road_monitoring/Blobs.h>

#include "data_msg_conv.h"

using namespace std;

class BlobExtractorNode
{
public:
    BlobExtractorNode();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter frame_sub_, bgnd_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image
      > MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> synchronizer;

    ros::Publisher blobs_pub_;

    vector<Blob> blobs_;

    /// extract blobs from the binary frame
    void extract(cv::Mat inFrameBin, double time);


    int skip_frames_, skip_frames_count_;
    double period_, last_time_;

    unsigned diff_thresh_; ///< threshold used when converting diff image to binary
    int dilate_size_; ///< how much dilation should be applied
    int erode_size_;  ///< how much erosion should be applied
    int blurring_size_; ///< size of the blurring kernel
    cv::Mat diffImg_; ///< the resulting binary image.

    bool view_intermediate_images_; ///< display images resulting from blurring, erosion, dilation, threshold, etc.

    void diff(cv::Mat frame, cv::Mat background);
    void dilate(cv::Mat &);
	void erode(cv::Mat  &);

    dynamic_reconfigure::Server<infrastructure_road_monitoring::BlobExtractorConfig> server;

    void imgCallback(const sensor_msgs::Image::ConstPtr &  frame,
            const sensor_msgs::Image::ConstPtr &  background);
    void configCallback(infrastructure_road_monitoring::BlobExtractorConfig & config, uint32_t level);
};


BlobExtractorNode::BlobExtractorNode()
: it_(nh_),
  frame_sub_(it_, "camera", 20),
  bgnd_sub_(it_, "background", 20),
  synchronizer(MySyncPolicy(20), frame_sub_, bgnd_sub_),
  skip_frames_(0),
  skip_frames_count_(0),
  period_(0.0),
  last_time_(0.0),
  diff_thresh_(30),
  dilate_size_(5),
  erode_size_(5),
  blurring_size_(2),
  view_intermediate_images_(false)
{
    synchronizer.registerCallback( boost::bind(&BlobExtractorNode::imgCallback, this, _1, _2) );

    blobs_pub_ = nh_.advertise<infrastructure_road_monitoring::Blobs>("blobs", 10);

    server.setCallback( boost::bind(&BlobExtractorNode::configCallback, this, _1, _2) );
}

void BlobExtractorNode::diff(cv::Mat frame, cv::Mat background)
{
	cv::Mat bf, bb; // blurred frame and background

	if( blurring_size_>0 ) {
		cv::Size s( 2*blurring_size_ + 1, 2*blurring_size_+1 );
		cv::GaussianBlur( frame, bf, s, 0 );
		cv::GaussianBlur( background, bb, s, 0 );
	} else {
		bf = frame;
		bb = background;
	}

	if( view_intermediate_images_ )
		cv::imshow(ros::this_node::getName()+"/1blurred", bf);

    // compute difference between current image and background
    cv::Mat tmp;
    cv::absdiff(bb, bf, tmp);

    cvtColor(tmp, tmp, CV_BGR2GRAY);

    // apply threshold to foreground image
    cv::threshold(tmp, diffImg_, diff_thresh_, 255, CV_THRESH_BINARY);

	if( view_intermediate_images_ )
		cv::imshow(ros::this_node::getName()+"/2diff", diffImg_);

    dilate(diffImg_);
	if( view_intermediate_images_ )
		cv::imshow(ros::this_node::getName()+"/3dilate", diffImg_);

    erode(diffImg_);
	if( view_intermediate_images_ ) {
		cv::imshow(ros::this_node::getName()+"/4erode", diffImg_);
		cv::waitKey(10);
	}

}

void BlobExtractorNode::dilate(cv::Mat & img)
{
    //MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE
    int s = dilate_size_;
    if( s<=0 ) return;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 2*s + 1, 2*s+1 ),
                                       cv::Point( s, s ) );
    cv::dilate( img, img, element );
}

void BlobExtractorNode::erode(cv::Mat & img)
{
    //MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE
    int s = erode_size_;
    if( s<=0 ) return;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 2*s + 1, 2*s+1 ),
                                       cv::Point( s, s ) );
    cv::erode( img, img, element );
}

void BlobExtractorNode::extract(cv::Mat bin_input, double time)
{
    blobs_.clear();

    // a temp mat is required, otherwise diffImg is no longer binary and
    // display is messed up.
    cv::Mat tmp = bin_input.clone();

    // Connected points
    vector<vector<cv::Point> > v;

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
    cv::findContours(tmp, v, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

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
        blobs_.push_back(blob);
    }
}

void BlobExtractorNode::imgCallback(const sensor_msgs::Image::ConstPtr & frame,
        const sensor_msgs::Image::ConstPtr & background)
{
	double t_now = frame->header.stamp.toSec();
	if( last_time_==0.0 ||
			(++skip_frames_count_ > skip_frames_ && t_now>last_time_+period_) )
	{
		//ROS_INFO("processing");
		skip_frames_count_ = 0;
		last_time_ = t_now;
	}
	else
	{
		//ROS_INFO("skipping");
		return;
	}

    cv_bridge::CvImageConstPtr cvImgFrame = cv_bridge::toCvShare(frame, "bgr8");
    cv_bridge::CvImageConstPtr cvImgBgnd = cv_bridge::toCvShare(background, "bgr8");

    diff(cvImgFrame->image, cvImgBgnd->image);
    extract(diffImg_, t_now);

    infrastructure_road_monitoring::Blobs msg;
    msg.header = frame->header;
    for(unsigned i=0; i<blobs_.size(); i++)
        msg.blobs.push_back( blobDataToMsg(blobs_[i]) );

    blobs_pub_.publish(msg);
}

void BlobExtractorNode::configCallback(infrastructure_road_monitoring::BlobExtractorConfig & config, uint32_t level)
{
    //ROS_INFO("diff_threshold=%d, dilate_size=%d, erode_size=%d",
    //        config.diff_threshold, config.dilate_size, config.erode_size);

    if( level&1 ) { skip_frames_ = config.skip; skip_frames_count_=0; }
    if( level&2 ) { period_=config.period; last_time_=0.0; }
    diff_thresh_ = config.diff_threshold;
    dilate_size_ = config.dilate_size;
    erode_size_ = config.erode_size;
    blurring_size_ = config.blurring_size;
    view_intermediate_images_ = config.debug_view;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_extractor");
    BlobExtractorNode node;
    ros::spin();
    return 0;
}
