#include "feature_points.h"

feature_points::feature_points(ros::NodeHandle &n) : private_nh_("~"), n_(n), it_(n_)
{
    image_sub_ = it_.subscribe("/camera_front/image_raw", 1, &feature_points::imageCallback, this);
    image_pub_ = it_.advertise("/camera_front/image_opticalflow", 1);
}
feature_points::~feature_points()
{
	 destroyAllWindows();
}

void feature_points::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
	//cv_bridge::CvImagePtr cv_image;
	try
	{
		cv_image_ = cv_bridge::toCvCopy(image, "bgr8");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	cv_image_->image.copyTo(frame1_rgb_);
	cvtColor(frame1_rgb_, frame1_, CV_BGR2GRAY);

	if(frame0_.empty()==true)
	{
		frame1_.copyTo(frame0_);
		frame1_rgb_.copyTo(frame0_rgb_);
	}
	else
	{
		int minHessian = 100;
		SiftFeatureDetector detector( minHessian );
		std::vector<KeyPoint> keypoints_0, keypoints_1;
		detector.detect( frame0_, keypoints_0 );
		detector.detect( frame1_, keypoints_1 );

		SiftDescriptorExtractor extractor;
		Mat descriptors_0, descriptors_1;
		extractor.compute( frame0_, keypoints_0, descriptors_0 );
		extractor.compute( frame1_, keypoints_1, descriptors_1 );

		FlannBasedMatcher matcher;
		std::vector< DMatch > matches;
		matcher.match( descriptors_0, descriptors_1, matches );

		double max_dist = 0; double min_dist = 100;
		//-- Quick calculation of max and min distances between keypoints
		for( int i = 0; i < descriptors_0.rows; i++ )
		{
			double dist = matches[i].distance;
			if( dist < min_dist ) min_dist = dist;
			if( dist > max_dist ) max_dist = dist;
		}

		std::vector< DMatch > good_matches;

		for( int i = 0; i < descriptors_0.rows; i++ )
		{
			if( matches[i].distance <= 10*min_dist )
			{ good_matches.push_back( matches[i]); }
		}

		Mat img_matches;
		//drawKeypoints( frame0_rgb_, keypoints_0, img_matches, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		drawMatches( frame0_rgb_, keypoints_0, frame1_rgb_, keypoints_1,good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

		//-- Show detected matches
		imshow( "Good Matches", img_matches );
		waitKey(3);

		frame1_.copyTo(frame0_);
		frame1_rgb_.copyTo(frame0_rgb_);
	}
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "feature_points");
    ros::NodeHandle n;
    feature_points feature_points_node(n);
    ros::spin();
    return 0;
}

