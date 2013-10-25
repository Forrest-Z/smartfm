#include "ground_extraction.h"

ground_extracter::ground_extracter(ros::NodeHandle &n) : private_nh_("~"), n_(n), it_(n_)
{
    image_sub_ = it_.subscribe("/camera_front/image_ipm", 1, &ground_extracter::imageCallback, this);
    image_pub_ = it_.advertise("/camera_front/image_seg", 1);
    edge_pub_ = it_.advertise("/camera_front/image_edge", 1);
}
ground_extracter::~ground_extracter()
{

}

void ground_extracter::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
	//cv_bridge::CvImagePtr cv_image;
	try
	{
		cv_image_ = cv_bridge::toCvCopy(image, "bgra8");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	Mat mat_img(cv_image_->image);
	gpu::GpuMat gpu_img(mat_img);
	Mat seg_img;
	gpu::meanShiftSegmentation(gpu_img, seg_img, 10, 35, 100, TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 5, 1));
	//imshow("meanShiftSeg", seg_img);
	cv_image_->image = seg_img;
	image_pub_.publish(cv_image_->toImageMsg());
	boundary_find(seg_img);
	//waitKey(30);
}

void ground_extracter::boundary_find(const Mat& segImg)
{
	Mat gray_img;
	cvtColor(segImg, gray_img, CV_RGB2GRAY);

	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	int thresh = 10;
	Canny( gray_img, canny_output, thresh, thresh*2, 3 );
	findContours( canny_output, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, Point(0, 0) );
	RNG rng(12345);
	Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++ )
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point() );
	}
	cv_image_->image = drawing;
	cv_image_->encoding = "bgr8";
	edge_pub_.publish(cv_image_->toImageMsg());
	//imshow( "Contours", drawing );
	//waitKey(30);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ground_extracter");
    ros::NodeHandle n;
    ground_extracter vehicle_detector(n);
    ros::spin();
    return 0;
}

