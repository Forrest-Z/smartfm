#include "ground_extraction.h"

ground_extracter::ground_extracter(ros::NodeHandle &n) : private_nh_("~"), n_(n), it_(n_)
{
    image_sub_ = it_.subscribe("/camera_front/image_raw", 1, &ground_extracter::imageCallback, this);
}
ground_extracter::~ground_extracter()
{
}

void ground_extracter::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
	cv_bridge::CvImagePtr cv_image;
	try
	{
		cv_image = cv_bridge::toCvCopy(image, "bgra8");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	Mat mat_img(cv_image->image);
	gpu::GpuMat gpu_img(mat_img);
	Mat seg_img;
	gpu::meanShiftSegmentation(gpu_img, seg_img, 10, 35, 50, TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 5, 1));
	imshow("a", seg_img);
	waitKey(30);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ground_extracter");
    ros::NodeHandle n;
    ground_extracter vehicle_detector(n);
    ros::spin();
    return 0;
}

