#include "optical_flow.h"

optical_flower::optical_flower(ros::NodeHandle &n) : private_nh_("~"), n_(n), it_(n_)
{
    image_sub_ = it_.subscribe("/camera_front/image_raw", 1, &optical_flower::imageCallback, this);
    image_pub_ = it_.advertise("/camera_front/image_opticalflow", 1);

    // Some global variables for the optical flow
    const int numLevels = 4;
    const float pyrScale = 0.5;
    const bool fastPyramids = true;
    const int winSize = 7;
    const int numIters = 10;
    const int polyN = 7; // 5 or 7
    const float polySigma = 2.4;
    const int flags = 0;
    const bool resize_img = false;
    const float rfactor = 2.0;

    dflow.numLevels = numLevels;
    dflow.pyrScale = pyrScale;
    dflow.fastPyramids = fastPyramids;
    dflow.winSize = winSize;
    dflow.numIters = numIters;
    dflow.polyN = polyN;
    dflow.polySigma = polySigma;

    namedWindow("Dense Flow",CV_WINDOW_NORMAL);
    namedWindow("Motion Flow",CV_WINDOW_NORMAL);
}
optical_flower::~optical_flower()
{
	 destroyAllWindows();
}

void optical_flower::imageCallback(const sensor_msgs::ImageConstPtr& image)
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
        // Upload images to the GPU
        frame1GPU.upload(frame1_);
        frame0GPU.upload(frame0_);

        // Do the dense optical flow
        dflow(frame0GPU,frame1GPU,uGPU,vGPU);

        uGPU.download(imgU);
        vGPU.download(imgV);

        Mat flow_rgb, motion_flow;
    	// Draw the optical flow results;
        frame1_rgb_.copyTo(flow_rgb);
        frame1_rgb_.copyTo(motion_flow);
    	drawColorField(imgU,imgV,flow_rgb);
    	drawMotionField(imgU,imgV,motion_flow,15,15,.0,1.0,CV_RGB(0,255,0));
        imshow("Dense Flow",flow_rgb);
        imshow("Motion Flow",motion_flow);
        waitKey(3);
    	//imshow("meanShiftSeg", seg_img);
    	cv_image_->image = motion_flow;
    	image_pub_.publish(cv_image_->toImageMsg());

		frame1_.copyTo(frame0_);
		frame1_rgb_.copyTo(frame0_rgb_);
	}
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "optical_flower");
    ros::NodeHandle n;
    optical_flower optical_flower_node(n);
    ros::spin();
    return 0;
}

