#include <visualizer.h>

namespace HOG_Classifier {

	HOGClassifier::HOGClassifier(ros::NodeHandle &n) : n_(n), it_(n_)
	{
		image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &HOGClassifier::imageCallback, this);
		people_roi_sub_ = n.subscribe("verified_objects", 1, &HOGClassifier::peopleRoiCallback, this);
		people_detect_sub_ = n.subscribe("pedestrian_detect", 1, &HOGClassifier::peopleDetectCallback, this);
		people_verified_sub_ = n.subscribe("pedestrian_verified", 1, &HOGClassifier::peopleVerifiedCallback, this);

		started = false;
		cvNamedWindow("Image window");
	}
	HOGClassifier::~HOGClassifier(){}
	void HOGClassifier::peopleVerifiedCallback(sensing_on_road::pedestrian_vision_batch pr)
	{
		if(started)
		{
			verified_rects_.pd_vector.clear();
			verified_rects_=pr;
		}
	}
	void HOGClassifier::peopleRoiCallback(sensing_on_road::pedestrian_vision_batch pr)
	{
		if(started)
		{
			roi_rects_.pd_vector.clear();
			//for(int i=0;i<roi_rects_.pd_vector.size();i++)
				//roi_rects_.pd_vector.push_back(pr);		
			roi_rects_=pr;
		}
	}
	void HOGClassifier::peopleDetectCallback(sensing_on_road::pedestrian_vision_batch pr)
	{
		if(started)
		{
			detect_rects_.pd_vector.clear();
			detect_rects_=pr;		
		}
	}	

	void HOGClassifier::imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
	{
		cv::Mat img;
        cv_bridge::CvImagePtr cv_image;
		try
		{
			cv_image = cv_bridge::toCvCopy(msg_ptr, "bgra8");
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
		cv::Mat l_img(cv_image->image);
		img = l_img;
		for(unsigned int i=0;i<detect_rects_.pd_vector.size();i++)
			cv::rectangle(img,cv::Point(detect_rects_.pd_vector[i].cvRect_x1, detect_rects_.pd_vector[i].cvRect_y1) , cv::Point(detect_rects_.pd_vector[i].cvRect_x2, detect_rects_.pd_vector[i].cvRect_y2), cv::Scalar(0,255,0), 1);
		for(unsigned int i=0;i<roi_rects_.pd_vector.size();i++)
			cv::rectangle(img,cv::Point(roi_rects_.pd_vector[i].cvRect_x1, roi_rects_.pd_vector[i].cvRect_y1) , cv::Point(roi_rects_.pd_vector[i].cvRect_x2, roi_rects_.pd_vector[i].cvRect_y2), cv::Scalar(255,0,0), 1);
		for(unsigned int i=0;i<verified_rects_.pd_vector.size();i++)
			cv::rectangle(img,cv::Point(verified_rects_.pd_vector[i].cvRect_x1, verified_rects_.pd_vector[i].cvRect_y1) , cv::Point(verified_rects_.pd_vector[i].cvRect_x2, verified_rects_.pd_vector[i].cvRect_y2), cv::Scalar(0,0,255), 5);
		started = true;
		imshow("Image window", img);
		cvWaitKey(3);

	}	
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "HOGClassifierVis");
  ros::NodeHandle n;
  HOG_Classifier::HOGClassifier ic(n);
  ros::spin();
	return 0;
}
