#include <people_detect_sensor.h>

namespace HOG_Classifier {

	HOGClassifier::HOGClassifier(ros::NodeHandle &n) : n_(n), it_(n_)
	{
		image_pub_ = it_.advertise("pedestrian_detector2",1);
		image_sub_ = it_.subscribe("image_raw", 1, &HOGClassifier::imageCallback, this);
		people_detect_pub_ = n.advertise<people_detector::people_rects>("pedestrian_detect2",1);
		sensor_pub_ = n.advertise<geometry_msgs::Point32>("pedestrian", 1000);
		//people_ver_pub_ = n.advertise<people_detector::verified_objs>("verified_objects",1);
		//initializing people service
		service_ = n.advertiseService("pedestrian_detector",  &HOGClassifier::peopleSrvSvr, this);
		//initializing classifier
		detector = cv::gpu::HOGDescriptor::getDefaultPeopleDetector();
		gpu_hog.setSVMDetector(detector);
		gpu_hog.nlevels=18;
		started = false;
		pedestrian_detected_=false;
		cvNamedWindow("Image window");
		//&cv_image = NULL;
	}
	HOGClassifier::~HOGClassifier(){}
	
	void HOGClassifier::people_detect()
	{
			std::cout<<"Inside people rect\n";
			int image_width = 640;
			int image_height = 480;
			cv::resize(img, img, cv::Size(image_width, image_height));
			cv::vector<cv::Rect> found;
			
			people_detector::people_rects detect_rects;
			people_detector::people_rect temp_rect;
		
				cv::gpu::GpuMat gpu_img(img);
				double t = (double)cv::getTickCount();   
				/*
				 gpu_hog.detectMultiScale(gpu_img, found, hit_threshold, win_stride, 
                                         Size(0, 0), scale, gr_threshold);*/
				gpu_hog.detectMultiScale(gpu_img, found, 0, cv::Size(8,8), cv::Size(0,0), 1.05, 1);
				
				t = (double)cv::getTickCount() - t;
				printf("Detection time = %gms\n", t*1000./cv::getTickFrequency());
				for( int j = 0; j < (int)found.size(); j++ )
				{
					cv::Rect r = found[j];
					cv::Point topleftPoint = r.tl();
					cv::Point bottomrightPoint = r.br();
					temp_rect.cvRect_x1=topleftPoint.x;temp_rect.cvRect_y1=topleftPoint.y;
					temp_rect.cvRect_x2=bottomrightPoint.x;temp_rect.cvRect_y2=bottomrightPoint.y;
					detect_rects.pr_vector.push_back(temp_rect);
				}
				geometry_msgs::Point32 p;
				if((int)found.size()>0) 
				{
					p.x = 255;
					pedestrian_detected_ = true;
				}
				else
				{
					p.x = 0;
					pedestrian_detected_ = false;
				}
				sensor_pub_.publish(p);
			
			people_detect_pub_.publish(detect_rects);
	}
	//establish service server
	bool HOGClassifier::peopleSrvSvr(pedestrian_detector::pedestrian::Request &req, pedestrian_detector::pedestrian::Response &res)
	{
		if(pedestrian_detected_ )
			res.people = 255;
		else res.people = 0;
		
		return true;
	}

	void HOGClassifier::imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
	{
		std::cout<<"Inside image call\n";
		
		try
		{
			cv_image = bridge_.imgMsgToCv(msg_ptr, "bgra8");
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("error");
		}
		cv::Mat l_img(cv_image);
		img = l_img;
		HOGClassifier::people_detect();
		

	}	
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "HOGClassifierCrossing");
  ros::NodeHandle n;
  HOG_Classifier::HOGClassifier ic(n);
  ros::spin();
	return 0;
}
