#include <classifier_cpu.h>

namespace HOG_Classifier {

	HOGClassifier::HOGClassifier(ros::NodeHandle &n) : n_(n), it_(n_)
	{
		image_pub_ = it_.advertise("pedestrian_detector",1);
		image_sub_ = it_.subscribe("image_raw", 1, &HOGClassifier::imageCallback, this);
		people_rects_sub_ = n.subscribe("pr_vector", 1, &HOGClassifier::peopleRectsCallback, this);
		//initializing classifier
		detector = cv::HOGDescriptor::getDefaultPeopleDetector();
		cpu_hog.setSVMDetector(detector);
		cpu_hog.nlevels=18;
		started = false;
		//&cv_image = NULL;
	}
	HOGClassifier::~HOGClassifier(){}
	
	void HOGClassifier::peopleRectsCallback(people_detector::people_rects pr)
	{
		if(started)
		{
			std::cout<<"Inside people rect\n";
			cv::resize(img, img, cv::Size(640, 480));
			cv::vector<cv::Rect> found;
			
			//grab the latest frame available
			for(int i=0;i<pr.pr_vector.size();i++)
			{
				
				//std::cout<<pr.pr_vector[i].x<<pr.pr_vector[i].y<<pr.pr_vector[i].width<<pr.pr_vector[i].height<<"\n";
				cv::Rect roi(pr.pr_vector[i].scaled_x,pr.pr_vector[i].scaled_y,pr.pr_vector[i].scaled_width,pr.pr_vector[i].scaled_height);
				//img = img(roi);//cv::Rect(0,0,640,480));
				cv::Mat cpu_img(img(roi));
				double t = (double)cv::getTickCount();   
				cpu_hog.detectMultiScale(cpu_img, found, 0, cv::Size(8,8), cv::Size(0,0), 1.05, 1);
				
				t = (double)cv::getTickCount() - t;
				printf("Detection time = %gms\n", t*1000./cv::getTickFrequency());
				cv::Point offset(pr.pr_vector[i].scaled_x, pr.pr_vector[i].scaled_y);
				for( int j = 0; j < (int)found.size(); j++ )
				{
					cv::Rect r = found[j];
					
					cv::rectangle(img, r.tl()+offset, r.br()+offset, cv::Scalar(0,255,0), 1);
				}
				cv::rectangle(img, offset, offset+cv::Point(pr.pr_vector[i].scaled_width, pr.pr_vector[i].scaled_height), cv::Scalar(255,0,0),1);
				//IplImage *cv = &frame;
				
			}
			try
				{
					image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8"));
				}
				catch (sensor_msgs::CvBridgeException error)
				{
					ROS_ERROR("error");
				}
		}/*
		if(started){
			
			std::cout<<"Inside people rect\n";
			cv::resize(img, img, cv::Size(640, 480));
			std::cout<<"Inside people rect0\n";
			cv::vector<cv::Rect> found;
			std::cout<<"Inside people rect0b\n";
			//cv::gpu::GpuMat gpu_img(img);
			double t = (double)cv::getTickCount();   
			std::cout<<"Inside people rect1\n";
			cpu_hog.detectMultiScale(img, found, 0, cv::Size(8,8), cv::Size(0,0), 1.05, 1);
			t = (double)cv::getTickCount() - t;
			printf("Detection time = %gms\n", t*1000./cv::getTickFrequency());
			for( int i = 0; i < (int)found.size(); i++ )
			{
				cv::Rect r = found[i];
				cv::rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 1);
				std::cout<<"Inside people rect1"<<i<<"\n";
			}
			std::cout<<"Inside people rect2\n";
			//IplImage *cv = &frame;
			try
			{
				image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8"));
			}
			catch (sensor_msgs::CvBridgeException error)
			{
				ROS_ERROR("error");
			}
		}*/
	}
		

	void HOGClassifier::imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
	{
		std::cout<<"Inside image call\n";
		
		try
		{
			cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("error");
		}
		cv::Mat l_img(cv_image);
		img = l_img;
		started=true;
					

	}	
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "HOGClassifier");
  ros::NodeHandle n;
  HOG_Classifier::HOGClassifier ic(n);
  ros::spin();
	return 0;
}
