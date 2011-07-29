#include <classifier.h>
using namespace std;
namespace HOG_Classifier {

	HOGClassifier::HOGClassifier(ros::NodeHandle &n) : n_(n), it_(n_)
	{
		image_pub_ = it_.advertise("pedestrian_detector",1);
		image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &HOGClassifier::imageCallback, this);
		people_rects_sub_ = n.subscribe("pr_vector", 1, &HOGClassifier::peopleRectsCallback, this);
		people_roi_pub_ = n.advertise<people_detector::people_rects>("verified_objects", 1);
		people_detect_pub_ = n.advertise<people_detector::people_rects>("pedestrian_detect",1);
		//people_ver_pub_ = n.advertise<people_detector::verified_objs>("verified_objects",1);
		//initializing classifier
		detector = cv::gpu::HOGDescriptor::getDefaultPeopleDetector();
		gpu_hog.setSVMDetector(detector);
		gpu_hog.nlevels=16;
		new_image = false;
		cvNamedWindow("Image window");
		cout<<"Classifier started"<<endl;
		//&cv_image = NULL;
	}
	HOGClassifier::~HOGClassifier(){}
	
	void HOGClassifier::peopleRectsCallback(people_detector::people_rects pr)
	{
		if(new_image)
		{
			std::cout<<"Inside people rect\n";
			int image_width = 640;
			int image_height = 480;
			cv::resize(img, img, cv::Size(image_width, image_height));
			cv::vector<cv::Rect> found;
			
			people_detector::people_rects roi_rects;
			people_detector::people_rects detect_rects;
			people_detector::people_rect temp_rect;
			
			for(int i=0;i<(unsigned int)pr.pr_vector.size();i++)
			{
				
				
				//added for bayesian filter
				
				temp_rect.decision_flag = false;
				
				int img_x = pr.pr_vector[i].scaled_x;//>image_width)?image_width:pr.pr_vector[i].scaled_x;
				int img_y = pr.pr_vector[i].scaled_y;//>image_height)?image_height:pr.pr_vector[i].scaled_y;
				
				if(img_x > image_width || img_y > image_height) return;
				if(img_x<0)img_x=0;
				if(img_y<0)img_y=0;
				
				int img_width = ((pr.pr_vector[i].scaled_width+img_x)>image_width)?(image_width-img_x):pr.pr_vector[i].scaled_width;
				int img_height =  ((pr.pr_vector[i].scaled_height+img_y)>image_height)?(image_height-img_y):pr.pr_vector[i].scaled_height;
				
				//the minimum rectangle size that is accepted is 64x128
				if(img_width<64)
				{
					img_width = 64;
					//check if there is enough pixel to the right
					if(img_x + 64 > image_width) img_x = image_width - 64;
				}
				//the same with bottom pixels
				if(img_height<128)
				{
					img_height = 128;
					if(img_y + 128 > img_height) img_y = image_height - 128;
				}
				std::cout<<img_x<<" "<<img_y<<" "<<img_width<<" "<<img_height<<"\n";				
				cv::Rect roi(img_x,img_y,img_width,img_height);
				//img = img(roi);//cv::Rect(0,0,640,480));
				cv::gpu::GpuMat gpu_img(img(roi));
				double t = (double)cv::getTickCount();   
				gpu_hog.detectMultiScale(gpu_img, found, 0.2, cv::Size(8,8), cv::Size(0,0), 1.05, 1);
				
				t = (double)cv::getTickCount() - t;
				printf("Detection time = %gms\n", t*1000./cv::getTickFrequency());
				cv::Point offset(img_x, img_y);
				for( int j = 0; j < (int)found.size(); j++ )
				{
					cv::Rect r = found[j];
					cv::Point topleftPoint = r.tl()+offset;
					cv::Point bottomrightPoint = r.br()+offset;
					cv::rectangle(img,topleftPoint , bottomrightPoint, cv::Scalar(0,255,0), 1);
					temp_rect.cvRect_x1=topleftPoint.x;temp_rect.cvRect_y1=topleftPoint.y;
					temp_rect.cvRect_x2=bottomrightPoint.x;temp_rect.cvRect_y2=bottomrightPoint.y;
					detect_rects.pr_vector.push_back(temp_rect);
				}
				cv::rectangle(img, offset, offset+cv::Point(img_width, img_height), cv::Scalar(255,0,0),1);
				temp_rect = pr.pr_vector[i];
				temp_rect.cvRect_x1=img_x;temp_rect.cvRect_y1=img_y;
				temp_rect.cvRect_x2=img_x+img_width;temp_rect.cvRect_y2=img_y+img_height;
				if((int)found.size()>0) temp_rect.decision_flag = true;
				
				roi_rects.pr_vector.push_back(temp_rect);
				//IplImage *cv = &frame;
				
			}
			people_roi_pub_.publish(roi_rects);
			people_detect_pub_.publish(detect_rects);
			//imshow("Image window", img);
			//cvWaitKey(3);
			/*
			try
				{
					image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgra8"));
				}
			catch (sensor_msgs::CvBridgeException error)
				{
					ROS_ERROR("error");
				}*/
			new_image = false;
		}
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
		new_image = true;
		

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
