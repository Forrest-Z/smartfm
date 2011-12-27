#include <classifier.h>
using namespace std;
namespace HOG_Classifier {

	HOGClassifier::HOGClassifier(ros::NodeHandle &n) : n_(n), it_(n_)
	{
		image_pub_ = it_.advertise("pedestrian_detector",1);
		image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &HOGClassifier::imageCallback, this);
		people_rects_sub_ = n.subscribe("pd_vision_batch", 1, &HOGClassifier::peopleRectsCallback, this);
		people_roi_pub_ = n.advertise<sensing_on_road::pedestrian_vision_batch>("veri_pd_vision", 1);
		people_detect_pub_ = n.advertise<sensing_on_road::pedestrian_vision_batch>("pedestrian_detect",1);
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
	
	void HOGClassifier::peopleRectsCallback(sensing_on_road::pedestrian_vision_batch pr)
	{
		if(new_image)
		{
			std::cout<<"Inside people rect\n";
			int image_width = 640;
			int image_height = 360;
			cv::resize(img, img, cv::Size(image_width, image_height));
			cv::vector<cv::Rect> found;
			
			sensing_on_road::pedestrian_vision_batch roi_rects;
			sensing_on_road::pedestrian_vision_batch detect_rects;
			sensing_on_road::pedestrian_vision temp_rect;
			
			for(unsigned int i=0;i<pr.pd_vector.size();i++)
			{
				
				
				//added for bayesian filter
				
				temp_rect.decision_flag = false;
				
				int img_x = pr.pd_vector[i].x;//>image_width)?image_width:pr.pr_vector[i].x;
				int img_y = pr.pd_vector[i].y;//>image_height)?image_height:pr.pr_vector[i].y;
				
				if(img_x > image_width || img_y > image_height) return;
				if(img_x<0)img_x=0;
				if(img_y<0)img_y=0;
				
				int img_width = ((pr.pd_vector[i].width+img_x)>image_width)?(image_width-img_x):pr.pd_vector[i].width;
				int img_height =  ((pr.pd_vector[i].height+img_y)>image_height)?(image_height-img_y):pr.pd_vector[i].height;
				
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
				gpu_hog.detectMultiScale(gpu_img, found, 0.2, cv::Size(8,8), cv::Size(0,0), 2.4, 0);
				
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
					detect_rects.pd_vector.push_back(temp_rect);
				}
				cv::rectangle(img, offset, offset+cv::Point(img_width, img_height), cv::Scalar(255,0,0),1);
				temp_rect = pr.pd_vector[i];
				temp_rect.cvRect_x1=img_x;temp_rect.cvRect_y1=img_y;
				temp_rect.cvRect_x2=img_x+img_width;temp_rect.cvRect_y2=img_y+img_height;
				if((int)found.size()>0) temp_rect.decision_flag = true;
				
				roi_rects.pd_vector.push_back(temp_rect);
				
			}
			people_roi_pub_.publish(roi_rects);
			people_detect_pub_.publish(detect_rects);
			new_image = false;
		}
	}
		

	void HOGClassifier::imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
	{
		std::cout<<"Inside image call\n";
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
