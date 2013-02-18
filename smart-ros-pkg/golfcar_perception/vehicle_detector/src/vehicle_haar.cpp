#include "vehicle_haar.h"

HAARClassifier::HAARClassifier(ros::NodeHandle &n) : private_nh_("~"), n_(n), it_(n_)
{
    vehicle_roi_pub_ = n.advertise<sensor_msgs::PointCloud>("vehicle_roi", 1);
    polygon_pub_ = n.advertise<geometry_msgs::PolygonStamped>("vehicle_detect_visual",1);
    image_sub_ = it_.subscribe("/camera_front/image_raw", 1, &HAARClassifier::imageCallback, this);
    cout<<"Classifier started"<<endl;
    count=0;

    private_nh_.param("cascade_path", cascade_path, std::string("/home/baoxing/workspace/data_and_model/cars3.xml"));
    private_nh_.param("input_resize_percent", input_resize_percent, 100);
    cascade = (CvHaarClassifierCascade*) cvLoad(cascade_path.c_str(), 0, 0, 0);
    if(!cascade) cout<<"cannot initialize the cascade"<<endl;
    storage = cvCreateMemStorage(0);
    cvNamedWindow("vehicle_show", 1);
}
HAARClassifier::~HAARClassifier()
{
	cvDestroyWindow("vehicle_show");
	cvReleaseHaarClassifierCascade(&cascade);
	cvReleaseMemStorage(&storage);
}

void HAARClassifier::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    updateParameter();

    IplImage* color_image;
	try
	{
	   color_image = bridge_.imgMsgToCv(image, "bgr8");
	}
	catch (sensor_msgs::CvBridgeException& ex)
	{
	   ROS_ERROR("Failed to convert image");
	   return;
	}

	IplImage* frame = cvCreateImage(cvSize((int)((color_image->width*input_resize_percent)/100) , (int)((color_image->height*input_resize_percent)/100)), 8, 3);
	cvCopy(color_image, frame);

	std::vector<CvRect> vehicle_rects;
  	detectVehicle(frame, vehicle_rects);
	cvShowImage("vehicle_show", frame);

	cvWaitKey(3);
	cvReleaseImage(&frame);

    //to project into angular distance
    geometry_msgs::PolygonStamped polyStamped;
    sensor_msgs::PointCloud pc;
    geometry_msgs::Point32 p;
    polyStamped.header = image->header;
    pc.header = image->header;

    polyStamped.header.frame_id = "camera_front_base";
    pc.header.frame_id  = "camera_front_base";

    for(size_t i=0; i< vehicle_rects.size(); i++)
    {
        //draw the polygon
        p.x=p.y=p.z=0;
        polyStamped.polygon.points.push_back(p);
        p.x = 25;
        p.y = -tan(getAngularDistance(vehicle_rects[i].x))*25;
        polyStamped.polygon.points.push_back(p);
        pc.points.push_back(p);
        p.x=p.y=p.z=0;
        polyStamped.polygon.points.push_back(p);
        p.x = 25;
        p.y = -tan(getAngularDistance(vehicle_rects[i].x+vehicle_rects[i].width))*25;
        polyStamped.polygon.points.push_back(p);
        pc.points.push_back(p);
    }
    polygon_pub_.publish(polyStamped);
    vehicle_roi_pub_.publish(pc);
}

double HAARClassifier::getAngularDistance(double x)
{
    int image_width = 640;
    int image_height = 360;
    //this function translate x coordinate pixels from camera frame to angular distance in 3D space
    //center around the camera frame
    //here assume a perfectly calibrated camera and using simple atan model of the camera
    //the logitech c910 has fov of 70 degree
    double fov = 70.0;
    double multiplier = (image_width/2.0) / tan(fov/360*M_PI);

    double angular_dist = atan((x - image_width/2.0)/multiplier);

    ROS_INFO("Estimated angular dist %lf", angular_dist/M_PI*180);

    return angular_dist;
}

void HAARClassifier::updateParameter()
{
    ros::NodeHandle nh("~");
    parameter_changed=false;
    bool temp_bool;
    nh.param("show_processed_image", temp_bool , false); checkParamChanged(temp_bool, show_processed_image);
    if(parameter_changed)
    {
        cout<<"Parameter: show_processed_image"<<show_processed_image<<endl;
    }
}

template <class T>
void HAARClassifier::checkParamChanged(T &newParam, T &curParam)
{
    if(newParam!=curParam)
    {
        parameter_changed = true;
        curParam = newParam;
    }
}

void HAARClassifier::detectVehicle(IplImage* frame, vector<CvRect> & vehicle_rects)
{
	  CvSize img_size = cvGetSize(frame);
	  CvSeq *object = cvHaarDetectObjects(
		frame,
	    cascade,
	    storage,
	    1.1, //1.1,//1.5, //-------------------SCALE FACTOR
	    1, //2        //------------------MIN NEIGHBOURS
	    0, //CV_HAAR_DO_CANNY_PRUNING
	    cvSize(60,60),//cvSize( 30,30), // ------MINSIZE
	    img_size //cvSize(70,70)//cvSize(640,480)  //---------MAXSIZE
	    );

	  std::cout << "Total: " << object->total << " cars" << std::endl;
	  for(int i = 0 ; i < ( object ? object->total : 0 ) ; i++)
	  {
	    CvRect *r = (CvRect*)cvGetSeqElem(object, i);
	    vehicle_rects.push_back(*r);
	    cvRectangle(frame, cvPoint(r->x, r->y), cvPoint(r->x + r->width, r->y + r->height), CV_RGB(255, 0, 0), 2, 8, 0);
	  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "HAARClassifier");
    ros::NodeHandle n;
    HAARClassifier vehicle_detector(n);
    ros::spin();
    return 0;
}

