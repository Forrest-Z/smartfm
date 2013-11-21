#include "vehicle_detection.h"

vehicle_detection::vehicle_detection(ros::NodeHandle &n) : private_nh_("~"), n_(n), it_(n_)
{

	std::string model_name;
	private_nh_.param("car_model", model_name, std::string("car.xml"));
	private_nh_.param("overlap_threshold", overlap_threshold_, 0.3);
	private_nh_.param("threadNum", threadNum_, 4);

	std::vector<std::string> model_name_vector;
	model_name_vector.push_back(model_name);
	LatentSVMdetector_ = new LatentSvmDetector(model_name_vector);
    if( LatentSVMdetector_->empty() )
    {
        cout << "Models cann't be loaded" << endl;
        exit(-1);
    }

	generateColors( colors_, LatentSVMdetector_->getClassNames().size() );

    image_sub_ = it_.subscribe("/camera_front/image_raw", 1, &vehicle_detection::imageCallback, this);
    image_pub_ = it_.advertise("/camera_front/vehicle_detection", 1);
}

vehicle_detection::~vehicle_detection()
{

}

void vehicle_detection::imageCallback(const sensor_msgs::ImageConstPtr& image)
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
	Mat mat_img;
	resize(cv_image_->image, mat_img, Size(cv_image_->image.cols, cv_image_->image.rows), 0,0);

	detectAndDrawObjects(mat_img, *LatentSVMdetector_, colors_, overlap_threshold_, threadNum_);

	cv_image_->image = mat_img;
	image_pub_.publish(cv_image_->toImageMsg());
	//waitKey(30);
}

void vehicle_detection::detectAndDrawObjects( Mat& image, LatentSvmDetector& detector, const vector<Scalar>& colors, float overlapThreshold, int numThreads )
{
    vector<LatentSvmDetector::ObjectDetection> detections;

    TickMeter tm;
    tm.start();
    detector.detect( image, detections, overlapThreshold, numThreads);
    tm.stop();

    cout << "Detection time = " << tm.getTimeSec() << " sec" << endl;

    const vector<string> classNames = detector.getClassNames();
    CV_Assert( colors.size() == classNames.size() );

    for( size_t i = 0; i < detections.size(); i++ )
    {
        const LatentSvmDetector::ObjectDetection& od = detections[i];
        rectangle( image, od.rect, colors[od.classID], 3 );
    }
    // put text over the all rectangles
    for( size_t i = 0; i < detections.size(); i++ )
    {
        const LatentSvmDetector::ObjectDetection& od = detections[i];
        putText( image, classNames[od.classID], Point(od.rect.x+4,od.rect.y+13), FONT_HERSHEY_SIMPLEX, 0.55, colors[od.classID], 2 );
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_detection");
    ros::NodeHandle n;
    vehicle_detection vehicle_detector(n);
    ros::spin();
    return 0;
}

