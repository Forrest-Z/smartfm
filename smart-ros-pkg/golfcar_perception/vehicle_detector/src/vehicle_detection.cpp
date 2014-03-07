#include "vehicle_detection.h"

vehicle_detection::vehicle_detection(ros::NodeHandle &n) : private_nh_("~"), n_(n), it_(n_)
{

	std::string model_name;
	private_nh_.param("car_model", model_name, std::string("/home/baoxing/workspace/data_and_model/car.xml"));
	private_nh_.param("overlap_threshold", overlap_threshold_, 0.5);
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

    cam_sub_ = it_.subscribeCamera("/camera_front/image_raw", 1, &vehicle_detection::imageCallback, this);
    segpose_batch_sub_  = n_.subscribe("/segment_pose_batches", 1, &vehicle_detection::lidarMeas_callback, this);

    image_pub_ = it_.advertise("/camera_front/vehicle_detection", 1);

	private_nh_.param("laser_frame_id",     laser_frame_id_,    std::string("front_bottom_lidar"));
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
	private_nh_.param("camera_frame_id",    camera_frame_id_,     std::string("/camera_front_img"));
	camera_initialized_ = false;
}

vehicle_detection::~vehicle_detection()
{

}

void vehicle_detection::imageCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info_msg)
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

	if(!camera_initialized_){
		cam_model_.fromCameraInfo(info_msg);
		camera_initialized_ = true;
	}


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
        cout<<od.score<<"\t";
    }
    cout<<endl;
}

void vehicle_detection::lidarMeas_callback(const MODT::segment_pose_batches& batches)
{
	boost::recursive_mutex::scoped_lock l(configuration_mutex_);

	ROS_INFO("lidarMeas_callback size(): %ld", batches.clusters.size());

	if(!camera_initialized_)return;

	//1st: calculate centroids in the baselink frame;
	sensor_msgs::PointCloud cluster_centroids;
	cluster_centroids.header = batches.header;
	cluster_centroids.header.stamp = cv_image_->header.stamp;

	for(size_t i=0; i<batches.clusters.size(); i++)
	{
		geometry_msgs::Point32 centroid_point_tmp;
		for(size_t j=0; j<batches.clusters[i].segments.back().points.size(); j++)
		{
			centroid_point_tmp.x += batches.clusters[i].segments.back().points[j].x;
			centroid_point_tmp.y += batches.clusters[i].segments.back().points[j].y;
			centroid_point_tmp.z += batches.clusters[i].segments.back().points[j].z;
		}
		if(batches.clusters[i].segments.back().points.size()!=0)
		{
			centroid_point_tmp.x = centroid_point_tmp.x/(float)batches.clusters[i].segments.back().points.size();
			centroid_point_tmp.y = centroid_point_tmp.y/(float)batches.clusters[i].segments.back().points.size();
			centroid_point_tmp.z = centroid_point_tmp.z/(float)batches.clusters[i].segments.back().points.size();
		}
		cluster_centroids.points.push_back(centroid_point_tmp);
	}

	//2nd: construct ROIs for image verification;
	try{tf_.transformPointCloud(base_frame_id_, cluster_centroids, cluster_centroids);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}

	std::vector<Rect> image_ROIs;

	for(size_t i=0; i<cluster_centroids.points.size(); i++)
	{
		geometry_msgs::PointStamped right_lower, left_upper;
		left_upper.header = cluster_centroids.header;
		right_lower.header = cluster_centroids.header;

		if(cluster_centroids.points[i].x <= 0.0)
		{
			right_lower.point.x = cluster_centroids.points[i].x-3.0;
			right_lower.point.y = cluster_centroids.points[i].y-3.0;
			right_lower.point.z = 0;

			left_upper.point.x = cluster_centroids.points[i].x+3.0;
			left_upper.point.y = cluster_centroids.points[i].y+3.0;
			left_upper.point.z = 3.0;
		}
		else
		{
			right_lower.point.x = cluster_centroids.points[i].x + 3.0;
			right_lower.point.y = cluster_centroids.points[i].y - 3.0;
			right_lower.point.z = 0;

			left_upper.point.x = cluster_centroids.points[i].x + 3.0;
			left_upper.point.y = cluster_centroids.points[i].y + 3.0;
			left_upper.point.z = 3.0;
		}

		try{tf_.transformPoint(camera_frame_id_, right_lower, right_lower);}
		catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();continue;}
		try{tf_.transformPoint(camera_frame_id_, left_upper, left_upper);}
		catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();continue;}

		cv::Point3d right_lower_cv(right_lower.point.x, right_lower.point.y, right_lower.point.z);
		cv::Point2d right_lower_uv;
		cam_model_.project3dToPixel(right_lower_cv, right_lower_uv);
		cv::Point3d left_upper_cv(left_upper.point.x, left_upper.point.y, left_upper.point.z);
		cv::Point2d left_upper_uv;
		cam_model_.project3dToPixel(left_upper_cv, left_upper_uv);

		cout<<"ROI:"<<left_upper_uv.x<<","<<left_upper_uv.y<<","<<right_lower_uv.x<<","<<right_lower_uv.y<<endl;
		if(left_upper_uv.x<0) left_upper_uv.x = 0;
		if(left_upper_uv.y<0) left_upper_uv.y = 0;
		if(left_upper_uv.x > cv_image_->image.cols -1) {left_upper_uv.x = cv_image_->image.cols -1;continue;}
		if(left_upper_uv.y > cv_image_->image.rows -1) {left_upper_uv.y = cv_image_->image.rows -1;continue;}

		if(right_lower_uv.x<0) {right_lower_uv.x = 0;continue;}
		if(right_lower_uv.y<0) {right_lower_uv.y = 0;continue;}
		if(right_lower_uv.x > cv_image_->image.cols -1) right_lower_uv.x = cv_image_->image.cols -1;
		if(right_lower_uv.y > cv_image_->image.rows -1) right_lower_uv.y = cv_image_->image.rows -1;

		int width  = right_lower_uv.x-left_upper_uv.x;
		int height = right_lower_uv.y-left_upper_uv.y;

		if(width<=40||height<=40) {continue;}

		cout<<left_upper_uv.x<<","<<left_upper_uv.y<<","<<width<<","<<height<<endl;

		Rect roi(left_upper_uv.x, left_upper_uv.y, width, height);
		image_ROIs.push_back(roi);
	}

	Mat mat_img;
	resize(cv_image_->image, mat_img, Size(cv_image_->image.cols, cv_image_->image.rows), 0,0);
	if(image_ROIs.size()==0){ROS_INFO("NO roi"); return;}

	Rect roi_to_use = image_ROIs.front();
	cout<<"roi to use:"<<roi_to_use.x<<","<<roi_to_use.y<<","<<roi_to_use.width<<","<<roi_to_use.height<<endl;

	Mat imageROI = mat_img(roi_to_use);

	detectAndDrawObjects(imageROI, *LatentSVMdetector_, colors_, overlap_threshold_, threadNum_);

	cv::imshow("detection", imageROI);
	cv_image_->image = imageROI;
	image_pub_.publish(cv_image_->toImageMsg());
	waitKey(3);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_detection");
    ros::NodeHandle n;
    vehicle_detection vehicle_detector(n);
    ros::spin();
    return 0;
}

