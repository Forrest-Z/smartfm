#include "vehicle_detection_blackcar.h"

vehicle_detection::vehicle_detection(ros::NodeHandle &n) : private_nh_("~"), n_(n), it_(n_)
{
	std::string car_model_name, bus_model_name;
	private_nh_.param("car_model", car_model_name, std::string("/home/baoxing/workspace/data_and_model/car.xml"));
	private_nh_.param("bus_model", bus_model_name, std::string("/home/baoxing/workspace/data_and_model/bus.xml"));
	private_nh_.param("overlap_threshold", overlap_threshold_, 0.5);
	private_nh_.param("threadNum", threadNum_, 4);

	std::vector<std::string> model_name_vector;
	model_name_vector.push_back(car_model_name);
	//model_name_vector.push_back(bus_model_name);

	LatentSVMdetector_ = new LatentSvmDetector(model_name_vector);
    if( LatentSVMdetector_->empty() )
    {
        cout << "Models cann't be loaded" << endl;
        exit(-1);
    }

	generateColors( colors_, LatentSVMdetector_->getClassNames().size() );

    cam_sub_ = it_.subscribeCamera("camera_front/image_raw", 1, &vehicle_detection::imageCallback, this);
    image_pub_ = it_.advertise("camera_front/vehicle_detection", 1);

	private_nh_.param("laser_frame_id",     laser_frame_id_,    std::string("front_bottom_lidar"));
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
	private_nh_.param("camera_frame_id",    camera_frame_id_,     std::string("camera_front_img"));
	camera_initialized_ = false;

	private_nh_.param("detection_threshold", detection_threshold_, -1.3);

	new_image_ = false;

	laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (n_, "front_bottom_scan", 100);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_, camera_frame_id_, 100);
	tf_filter_->registerCallback(boost::bind(&vehicle_detection::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.1));

	beam_pub_	= n_.advertise<geometry_msgs::PolygonStamped>("scan_beam", 2);
	endPt_pub_  = n_.advertise<sensor_msgs::PointCloud>("endPoints", 2);
	private_nh_.param("x_buff", x_buff_, 10);
	private_nh_.param("y_buff", y_buff_, 10);
	private_nh_.param("angle_aperture_thresh", angle_aperture_thresh_, 5.0);
	private_nh_.param("vehicle_width", vehicle_width_, 1.5);
	private_nh_.param("vehicle_height", vehicle_height_, 3.0);
	private_nh_.param("max_detection_range", max_detection_range_, 40.0);
}

vehicle_detection::~vehicle_detection()
{

}

void vehicle_detection::imageCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	//cv_bridge::CvImagePtr cv_image;
	try{cv_image_ = cv_bridge::toCvCopy(image, "bgr8");}
	catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());}
	cv_image_copy_ = cv_image_;
	new_image_ = true;

	if(!camera_initialized_)
	{
		cam_model_.fromCameraInfo(info_msg);
		camera_initialized_ = true;
	}
}

void vehicle_detection::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	boost::recursive_mutex::scoped_lock scl(configuration_mutex_);
	ROS_INFO("scan callback");

	if(!new_image_) return;
	new_image_ = false;
	if(!camera_initialized_)return;
	std::vector<Rect> object_ROIs;

	sensor_msgs::PointCloud baselink_cloud, camera_cloud;
	try{projector_.transformLaserScanToPointCloud(laser_frame_id_, *scan_in, baselink_cloud, tf_);}
	catch (tf::TransformException& e){ROS_WARN("LASER CANNOT TRANSFER INTO LASER FRAME CLOUD"); std::cout << e.what();return;}
	try{projector_.transformLaserScanToPointCloud(camera_frame_id_, *scan_in, camera_cloud, tf_);}
	catch (tf::TransformException& e){ROS_WARN("LASER CANNOT TRANSFER INTO CAMERA FRAME CLOUD"); std::cout << e.what();return;}

	//visualize the laser readings;
	geometry_msgs::PolygonStamped lidar_beams;
	lidar_beams.header = baselink_cloud.header;
	geometry_msgs::Point32 origin_pt, end_pt;
	origin_pt.x = 0.0; origin_pt.y = 0.0; origin_pt.z = 0.0;
	for(size_t i=0; i<baselink_cloud.points.size(); i++)
	{
		end_pt = baselink_cloud.points[i];
		lidar_beams.polygon.points.push_back(origin_pt);
		lidar_beams.polygon.points.push_back(end_pt);
	}
	lidar_beams.polygon.points.push_back(origin_pt);
	beam_pub_.publish(lidar_beams);
	endPt_pub_.publish(baselink_cloud);

	if(!camera_initialized_) {ROS_WARN("camera not initialized!");return;}

	//visualize the laser readings on image; calculate the ROIs of black vehicles;
	std::vector<Point> image_points;
	for(size_t j=0; j<camera_cloud.points.size(); j++)
	{
		//in case when points are projected at the behind of the camera;
		cv::Point3d pt3d_cv(camera_cloud.points[j].x, camera_cloud.points[j].y, camera_cloud.points[j].z);
		cv::Point2d ptImg_uv;
		cam_model_.project3dToPixel(pt3d_cv, ptImg_uv);

		if(camera_cloud.points[j].z<1.0) {continue;}

		Point point_tmp(ptImg_uv.x, ptImg_uv.y);
		circle(cv_image_copy_->image, point_tmp, 4, Scalar(0, 0, 255), -1);


		assert(camera_cloud.channels[1].name == "index");
		if(j>0)
		{
			int ID_in_scan = (int)camera_cloud.channels[1].values[j];
			int ID_in_scan_prev = (int)camera_cloud.channels[1].values[j-1];

			double angle_diff = (ID_in_scan - ID_in_scan_prev)*scan_in->angle_increment;

			if(angle_diff > 5.0*M_PI/180.0)
			{
				double angle_tmp = ID_in_scan*scan_in->angle_increment +scan_in->angle_min;
				double angle_prev_tmp = ID_in_scan_prev*scan_in->angle_increment +scan_in->angle_min;

				double dist_tmp = vehicle_width_/(tan(angle_tmp) - tan(angle_prev_tmp));
				geometry_msgs::PointStamped pt_baselink_tmp, pt_baselink_top_tmp;

				pt_baselink_tmp.header = baselink_cloud.header;
				pt_baselink_top_tmp.header = baselink_cloud.header;
 				pt_baselink_tmp.point.x = dist_tmp;
				pt_baselink_tmp.point.y = 0.0;
				pt_baselink_tmp.point.z = 0.0;
				pt_baselink_top_tmp.point.x = max_detection_range_;
				pt_baselink_top_tmp.point.y = 0.0;
				pt_baselink_top_tmp.point.z = vehicle_height_;

				try{tf_.transformPoint(camera_frame_id_, pt_baselink_tmp, pt_baselink_tmp);}
				catch (tf::TransformException& e){ROS_WARN("cannot transform back into camera frame"); std::cout << e.what(); continue;}
				try{tf_.transformPoint(camera_frame_id_, pt_baselink_top_tmp, pt_baselink_top_tmp);}
				catch (tf::TransformException& e){ROS_WARN("cannot transform back into camera frame"); std::cout << e.what(); continue;}

				cv::Point3d pt3d_center_tmp(pt_baselink_tmp.point.x, pt_baselink_tmp.point.y, pt_baselink_tmp.point.z);
				cv::Point3d pt3d_center_top_tmp(pt_baselink_top_tmp.point.x, pt_baselink_top_tmp.point.y, pt_baselink_top_tmp.point.z);
				cv::Point2d ptImg_center_tmp, ptImg_center_top_tmp;
				cam_model_.project3dToPixel(pt3d_center_tmp, ptImg_center_tmp);
				cam_model_.project3dToPixel(pt3d_center_top_tmp, ptImg_center_top_tmp);

				Point center_tmp(ptImg_center_tmp.x, ptImg_center_tmp.y);
				Point center_top_tmp(ptImg_center_top_tmp.x, ptImg_center_top_tmp.y);

				if(image_points.size()>0)
				{
					Point lower_left(point_tmp.x, center_tmp.y), upper_left(point_tmp.x, center_top_tmp.y);
					Point lower_right(image_points.back().x, center_tmp.y), upper_right(image_points.back().x, center_top_tmp.y);

					Rect black_ROI(upper_left.x - x_buff_, upper_left.y - y_buff_, (lower_right.x - upper_left.x)+ 2*x_buff_, (lower_right.y- upper_left.y)+ 2*y_buff_);
					//rectangle(cv_image_copy_->image, upper_left, lower_right, Scalar(0, 255, 255), 3);
					object_ROIs.push_back(black_ROI);
				}
			}
		}

		image_points.push_back(point_tmp);
	}

	std::vector<vehicle_ROI> filtered_ROIs;
	filterROIs(object_ROIs, filtered_ROIs);
	detectAndDrawObjects(filtered_ROIs, *LatentSVMdetector_, colors_, overlap_threshold_, threadNum_);
	image_pub_.publish(cv_image_copy_->toImageMsg());


}

void vehicle_detection::filterROIs(std::vector<Rect> & object_ROIs, std::vector<vehicle_ROI> & filtered_ROIs)
{
	for(size_t i=0; i<object_ROIs.size(); i++)
	{
		vehicle_ROI roi_tmp;
		roi_tmp.original_ROI = object_ROIs[i];

		Point upper_left, lower_right;
		upper_left.x = roi_tmp.original_ROI.x;
		upper_left.y = roi_tmp.original_ROI.y;
		lower_right.x = roi_tmp.original_ROI.x + roi_tmp.original_ROI.width;
		lower_right.y = roi_tmp.original_ROI.y + roi_tmp.original_ROI.height;

		if((upper_left.x >= cv_image_->image.cols && upper_left.y >=cv_image_->image.rows) || (lower_right.x <= 0 || lower_right.y <=0)) continue;

		if(upper_left.x <0) upper_left.x = 0;
		if(upper_left.y <0) upper_left.y = 0;
		if(lower_right.x>= cv_image_->image.cols ) lower_right.x = cv_image_->image.cols -1;
		if(lower_right.y>= cv_image_->image.rows ) lower_right.y = cv_image_->image.rows -1;
		roi_tmp.ROI = Rect(upper_left.x, upper_left.y, lower_right.x - upper_left.x, lower_right.y - upper_left.y);
		rectangle(cv_image_copy_->image, upper_left, lower_right, Scalar(255, 0, 0), 3);

		//here actually choose the shorter side of the two sides, even though it is named as long side (compared to the later cropped side, it is longer);
		roi_tmp.long_side = roi_tmp.ROI.height>roi_tmp.ROI.width? roi_tmp.ROI.height: roi_tmp.ROI.width;

		if(roi_tmp.ROI.height >= 40 && roi_tmp.ROI.width  >= 40) filtered_ROIs.push_back(roi_tmp);
	}
}

void vehicle_detection::detectAndDrawObjects(std::vector<vehicle_ROI> &filtered_ROIs, LatentSvmDetector& detector, const vector<Scalar>& colors, float overlapThreshold, int numThreads )
{
    vector<LatentSvmDetector::ObjectDetection> detections;

    TickMeter tm;
    tm.start();

    for(size_t i=0; i<filtered_ROIs.size(); i++)
	{
        const vector<string> classNames = detector.getClassNames();
        CV_Assert( colors.size() == classNames.size() );

        cout<<"filtered_ROIs:"<<filtered_ROIs[i].ROI.x<<","<<filtered_ROIs[i].ROI.y<<","<<filtered_ROIs[i].ROI.width<<","<<filtered_ROIs[i].ROI.height<<","<<filtered_ROIs[i].long_side<<endl;

        Mat imageROI = cv_image_copy_->image(filtered_ROIs[i].ROI);
    	detector.detect( imageROI, detections, filtered_ROIs[i].long_side, overlapThreshold, numThreads);

    	for( size_t j = 0; j < detections.size(); j++ )
        {
            const LatentSvmDetector::ObjectDetection& od = detections[j];
            if(od.score > detection_threshold_)
            {
            	Rect RectInFullPic(od.rect.x + filtered_ROIs[i].ROI.x, od.rect.y + filtered_ROIs[i].ROI.y, od.rect.width, od.rect.height);
            	rectangle( cv_image_copy_->image, RectInFullPic, colors[od.classID], 3 );
            	putText( cv_image_copy_->image, classNames[od.classID], Point(RectInFullPic.x+4,RectInFullPic.y+13), FONT_HERSHEY_SIMPLEX, 0.55, colors[od.classID], 2 );
            }
            cout<<"score:"<<od.score<<"\t";
        }
        cout<<endl;
	}

    tm.stop();
    cout << "Detection time = " << tm.getTimeSec() << " sec" << endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_detection");
    ros::NodeHandle n;
    vehicle_detection vehicle_detector(n);
    ros::spin();
    return 0;
}

