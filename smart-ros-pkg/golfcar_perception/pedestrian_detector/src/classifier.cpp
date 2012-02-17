#include "classifier.h"




HOGClassifier::HOGClassifier(ros::NodeHandle &n) : n_(n), it_(n_)
{
    image_pub_ = it_.advertise("pedestrian_detector",1);
    //image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &HOGClassifier::imageCallback, this);
    //people_rects_sub_ = n.subscribe("pd_vision_batch", 1, &HOGClassifier::peopleRectsCallback, this);
    people_roi_pub_ = n.advertise<sensing_on_road::pedestrian_vision_batch>("veri_pd_vision", 1);
    people_detect_pub_ = n.advertise<sensing_on_road::pedestrian_vision_batch>("pedestrian_detect",1);
    //people_ver_pub_ = n.advertise<people_detector::verified_objs>("verified_objects",1);

    //First call to hog to ready the CUDA
    cv::gpu::HOGDescriptor g_hog;

    people_rects_sub_.subscribe(n_, "pd_vision_batch", 10);
    image_sub_.subscribe(it_,"/usb_cam/image_raw",10);
    typedef sync_policies::ApproximateTime<feature_detection::clusters, sensor_msgs::Image> MySyncPolicy;
    //TimeSynchronizer<sensing_on_road::pedestrian_vision_batch, sensor_msgs::Image> sync(people_rects_sub_, image_sub_, 100);
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), people_rects_sub_, image_sub_);
    sync.registerCallback(boost::bind(&HOGClassifier::syncCallback,this, _1, _2));
    cout<<"Classifier started"<<endl;


    cv::namedWindow("processed_image");
    count=0;

    ros::spin();
}
HOGClassifier::~HOGClassifier(){}

bool sort_pv(sensing_on_road::pedestrian_vision const &a, sensing_on_road::pedestrian_vision const &b)
{
    return a.disz < b.disz;
}

sensing_on_road::pedestrian_vision_batch HOGClassifier::transformClusterToPedVisionBatch(const feature_detection::clustersConstPtr clusters_ptr)
{
    sensing_on_road::pedestrian_vision_batch ped_vision_batch_msg;
    ped_vision_batch_msg.header = clusters_ptr->header;

    geometry_msgs::PointStamped pt_src, pt_dest;
    pt_src.header = clusters_ptr->header;
    // For each
    for( unsigned i=0; i<clusters_ptr->clusters.size(); i++ )
    {
        // Convert to PointStamped
        pt_src.point.x = clusters_ptr->clusters[i].centroid.x;
        pt_src.point.y = clusters_ptr->clusters[i].centroid.y;
        pt_src.point.z = clusters_ptr->clusters[i].centroid.z;

        camera_project::CvRectangle rect;
        try {
            rect = camera_project.project(pt_src, clusters_ptr->clusters[i].width, 2.0);
        } catch( std::out_of_range & e ) {
            ROS_WARN("out of range: %s", e.what());
            continue;
        } catch( tf::TransformException & e ) {
            ROS_WARN("camera project tf error: %s", e.what());
            continue;
        }

        sensing_on_road::pedestrian_vision msg;
        msg.cluster = clusters_ptr->clusters[i];
        msg.decision_flag = false;
        msg.complete_flag = true;
        msg.disz = clusters_ptr->clusters[i].centroid.x;
        setRectMsg(rect, &msg);
        ped_vision_batch_msg.pd_vector.push_back(msg);
    }
    return ped_vision_batch_msg;
}
void HOGClassifier::syncCallback(const feature_detection::clustersConstPtr cl_ptr, const sensor_msgs::ImageConstPtr image)
{
    updateParameter();

    ROS_DEBUG("Inside people rect");
    int image_width = 640;
    int image_height = 360;
    sensing_on_road::pedestrian_vision_batch roi_rects;
    sensing_on_road::pedestrian_vision_batch detect_rects;
    sensing_on_road::pedestrian_vision temp_rect;
    sensing_on_road::pedestrian_vision_batch pr = transformClusterToPedVisionBatch(cl_ptr);


    sort(pr.pd_vector.begin(), pr.pd_vector.end(), sort_pv);
    //detect_rects.pd_vector = pr.pd_vector;
    roi_rects.header = pr.header;
    roi_rects.pd_vector = pr.pd_vector;
    detect_rects.header = pr.header;
    //get a deep copy
    Mat temp;
    Cv_helper::sensormsgsToCv(image, temp);
    cv::Mat img_clone(temp.clone());

    for(unsigned int i=0;i<pr.pd_vector.size();i++)
    {
        Size roi_size;
        Point roi_point;
        roi_rects.pd_vector[i].decision_flag = false;

        sensing_on_road::pedestrian_vision pv = pr.pd_vector[i];
        //if(!Cv_helper::fillRoiRectangle(Size(image_width, image_height), &roi_size, &roi_point, pv)) return;
        int img_width  = pv.width;//roi_size.width;
        int img_height = pv.height;//roi_size.height;
        int img_x = pv.x;//roi_point.x;
        int img_y = pv.y;//roi_point.y;
        ROS_DEBUG_STREAM(img_clone.cols<<" "<<img_clone.rows);
        ROS_DEBUG_STREAM(img_x<<" "<<img_y<<" "<<img_width<<" "<<img_height);
        ROS_DEBUG_STREAM(img_x+img_width<<" "<<img_y+img_height);
        Mat m = img_clone;
        Rect roi(img_x,img_y,img_width,img_height);

        //obtain the roi
        Mat mat_img(img_clone(roi));
        ROS_DEBUG("ROI obtained");
        double ratio; /// to scale image
        if(scaleWithDistance) ScaleWithDistanceRatio(&mat_img, pr.pd_vector[i].disz, norm_dist, Size(img_width, img_height),WIN_SIZE, &ratio);
        else ratio=1;
        gpu::GpuMat gpu_img(mat_img);
        vector<Rect> found;
        Point offset(img_x, img_y);
        ROS_DEBUG("ROI image size: x=%d y=%d",gpu_img.cols, gpu_img.rows);
        sensing_on_road::pedestrian_vision_batch detected_peds;
        if(gpu_img.cols>=WIN_SIZE.width && gpu_img.rows>=WIN_SIZE.height)
            detectPedestrian(offset, ratio, gpu_img, &detected_peds);

        /// Send ROI to visualizer

        /// Add confidence level
        temp_rect = pr.pd_vector[i];
        temp_rect.cvRect_x1=img_x;temp_rect.cvRect_y1=img_y;
        temp_rect.cvRect_x2=img_x+img_width-1;temp_rect.cvRect_y2=img_y+img_height-1;
        ROS_INFO("Vision detected pedestrian %d", detect_rects.pd_vector.size());
        if((int)detected_peds.pd_vector.size()>0) temp_rect.decision_flag = true;
        roi_rects.pd_vector[i] = temp_rect;



        //fill the roi with black
        if(black_front_roi)
        {
            cv::Mat Mask(img_clone.size(), CV_8UC1, cv::Scalar(0));
            cv::Mat MaskROI = Mask(roi);
            MaskROI = cv::Scalar(1);

            //Set the image to 0 in places where the mask is 1
            img_clone.setTo(cv::Scalar(0), Mask);
        }

        //add detected pedestrian for visualization
        detect_rects.pd_vector.insert(detect_rects.pd_vector.begin(), detected_peds.pd_vector.begin(), detected_peds.pd_vector.end());

        if(show_processed_image)
        {
            cv::imshow("processed_image", img_clone);
            cvWaitKey(3);
        }

    }
    people_roi_pub_.publish(roi_rects);
    people_detect_pub_.publish(detect_rects);

}

void HOGClassifier::updateParameter()
{
    ros::NodeHandle nh("~");
    double temp_double; bool temp_bool; string temp_str;
    parameter_changed=false;
    nh.param("hit_threshold", temp_double, 0.25); checkParamChanged(temp_double,hit_threshold);
    nh.param("group_threshold", temp_double, 1.0); checkParamChanged(temp_double,group_threshold);
    nh.param("scale", temp_double, 1.05); checkParamChanged(temp_double,scale);
    nh.param("norm_dist", temp_double, 12.0); checkParamChanged(temp_double, norm_dist);
    nh.param("show_processed_image", temp_bool , false); checkParamChanged(temp_bool, show_processed_image);
    nh.param("black_front_roi", temp_bool, true); checkParamChanged(temp_bool, black_front_roi);
    nh.param("write_image", temp_bool, false); checkParamChanged(temp_bool, write_image);
    nh.param("path", temp_str, string("")); checkParamChanged(temp_str, path);
    nh.param("scale_with_distance", scaleWithDistance, false);
    if(parameter_changed)
    {
        cout<<"Parameter: hit_threshold        "<<hit_threshold<<endl;
        cout<<"           group_threshold      "<<group_threshold<<endl;
        cout<<"           scale                "<<scale<<endl;
        cout<<"           norm_dist            "<<norm_dist<<endl;
        cout<<"           show_processed_image "<<show_processed_image<<endl;
        cout<<"           black_front_roi      "<<black_front_roi<<endl;
        cout<<"           write_image          "<<write_image<<endl;
        cout<<"           path: "<<path<<endl;
    }
}

template <class T>
void HOGClassifier::checkParamChanged(T &newParam, T &curParam)
{
    if(newParam!=curParam)
    {
        parameter_changed = true;
        curParam = newParam;
    }
}

void HOGClassifier::ScaleWithDistanceRatio(Mat *img, double disz, double norm_distance, Size img_size, Size smallest_size, double *ratio)
{
    *ratio=disz/norm_distance;
    //ratio = 1.0;
    double new_width = img_size.width * (*ratio);
    double new_height = img_size.height * (*ratio);

    double height_limit = smallest_size.height/new_height;
    double width_limit = smallest_size.width/new_width;
    if( height_limit>1 || width_limit>1 )
    {
        double corrective_ratio;
        if(height_limit>width_limit) corrective_ratio = height_limit;
        else corrective_ratio = width_limit;
        new_height *= corrective_ratio;
        new_width *= corrective_ratio;
    }
    Size norm_size(round(new_width),round(new_height));
    ROS_DEBUG_STREAM("Resized: "<<norm_size.height<<' '<<norm_size.width);
    resize(*img, *img, norm_size);
    if(write_image)
    {
        if(path.empty())
        {
            ROS_WARN("No path given, please specify under path parameter");
        }
        else
        {
            stringstream ss;
            ss<<path<<"/"<<count++<<".jpg";
            imwrite(ss.str(),*img);
            ROS_DEBUG_STREAM("Saved image path: "<<ss.str());
        }
    }
}

void HOGClassifier::detectPedestrian(Point offset, double ratio, gpu::GpuMat& gpu_img, sensing_on_road::pedestrian_vision_batch *detect_rects)
{
    double t = (double)getTickCount();
    bool gamma_corr=true;
    vector<Rect> found;
    cv::gpu::HOGDescriptor gpu_hog(WIN_SIZE, Size(16, 16), Size(8, 8), Size(8, 8), 9,
                                   cv::gpu::HOGDescriptor::DEFAULT_WIN_SIGMA, 0.2, gamma_corr,
                                   cv::gpu::HOGDescriptor::DEFAULT_NLEVELS);
    gpu_hog.setSVMDetector(gpu::HOGDescriptor::getPeopleDetector48x96());
    vector<Point> found_location;
    ROS_DEBUG("Start detection");
    if(scaleWithDistance)
    {
        gpu_hog.detect(gpu_img, found_location, hit_threshold, Size(8,8), Size(0,0));
        Size win_size = WIN_SIZE;
        for (size_t j = 0; j < found_location.size(); j++)
            found.push_back(Rect(Point2d((CvPoint)found_location[j]), win_size));
    }
    else
    {
        gpu_hog.detectMultiScale(gpu_img, found, hit_threshold, Size(8,8), Size(0,0), scale, group_threshold);
    }


    gpu_hog.nlevels = 1;
    /// does it give confidence level ?  

    t = (double)getTickCount() - t;
    ROS_DEBUG("Detection time = %gms", t*1000./getTickFrequency());
    detect_rects->pd_vector.clear();
    for( int j = 0; j < (int)(found).size(); j++ )
    {
        sensing_on_road::pedestrian_vision temp_rect;
        /// add the confidence level : float32 confidence

        Rect r = (found)[j];
        Point topleftPoint = Point(r.tl().x/ratio, r.tl().y/ratio)+offset;
        Point bottomrightPoint =  Point(r.br().x/ratio, r.br().y/ratio)+offset;
        temp_rect.cvRect_x1=topleftPoint.x;temp_rect.cvRect_y1=topleftPoint.y;
        temp_rect.cvRect_x2=bottomrightPoint.x;temp_rect.cvRect_y2=bottomrightPoint.y;
        detect_rects->pd_vector.push_back(temp_rect);
    }
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "HOGClassifier");
    ros::NodeHandle n;
    HOGClassifier ic(n);
    ros::spin();
    return 0;
}

