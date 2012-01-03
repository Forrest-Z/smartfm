#include "classifier.h"

#define WIN_SIZE Size(48,96)

HOGClassifier::HOGClassifier(ros::NodeHandle &n) : n_(n), it_(n_)
{
    image_pub_ = it_.advertise("pedestrian_detector",1);
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &HOGClassifier::imageCallback, this);
    people_rects_sub_ = n.subscribe("pd_vision_batch", 1, &HOGClassifier::peopleRectsCallback, this);
    people_roi_pub_ = n.advertise<sensing_on_road::pedestrian_vision_batch>("veri_pd_vision", 1);
    people_detect_pub_ = n.advertise<sensing_on_road::pedestrian_vision_batch>("pedestrian_detect",1);
    //people_ver_pub_ = n.advertise<people_detector::verified_objs>("verified_objects",1);
    ros::NodeHandle nh("~");
    nh.param("hit_threshold", hit_threshold, 1.65);
    nh.param("scale", scale, 1.05);
    nh.param("group_threshold", group_threshold, 1.0);
    nh.param("norm_dist", norm_dist, 5.0);
    //First call to hog to ready the CUDA
     cv::gpu::HOGDescriptor g_hog;

    new_image = false;
    cout<<"Classifier started"<<endl;
    cout<<"Parameter: hit_threshold   "<<hit_threshold<<endl;
    cout<<"           scale           "<<scale<<endl;
    cout<<"           group_threshold "<<group_threshold<<endl;
    cout<<"           norm_dist       "<<norm_dist<<endl;
}
HOGClassifier::~HOGClassifier(){}

void HOGClassifier::peopleRectsCallback(sensing_on_road::pedestrian_vision_batch pr)
{

    if(new_image)
    {


        ROS_DEBUG("Inside people rect");

        sensing_on_road::pedestrian_vision_batch roi_rects;
        sensing_on_road::pedestrian_vision_batch detect_rects;
        sensing_on_road::pedestrian_vision temp_rect;

        for(unsigned int i=0;i<pr.pd_vector.size();i++)
        {

            int image_width = 640;
            int image_height = 360;
            //added for bayesian filter

            temp_rect.decision_flag = false;

            int img_x = pr.pd_vector[i].x;
            int img_y = pr.pd_vector[i].y;

            if(img_x > image_width || img_y > image_height) return;
            if(img_x<0)img_x=0;
            if(img_y<0)img_y=0;

            int img_width = ((pr.pd_vector[i].width+img_x)>image_width)?(image_width-img_x):pr.pd_vector[i].width;
            int img_height =  ((pr.pd_vector[i].height+img_y)>image_height)?(image_height-img_y):pr.pd_vector[i].height;
            ROS_DEBUG_STREAM(img_x<<" "<<img_y<<" "<<img_width<<" "<<img_height);
            Rect roi(img_x,img_y,img_width,img_height);

            Mat mat_img(img(roi));
            double ratio;
            ScaleWithDistanceRatio(&mat_img, pr.pd_vector[i].disz, norm_dist, Size(img_width, img_height),WIN_SIZE, &ratio);
            gpu::GpuMat gpu_img(mat_img);
            vector<Rect> found;
            Point offset(img_x, img_y);
            detectPedestrian(offset, ratio, gpu_img, &detect_rects);

            //Send ROI to visualizer
            temp_rect = pr.pd_vector[i];
            temp_rect.cvRect_x1=img_x;temp_rect.cvRect_y1=img_y;
            temp_rect.cvRect_x2=img_x+img_width;temp_rect.cvRect_y2=img_y+img_height;
            if((int)detect_rects.pd_vector.size()>0) temp_rect.decision_flag = true;
            roi_rects.pd_vector.push_back(temp_rect);


        }
        people_roi_pub_.publish(roi_rects);
        people_detect_pub_.publish(detect_rects);
        new_image = false;
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
    gpu_hog.detectMultiScale(gpu_img, found, hit_threshold, Size(8,8), Size(0,0), scale, group_threshold);

    t = (double)getTickCount() - t;
    ROS_DEBUG("Detection time = %gms", t*1000./getTickFrequency());

    for( int j = 0; j < (int)(found).size(); j++ )
    {
        sensing_on_road::pedestrian_vision temp_rect;
        Rect r = (found)[j];
        Point topleftPoint = Point(r.tl().x/ratio, r.tl().y/ratio)+offset;
        Point bottomrightPoint =  Point(r.br().x/ratio, r.br().y/ratio)+offset;
        temp_rect.cvRect_x1=topleftPoint.x;temp_rect.cvRect_y1=topleftPoint.y;
        temp_rect.cvRect_x2=bottomrightPoint.x;temp_rect.cvRect_y2=bottomrightPoint.y;
        detect_rects->pd_vector.push_back(temp_rect);
    }
}

void HOGClassifier::imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
    ROS_DEBUG("Inside image call");
    cv_bridge::CvImagePtr cv_image;
    try
    {
        cv_image = cv_bridge::toCvCopy(msg_ptr, "bgra8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    Mat l_img(cv_image->image);
    img = l_img;
    new_image = true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "HOGClassifier");
    ros::NodeHandle n;
    HOGClassifier ic(n);
    ros::spin();
    return 0;
}
