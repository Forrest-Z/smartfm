#include "conti_lane.h"

namespace golfcar_vision{
  
	conti_lane::conti_lane():
				private_nh_("~"),
				it_(nh_)
    { 
	  polygon_init_ = false;
	  fixedTf_inited_ = false;
	  ipm_para_init_ = false;

      string conti_lane_model_path, conti_lane_scale_path;
	  private_nh_.param("lane_model_path", conti_lane_model_path, std::string("/home/baoxing/workspace/data_and_model/scaled_20120726.model"));
	  private_nh_.param("lane_scale_path", conti_lane_scale_path, std::string("/home/baoxing/workspace/data_and_model/range_20120726"));
	  conti_lane_classifier_ = new golfcar_ml::svm_classifier(conti_lane_model_path, conti_lane_scale_path);
      image_sub_ = it_.subscribe("/camera_front/image_ipm", 1, &conti_lane::imageCallback, this);

      polygon_sub_ = nh_.subscribe("img_polygon", 10, &conti_lane::polygonCallback, this);
      private_nh_.param("scale", scale_, 30.0);
      private_nh_.param("extract_training_image", extract_training_image_, false);
      frame_serial_ = 0;

      lanes_pub_ = nh_.advertise<vision_lane_detection::lanes_info>("lanes", 10);
      lanes_ptcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("lanes_ptcloud",2);

	  lane_extractor_ = new ransac_lane();
	  mask_init_ = false;
    }

	void conti_lane::polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& polygon_in)
	{
		polygon_init_ = true;
		for(size_t i=0; i<4; i++) ipm_polygon_.push_back(cvPoint(polygon_in->polygon.points[i].x, polygon_in->polygon.points[i].y));
	}

    void conti_lane::imageCallback (const sensor_msgs::ImageConstPtr& msg)
    {
    	if(!polygon_init_) return;
        if(!fixedTf_inited_)
        {
            tf::StampedTransform transform;
            try
            {
                ros::Time acquisition_time = msg->header.stamp;
                ros::Duration timeout(5.0 / 30);
                tf_.waitForTransform(msg->header.frame_id, "base_link", acquisition_time, timeout);
                tf_.lookupTransform(msg->header.frame_id, "base_link", acquisition_time, transform);
                fixedTf_inited_ = true;
            }
			catch (tf::TransformException& ex)
			{
                ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
                return;
			}

            double camera_baselink_dis = transform.inverse().getOrigin().x();
            center_x_ = (RECT_P0_X + RECT_P2_X)/2.0 + camera_baselink_dis;
            center_y_ = 0.0;
        }
        if(!ipm_para_init_)
        {
        	ipm_para_init_ = true;
        	center_pix_x_ = msg->width/2;
        	center_pix_y_ = msg->height/2;
        }

    	//------------------------------------------------------------------------------------------------------------------------------
        //1. to find the contours from image;
        //------------------------------------------------------------------------------------------------------------------------------
    	IplImage* color_image, *binary_img;
		try
		{
			color_image = bridge_.imgMsgToCv(msg, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException& ex)
		{
			ROS_ERROR("Failed to convert image");
			return;
		}
		binary_img = cvCreateImage(cvGetSize(color_image),8,1);
		cvCvtColor(color_image, binary_img, CV_BGR2GRAY);
		Img_preproc(binary_img, binary_img);

		//2013-March
		if(!mask_init_)
		{
			mask_init_ = true;
			image_mask_ = cvCreateImage(cvGetSize(color_image),8,1);
			cvZero(image_mask_);
	        int ipm_height 		= image_mask_ -> height;
			int ipm_width  		= image_mask_ -> width;
			int ipm_step	 	= image_mask_ -> widthStep/sizeof(uchar);
			uchar * ipm_data 	= (uchar*)image_mask_ ->imageData;
			for(int ih=0; ih < ipm_height; ih++)
			{
				for(int iw=0; iw < ipm_width; iw++)
				{
					CvPoint tmppoint = cvPoint(iw, ih);

					if(pointInPolygon <CvPoint> (tmppoint,ipm_polygon_))
					{
						ipm_data[ih*ipm_step+iw]=255;
					}
				}
			}
		}
		cvAnd(image_mask_, binary_img, binary_img);

        CvSeq *contours = 0;            //"contours" is a list of contour sequences, which is the core of "image_proc";
        CvSeq *first_contour = 0;       //always keep one copy of the beginning of this list, for further usage;
        CvMemStorage *mem_contours; 
        mem_contours = cvCreateMemStorage(0);
        
        //CvContourScanner scanner = cvStartFindContours(Itand, mem_contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        CvContourScanner scanner = cvStartFindContours(binary_img, mem_contours, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
                
        //-------------------------------------------------------------------------------------------------------------------------------
        //2. to filter noise at the boundary, and noise too small or too big; to re-write
        //-------------------------------------------------------------------------------------------------------------------------------
        contours = filter_contours(scanner);
        first_contour = contours;

        ROS_INFO("1");
        //-------------------------------------------------------------------------------------------------------------------------------
        //3. classify each remained candidates; visualize the markers detected;
        //-------------------------------------------------------------------------------------------------------------------------------
        IplImage *contour_img = cvCreateImage(cvSize(binary_img->width,binary_img->height),IPL_DEPTH_8U, 3);
		IplImage *thining_img = cvCreateImage(cvSize(contour_img->width,contour_img->height),IPL_DEPTH_8U, 1);
		cvZero(thining_img);
		cvZero(contour_img);
        cvCvtColor(binary_img, contour_img, CV_GRAY2BGR);
        CvScalar ext_color;
        ROS_INFO("2");

        CvMoments cvm; 
        CvHuMoments cvHM;
        
        CvBox2D cvBox;
        CvMemStorage *mem_box;
        mem_box = cvCreateMemStorage(0);
        
        
		CvSeq *contour_poly;
		CvMemStorage *mem_poly;
		mem_poly = cvCreateMemStorage(0);

		vision_lane_detection::lanes_info lanes_inImg;
		lanes_inImg.header = msg->header;
		lanes_inImg.header.frame_id = "base_link";

        int contour_num_in_this_image = 0;   // to calculate the number of contour candidates in one image;
        int lane_contour_serial = 0;
        for (; contours != 0; contours = contours->h_next)
        {
            contour_num_in_this_image++;
            
            //This denotes how many pixels of one certain object contour;
            //ROS_DEBUG("total pixels %d", contours->total);
            ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 ); 

			//---------------------------------------------------------------------------------------------
			//--Extract features for marker classification;
			//---------------------------------------------------------------------------------------------
			//1st feature, weights and perimeter;
            cvContourMoments(contours, &cvm);
            double contour_weight = cvm.m00;
            double contour_perimeter = cvContourPerimeter(contours);
            //2nd feature, HuMoment;
            cvGetHuMoments(&cvm, &cvHM);
            //3rd feature, side lengths of bounding box;
            cvBox = cvMinAreaRect2(contours, mem_box);
            //4th feature: number of points after polygon approximation;
			contour_poly = cvApproxPoly( contours, sizeof(CvContour), mem_poly, CV_POLY_APPROX_DP, 2, 0 );
			int approxPtNum = int (contour_poly->total);

            int contour_class = classify_contour (contour_weight, contour_perimeter, cvHM, cvBox, approxPtNum);

            if(contour_class==-1){ROS_ERROR("NO CLASSIFICATION!!!");}
            else if(contour_class==1)
			{
            	cvDrawContours(contour_img, contours, ext_color, CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));
            	//ransac lanes;
            	lane_extractor_->multiple_lanes(contours, scale_, thining_img, contour_img, lane_contour_serial, lanes_inImg);
            	lane_contour_serial++;
			}
			else {}
		}
        cvShowImage("lane_contour_img",contour_img);
        cvShowImage("lane_thining_img",thining_img);

        lanes_pub_.publish(lanes_inImg);
		sensor_msgs::PointCloud lanes_ptcloud;
		std::vector<CvPoint2D32f> lanes_pt_inImg;
		for(unsigned int il=0; il<lanes_inImg.lanes.size(); il++)
		{
		  for(unsigned int ip=0; ip< lanes_inImg.lanes[il].points.size(); ip++)
		  {
			  CvPoint2D32f pttmp;
			  pttmp.x = lanes_inImg.lanes[il].points[ip].x;
			  pttmp.y = lanes_inImg.lanes[il].points[ip].y;
			  lanes_pt_inImg.push_back(pttmp);
		  }
		}
		IpmImage_to_pcl(lanes_pt_inImg, lanes_ptcloud);
		lanes_ptcloud.header.stamp = msg -> header.stamp;
		lanes_ptcloud.header.frame_id = "base_link";
		lanes_ptcloud_pub_.publish(lanes_ptcloud);

        //-----------------------------------------------------------------------------------------------
        //only use when to extract training pictures;
        //once entering this loop, meaning at least one contour exist, save image as training sets;
        //-----------------------------------------------------------------------------------------------

        if(contour_num_in_this_image>0) extract_training_image(binary_img);

        cvReleaseMemStorage(&mem_contours);
        cvReleaseMemStorage(&mem_box);
        cvReleaseMemStorage(&mem_poly);

        cvWaitKey(1);
        cvReleaseImage(&contour_img);
        cvReleaseImage(&thining_img);
    }

    void conti_lane::extract_training_image(IplImage* binary_img)
    {
        if(extract_training_image_)
        {
			stringstream  name_string;
			int stored_serial= frame_serial_/2;
			//pay attention, this path to the folder cannot be too long, or OpenCV will crash;
			name_string<<"/home/baoxing/images/frame"<<stored_serial<<".jpg";
			const char *output_name = name_string.str().c_str();

			if (!cvSaveImage(output_name, binary_img))
			{
				ROS_ERROR("Cannot save images");
				return;
			}
            frame_serial_++;
        }
    }


    //Use libsvm to classify each contour; input features are of type "CvHuMoments" and "CvBox2D";
    //More and better features may be used in the future;
    int conti_lane::classify_contour(double weight_input, double perimeter_input,
									 CvHuMoments &HM_input, CvBox2D &Box_input, int polyNum_input)
    {
        float boxAngle  =   Box_input.angle;
        float height    =   Box_input.size.height;
        float width     =   Box_input.size.width;
        ROS_DEBUG("%lf, %lf, %lf, %lf, %lf, %lf, %lf", HM_input.hu1, HM_input.hu2, HM_input.hu3, HM_input.hu4, HM_input.hu5, HM_input.hu6, HM_input.hu7);
        ROS_DEBUG("boxAngle, height, width: %lf, %lf, %lf", boxAngle, height, width);
        //Data scaling here enables svm to get better accuracy;

        //keep accordance with "data_formating";
        double long_side_scaled = (max(width, height))/100.0;
        double short_side_scaled = (min(width, height))/100.0;

        int vector_length = 12;
		double marker_feature_vector[vector_length];
		marker_feature_vector[0] = HM_input.hu1;
		marker_feature_vector[1] = HM_input.hu2;
		marker_feature_vector[2] = HM_input.hu3;
		marker_feature_vector[3] = HM_input.hu4;
		marker_feature_vector[4] = HM_input.hu5;
		marker_feature_vector[5] = HM_input.hu6;
		marker_feature_vector[6] = HM_input.hu7;
		marker_feature_vector[7] = short_side_scaled;
		marker_feature_vector[8] = long_side_scaled;
		marker_feature_vector[9] = weight_input;
		marker_feature_vector[10] = perimeter_input;
		marker_feature_vector[11] = polyNum_input;

		int class_label = -1;
		class_label = conti_lane_classifier_->classify_objects(marker_feature_vector, vector_length);
        return class_label;
    }
    
    CvSeq* conti_lane::filter_contours (CvContourScanner &scanner)
    {
        CvSeq* c;
        CvBox2D cvBox;
        CvMemStorage *mem_box;
        mem_box = cvCreateMemStorage(0);
        while((c=cvFindNextContour(scanner))!=NULL)
        {
            //1st criterion: perimeter should be long enough;
            double len_pixel = cvContourPerimeter(c);
            double len_meter = len_pixel/scale_;
            bool len_criterion = (len_meter > CONTOUR_PERIMETER_THRESH);

            //2nd criterion: long side should exceed certain threshold;
			cvBox = cvMinAreaRect2(c, mem_box);
			float height =  cvBox.size.height;
			float width  =  cvBox.size.width;
			float long_side = max(height, width);
			bool  long_side_criterion = long_side > LONG_SIDE_THRESH*scale_;

			bool contour_criteria = len_criterion && long_side_criterion;
            if(!contour_criteria) cvSubstituteContour(scanner, NULL);
        }
        CvSeq *contours = cvEndFindContours(&scanner);
        cvReleaseMemStorage(&mem_box);
        return contours;
    }

	void conti_lane::IpmImage_to_pcl(std::vector <CvPoint2D32f> & pts_image, sensor_msgs::PointCloud &pts_3d)
	{
		pts_3d.points.clear();
		geometry_msgs::Point32 pttmp;

		for(unsigned int i = 0; i< pts_image.size(); i++)
		{
			pttmp.x = center_x_ -(pts_image[i].y- center_pix_y_)/scale_;
			pttmp.y = center_y_ -(pts_image[i].x - center_pix_x_)/scale_;
			//for the purpose of visualization in RVIZ;
			pttmp.z = 0.5;
			pts_3d.points.push_back(pttmp);
		}
	}



	conti_lane::~conti_lane()
    {
    	//2013-March
    	if(mask_init_)cvReleaseImage(&image_mask_);
    }
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "conti_lane");
	 ros::NodeHandle n;
	 golfcar_vision::conti_lane conti_lane_node;
     ros::spin();
     return 0;
}
