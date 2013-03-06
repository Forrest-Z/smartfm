#include "ped_crossing.h"

namespace golfcar_vision{
  
	ped_crossing::ped_crossing():
				private_nh_("~"),
				it_(nh_)
    { 
	  polygon_init_ = false;
	  fixedTf_inited_ = false;
	  ipm_para_init_ = false;

      string ped_crossing_model_path, ped_crossing_scale_path;
	  private_nh_.param("ped_crossing_model_path", ped_crossing_model_path, std::string("/home/baoxing/workspace/data_and_model/scaled_20120726.model"));
	  private_nh_.param("ped_crossing_scale_path", ped_crossing_scale_path, std::string("/home/baoxing/workspace/data_and_model/range_20120726"));
      ped_crossing_classifier_ = new golfcar_ml::svm_classifier(ped_crossing_model_path, ped_crossing_scale_path);
      image_sub_ = it_.subscribe("/camera_front/image_ipm", 1, &ped_crossing::imageCallback, this);

      polygon_sub_ = nh_.subscribe("img_polygon", 10, &ped_crossing::polygonCallback, this);
      private_nh_.param("scale", scale_, 20.0);

      private_nh_.param("extract_training_image", extract_training_image_, false);
      frame_serial_ = 0;
    }

	void ped_crossing::polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& polygon_in)
	{
		polygon_init_ = true;
		for(size_t i=0; i<4; i++) ipm_polygon_.push_back(cvPoint2D32f(polygon_in->polygon.points[i].x, polygon_in->polygon.points[i].y));
	}

    void ped_crossing::imageCallback (const sensor_msgs::ImageConstPtr& msg)
    {
    	printf("\n 1");
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
		IplImage* img_tmp = cvCreateImage(cvSize(color_image->width,color_image->height),IPL_DEPTH_8U, 3);
		cvResize(color_image, img_tmp);

		//2013-March: to erase the small yellow blocks accompanying the white strips;
		IplImage* yellow_mask = cvCreateImage(cvSize(color_image->width,color_image->height),IPL_DEPTH_8U, 1);
		IplImage* HSV_image = cvCreateImage(cvSize(color_image->width,color_image->height),IPL_DEPTH_8U, 3);
		cvCvtColor(color_image, HSV_image, CV_BGR2HSV);
		cvInRangeS(HSV_image, cvScalar(15, 80, 100), cvScalar(40, 255, 255), yellow_mask);
		cvThreshold(yellow_mask, yellow_mask, 100, 255, CV_THRESH_BINARY_INV);
		cvShowImage("yellow_mask", yellow_mask);

		binary_img = cvCreateImage(cvSize(img_tmp->width,img_tmp->height),IPL_DEPTH_8U, 1);
		cvCvtColor(img_tmp, binary_img, CV_BGR2GRAY);
		Img_preproc_local(binary_img, binary_img);
		cvAnd(yellow_mask, binary_img, binary_img);

		cvReleaseImage(&img_tmp);
		cvReleaseImage(&yellow_mask);
		cvReleaseImage(&HSV_image);
		cvShowImage("ped_binary_image", binary_img);

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

		IplImage *contour_img = cvCreateImage(cvSize(binary_img->width,binary_img->height),IPL_DEPTH_8U, 3);
		cvZero(contour_img);
		cvCvtColor(binary_img, contour_img, CV_GRAY2BGR);

		CvMoments cvm;
		CvHuMoments cvHM;

		CvBox2D cvBox;
		CvMemStorage *mem_box;
		mem_box = cvCreateMemStorage(0);

		CvSeq *contour_poly;
		CvMemStorage *mem_poly;
		mem_poly = cvCreateMemStorage(0);

		std::vector <size_t> lane_serials;

		std::vector<CvPoint2D32f> contour_boxes_centers;

		size_t contour_serial = 0;
        for (; contours != 0; contours = contours->h_next)
        {
        	cvContourMoments(contours, &cvm);
			double contour_weight = cvm.m00;
			double contour_perimeter = cvContourPerimeter(contours);
			//2nd feature, HuMoment;
			cvGetHuMoments(&cvm, &cvHM);
			//3rd feature, side lengths of bounding box;
			cvBox = cvMinAreaRect2(contours, mem_box);

			contour_boxes_centers.push_back(cvBox.center);
			//4th feature: number of points after polygon approximation;
			contour_poly = cvApproxPoly( contours, sizeof(CvContour), mem_poly, CV_POLY_APPROX_DP, 2, 0 );
			int approxPtNum = int (contour_poly->total);
			int contour_class = classify_contour (contour_weight, contour_perimeter, cvHM, cvBox, approxPtNum);

			//contour_class = 1;
			if(contour_class==1)
			{
				DrawBox(cvBox, contour_img, CV_RGB(255,255,0));
				lane_serials.push_back(contour_serial);
			}
			contour_serial ++ ;
		}

        contours = first_contour;

        IplImage *tmp_image = cvCreateImage(cvGetSize(contour_img),8,1);
        cvZero(tmp_image);

        std::vector<size_t> best_cluster;
        if(contours!=0)  best_cluster =  ped_crossing::cluster_contours (contours, lane_serials);
        printf("\n-----best_cluster size() %ld\n", best_cluster.size());
        if(best_cluster.size() > 3)
        {
        	for(size_t i=0; i<best_cluster.size();i++)
        	{
        		contours = first_contour;
        		size_t j=0;
				for(; contours != 0; contours = contours->h_next)
				{
					if(j==best_cluster[i])
					{
						cvDrawContours(contour_img, contours, CV_RGB(255,0,0), CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));
						cvDrawContours(tmp_image, contours, cvScalar(255), cvScalar(0), -1, CV_FILLED, 8, cvPoint(0,0));
						break;
					}
					j++;
				}
        	}

            for(size_t i=0; i<best_cluster.size()-1;i++)
            {
            	CvPoint center_tmp1, center_tmp2;
            	center_tmp1.x = (int)contour_boxes_centers[best_cluster[i]].x;
            	center_tmp1.y = (int)contour_boxes_centers[best_cluster[i]].y;
            	center_tmp2.x = (int)contour_boxes_centers[best_cluster[i+1]].x;
            	center_tmp2.y = (int)contour_boxes_centers[best_cluster[i+1]].y;
            	cvLine(tmp_image, center_tmp1, center_tmp2, cvScalar(255), 5);
            }

            CvSeq *contour_tmp = 0;       //always keep one copy of the beginning of this list, for further usage;
            CvMemStorage *mem_contour_tmp;
            CvMemStorage *mem_box_tmp;
            mem_contour_tmp = cvCreateMemStorage(0);
            mem_box_tmp = cvCreateMemStorage(0);

            //cvFindContours(tmp_image, mem_contour_tmp, &contour_tmp);

            CvContourScanner scanner_tmp = cvStartFindContours(tmp_image, mem_contour_tmp, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
            contour_tmp=cvFindNextContour(scanner_tmp);

            CvBox2D cvBox_tmp;
            cvBox_tmp = cvMinAreaRect2(contour_tmp, mem_box_tmp);
            DrawBox(cvBox_tmp, contour_img, CV_RGB(255,255,0));

            cvReleaseMemStorage(&mem_box_tmp);
            cvReleaseMemStorage(&mem_contour_tmp);
            cvReleaseImage(&tmp_image);
        }


        if(contour_serial>0) extract_training_image(binary_img);


        printf("2\n");

        cvShowImage("ped_contour_image",contour_img);

        cvReleaseMemStorage(&mem_contours);
        cvReleaseMemStorage(&mem_box);

        cvWaitKey(1);
        cvReleaseImage(&contour_img);
    }



    std::vector<size_t> ped_crossing::cluster_contours (CvSeq* contour, std::vector <size_t> lane_serials)
    {
    	CvMemStorage* storage = cvCreateMemStorage(0);
    	CvSeq* serial_seq= cvCreateSeq( 0, sizeof(CvSeq), sizeof(size_t), storage);
    	for(size_t i=0; i<lane_serials.size(); i++) cvSeqPush( serial_seq, &lane_serials[i]);

    	printf("\n lane_serials %ld \n", lane_serials.size());

    	CvSeq* labels = 0;
		int class_count = cvSeqPartition(serial_seq,
										 0,
										 &labels,
										 (CvCmpFunc)is_equal,
										 contour);

		printf("\n cluster number %d \n", class_count);

		std::vector<int> label_counts(class_count, 0);

	    for( size_t i = 0; i < labels->total; i++ )
	    {
	    	int class_label = *(int*)cvGetSeqElem( labels, i);
	    	printf("class_label %d\t", class_label);
	    	label_counts[class_label] = label_counts[class_label] + 1;
	    }

	    int class_serial =0;
	    int max_count = 0;
	    for( size_t i = 0; i < label_counts.size(); i++ )
	    {

	    	if(label_counts[i] >max_count)
	    	{
	    		max_count = label_counts[i];
	    		class_serial = i;
	    	}
	    	printf("count %ld, class_serial %ld\t", label_counts[i], class_serial);
	    }

		std::vector<size_t> tmp;
	    for( size_t i = 0; i < labels->total; i++ )
	    {
	    	if(*(int*)cvGetSeqElem( labels, i) == class_serial)
	    	{
	    		tmp.push_back(lane_serials[i]);
	    	}
	    }
		return tmp;
    }



    void ped_crossing::extract_training_image(IplImage* binary_img)
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
    int ped_crossing::classify_contour(double weight_input, double perimeter_input,
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
		class_label = ped_crossing_classifier_->classify_objects(marker_feature_vector, vector_length);

        return class_label;
    }

    //this filtering function can be simply replaced with a svm-classifier;
    CvSeq* ped_crossing::filter_contours (CvContourScanner &scanner)
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
            bool len_criterion = (len_meter > 3.0);

            //2nd criterion: long side should exceed certain threshold;
			cvBox = cvMinAreaRect2(c, mem_box);
			float height =  cvBox.size.height;
			float width  =  cvBox.size.width;
			float long_side = max(height, width);
			bool  long_side_criterion = long_side > 1.5*scale_;

            //3rd criterion: short side should exceed certain threshold;
			float short_side = min(height, width);
			bool  short_side_criterion = short_side > 0.5*scale_;

			//4th criterion: no touching polygon boundary;
			//this criterion only applies to discrete markers, while not to continuous lanes;
			bool inside_polygon = true;

	        CvSeq *contours_filter;
			CvMemStorage *mem_poly_filter;
			mem_poly_filter = cvCreateMemStorage(0);
			contours_filter = cvApproxPoly( c, sizeof(CvContour), mem_poly_filter, CV_POLY_APPROX_DP, 2, 0 );
	        for(int i=0; i<contours_filter->total; i++)
	        {
	            CvPoint* p = (CvPoint*)cvGetSeqElem(contours_filter, i);
	            for(int a = -1; a<=1; a=a+1)
				{
					for(int b = -1; b<=1; b=b+1)
					{
						CvPoint2D32f tmppoint = cvPoint2D32f(p->x+a, p->y+b);
						if(!pointInPolygon <CvPoint2D32f> (tmppoint,ipm_polygon_))
						{
							inside_polygon = false;
							break;
						}
					}
					if(!inside_polygon) break;
				}
				//special processing for the upper and lower bound;
				if(p->y+3 >=ipm_polygon_[0].y || p->y -3 <=ipm_polygon_[3].y)
				{
					inside_polygon = false;
					break;
				}
	        }
	        cvReleaseMemStorage(&mem_poly_filter);

			//5th: the polygon should have 4-5 corners;
			bool rectangle_criteria = true;
			CvSeq *contours;
			CvMemStorage *mem_poly;
			mem_poly = cvCreateMemStorage(0);
			contours = cvApproxPoly( c, sizeof(CvContour), mem_poly, CV_POLY_APPROX_DP, 5, 0 );
			if(contours->total > 6) rectangle_criteria = false;

			inside_polygon=true;

			bool contour_criteria = len_criterion && long_side_criterion && short_side_criterion && inside_polygon && rectangle_criteria;
            if(!contour_criteria) cvSubstituteContour(scanner, NULL);

            cvReleaseMemStorage(&mem_poly);
        }
        CvSeq *contours = cvEndFindContours(&scanner);
        cvReleaseMemStorage(&mem_box);
        return contours;
    }

	void ped_crossing::IpmImage_to_pcl(std::vector <CvPoint2D32f> & pts_image, sensor_msgs::PointCloud &pts_3d)
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


	ped_crossing::~ped_crossing()
    {
    }
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "ped_crossing");
	 ros::NodeHandle n;
	 golfcar_vision::ped_crossing ped_crossing_node;
     ros::spin();
     return 0;
}
