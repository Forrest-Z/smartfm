#include "lane_marker.h"

namespace golfcar_vision{
  
	lane_marker::lane_marker():
				private_nh_("~"),
				it_(nh_)
    { 
	  polygon_init_ = false;
	  fixedTf_inited_ = false;
	  ipm_para_init_ = false;

      string marker_model_path, marker_scale_path;
	  private_nh_.param("arrow_model_path", marker_model_path, std::string("/home/baoxing/workspace/data_and_model/arrow_20130308.model"));
	  private_nh_.param("arrow_scale_path", marker_scale_path, std::string("/home/baoxing/workspace/data_and_model/arrow_20130308.range"));
      marker_classifier_ = new golfcar_ml::svm_classifier(marker_model_path, marker_scale_path);
      image_sub_ = it_.subscribe("/camera_front/image_ipm", 1, &lane_marker::imageCallback, this);

      polygon_sub_ = nh_.subscribe("img_polygon", 10, &lane_marker::polygonCallback, this);
      private_nh_.param("scale", scale_, 20.0);
      private_nh_.param("extract_training_image", extract_training_image_, false);
      frame_serial_ = 0;

      markers_pub_ = nh_.advertise<vision_lane_detection::markers_info>("markers", 10);
      markers_ptcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("markers_ptcloud",2);

	  private_nh_.param("image_folder_path", image_folder_path_, std::string("/home/baoxing/images"));
      store_parameter_ = true;

      mask_init_ = false;
    }

	void lane_marker::polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& polygon_in)
	{
		if(!polygon_init_)
			for(size_t i=0; i<4; i++) ipm_polygon_.push_back(cvPoint2D32f(polygon_in->polygon.points[i].x, polygon_in->polygon.points[i].y));
		polygon_init_ = true;
	}

    void lane_marker::imageCallback (const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO("Arrow CallBack Begin");

        fmutil::Stopwatch sw;
        sw.start("image processing");

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
    	IplImage* color_image, *binary_img, *binary_copy;
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
		binary_copy = cvCreateImage(cvGetSize(color_image),8,1);
		cvCvtColor(color_image, binary_img, CV_BGR2GRAY);
		cvCopy(binary_img, binary_copy);
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
					CvPoint2D32f tmppoint = cvPoint2D32f(iw, ih);

					if(pointInPolygon <CvPoint2D32f> (tmppoint,ipm_polygon_))
					{
						ipm_data[ih*ipm_step+iw]=255;
					}
				}
			}
		}
		cvAnd(image_mask_, binary_img, binary_img);

		cvShowImage("arrow_binary_img", binary_img);

		sw.end();

		sw.start("extract contour");
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

        sw.end();

        sw.start("process contours");
        //-------------------------------------------------------------------------------------------------------------------------------
        //3. classify each remained candidates; visualize the markers detected;
        //-------------------------------------------------------------------------------------------------------------------------------
        IplImage *contour_img = cvCreateImage(cvSize(binary_img->width,binary_img->height),IPL_DEPTH_8U, 3);
        cvCvtColor(binary_img, contour_img, CV_GRAY2BGR);
        CvScalar ext_color;

        ROS_INFO("Arrow --------2----------");

        CvMoments cvm; 
        CvHuMoments cvHM;
        
        CvBox2D cvBox;
        CvMemStorage *mem_box;
        mem_box = cvCreateMemStorage(0);
        
		CvSeq *contour_poly;
		CvMemStorage *mem_poly;
		mem_poly = cvCreateMemStorage(0);

		vision_lane_detection::markers_info markers_output;
		markers_output.header = msg->header;
		markers_output.header.frame_id = "base_link";

        int contour_num_in_this_image = 0;   // to calculate the number of contour candidates in one image;
        for (; contours != 0; contours = contours->h_next)
        {
            contour_num_in_this_image++;
            ROS_INFO("contour %d", contour_num_in_this_image);
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
            if(contour_class==1||contour_class==2||contour_class==3)
            {
				//-----------------------------------------------------------------------------------------------
				//to visualize all the candidates, together with their bounding box;
				DrawBox(cvBox,contour_img, ext_color);
				cvDrawContours(contour_img, contours, ext_color, CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));
				//-----------------------------------------------------------------------------------------------
				
                vision_lane_detection::marker_info marker_output;
                marker_output.class_label = contour_class;
                //this default number denotes no angle information;
                marker_output.thetha = 3*M_PI;
                pose_contour(contours, cvm, marker_output);
                
                //to visualize the posing result from 
                CvFont font;
                double hScale=1.0;
                double vScale=1.0;
                int lineWidth=2;
                CvPoint origin;
                stringstream  class_string;
                class_string<<"type "<< contour_class; 
                const char *class_name = class_string.str().c_str();
                origin.x = (int)cvBox.center.x+10;
                origin.y = (int)cvBox.center.y;
                cvInitFont(&font,CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);
                cvPutText(contour_img, class_name, origin, &font, CV_RGB(0,255,0));
                
				CvPoint centroid_pt; 
				centroid_pt.x = int(cvm.m10/cvm.m00);
				centroid_pt.y = int(cvm.m01/cvm.m00);
                cvCircle( contour_img, centroid_pt, 3, CV_RGB(0,255,0), 2);
                ROS_INFO("lane marker detected");
                if(marker_output.thetha!=3*M_PI)
                {
					CvFont font2;
					double hScale2=0.5;
					double vScale2=0.5;
					int lineWidth2=1;
					CvPoint origin_2;
					stringstream  pose_string;
					pose_string<<"pose:(" <<setiosflags(ios::fixed) << setprecision(3) << marker_output.x << ","<<marker_output.y <<","<< marker_output.thetha<<")"; 
					const char *pose_info = pose_string.str().c_str();
					origin_2.x = (int)cvBox.center.x+10;
					origin_2.y = (int)cvBox.center.y+20;
					cvInitFont(&font2,CV_FONT_ITALIC, hScale2, vScale2, 0, lineWidth2);
					cvPutText(contour_img, pose_info, origin_2, &font2, CV_RGB(0,255,0));
					
					markers_output.vec.push_back(marker_output);
				}
            }
			else {}
		}
        cvShowImage("arrow_contour_image",contour_img);
        markers_pub_.publish(markers_output);

        sensor_msgs::PointCloud markers_ptcloud;
		std::vector<CvPoint2D32f> markers_pt_inImg;
		for(unsigned int il=0; il<markers_output.vec.size(); il++)
		{
		  for(unsigned int ip=0; ip< markers_output.vec[il].points.size(); ip++)
		  {
			  CvPoint2D32f pttmp;
			  pttmp.x = markers_output.vec[il].points[ip].x;
			  pttmp.y = markers_output.vec[il].points[ip].y;
			  markers_pt_inImg.push_back(pttmp);
		  }
		}

		lane_marker::IpmImage_to_pcl(markers_pt_inImg, markers_ptcloud);
		markers_ptcloud.header.stamp = msg -> header.stamp;
		markers_ptcloud.header.frame_id = "base_link";
		geometry_msgs::Point32 refresh_trick_rviz_pt;
		refresh_trick_rviz_pt.x=10000.0;
		markers_ptcloud.points.push_back(refresh_trick_rviz_pt);
		markers_ptcloud_pub_.publish(markers_ptcloud);

        //-----------------------------------------------------------------------------------------------
        //only use when to extract training pictures;
        //once entering this loop, meaning at least one contour exist, save image as training sets;
        //-----------------------------------------------------------------------------------------------
        if(contour_num_in_this_image>0) extract_training_image(color_image);

        if(extract_training_image_ && store_parameter_ )
        {
        	store_parameter_ = false;
        	FILE *fp;

        	stringstream  name_string;
        	name_string<< image_folder_path_ <<"/parameter_file";

        	const char *param_file_name = name_string.str().c_str();
        	if((fp=fopen(param_file_name, "w"))==NULL){printf("cannot open file\n");}
        	else
        	{
        		fprintf(fp, "%f\t", scale_);
        		fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t", ipm_polygon_[0].x, ipm_polygon_[0].y, ipm_polygon_[1].x, ipm_polygon_[1].y,
        														ipm_polygon_[2].x, ipm_polygon_[2].y, ipm_polygon_[3].x, ipm_polygon_[3].y);
        		fclose(fp);
        	}
        }

        cvReleaseMemStorage(&mem_contours);
        cvReleaseMemStorage(&mem_box);
        cvReleaseMemStorage(&mem_poly);

        cvWaitKey(1);
        cvReleaseImage(&contour_img);
        cvReleaseImage(&binary_img);
        cvReleaseImage(&binary_copy);

        ROS_INFO("Arrow CallBack End");
        sw.end();
    }

    void lane_marker::extract_training_image(IplImage* binary_img)
    {
        if(extract_training_image_)
        {
			stringstream  name_string;
			int stored_serial= frame_serial_/2;
			//pay attention, this path to the folder cannot be too long, or OpenCV will crash;
			name_string<< image_folder_path_ <<"/frame"<<stored_serial<<".png";

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
    int lane_marker::classify_contour(double weight_input, double perimeter_input,
									 CvHuMoments &HM_input, CvBox2D &Box_input, int polyNum_input)
    {
        float boxAngle  =   Box_input.angle;
        float height    =   Box_input.size.height;
        float width     =   Box_input.size.width;
        ROS_DEBUG("%lf, %lf, %lf, %lf, %lf, %lf, %lf", HM_input.hu1, HM_input.hu2, HM_input.hu3, HM_input.hu4, HM_input.hu5, HM_input.hu6, HM_input.hu7);
        ROS_DEBUG("boxAngle, height, width: %lf, %lf, %lf", boxAngle, height, width);
        //Data scaling here enables svm to get better accuracy;

        //keep accordance with "data_formating";
        double long_side_scaled = max(width, height);
        double short_side_scaled = min(width, height);

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
		class_label = marker_classifier_->classify_objects(marker_feature_vector, vector_length);
        return class_label;
    }
    
    CvSeq* lane_marker::filter_contours (CvContourScanner &scanner)
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
				if(!inside_polygon) break;
	        }
	        cvReleaseMemStorage(&mem_poly_filter);

			if(extract_training_image_)inside_polygon = true;

			bool contour_criteria = len_criterion && long_side_criterion && inside_polygon;
            if(!contour_criteria)
            {
            	cvSubstituteContour(scanner, NULL);
            }
        }
        CvSeq *contours = cvEndFindContours(&scanner);
        cvReleaseMemStorage(&mem_box);
        return contours;
      }
    
    
    void lane_marker::pose_contour(CvSeq *contour_raw, CvMoments &cvm, vision_lane_detection::marker_info &marker_para)
    {
        CvSeq *contours;
        CvMemStorage *mem_poly;
        mem_poly = cvCreateMemStorage(0);
        contours = cvApproxPoly( contour_raw, sizeof(CvContour), mem_poly, CV_POLY_APPROX_DP, 2, 0 );
        
        //X=(x1m1+x2m2+‥+ximi)/M, Y=(y1m1+y2m2+‥+yimi)/M
        marker_para.x = cvm.m10/cvm.m00;
        marker_para.y = cvm.m01/cvm.m00;
        CvPoint2D32f contour_center;
        contour_center.x = (float)marker_para.x;
        contour_center.y = (float)marker_para.y;

        CvMoments *arrow_moment = &cvm;
        double u11 = cvGetCentralMoment(arrow_moment, 1, 1);
        double u20 = cvGetCentralMoment(arrow_moment, 2, 0);
        double u02 = cvGetCentralMoment(arrow_moment, 0, 2);

        marker_para.thetha = 0.5*atan(2*u11/(u20-u02));

        //turn it into the vehicle frame;
        //this angle is the deflection angle from "vehicle x-axis" to its 1st principal axis;
        marker_para.thetha = -marker_para.thetha;


        /*
        if(contours->total<5){ROS_DEBUG("this marker is bad! no angle information;");}
        else
        {
			   //add this to visualize the contour of the lane_markers;
			   geometry_msgs::Point32 pttmp;
			   for(int i=0; i<contour_raw->total; i++)
            {
					 CvPoint* p = (CvPoint*)cvGetSeqElem(contour_raw, i);
					 pttmp.x = p->x;
					 pttmp.y = p->y;
					 marker_para.points.push_back(pttmp);
				}
				//--------------------------------------------------------
				
            std::vector <CvPoint> vertices;
            std::vector <CvPoint2D32f> centered_vertices;
            
            for(int i=0; i<contours->total; i++)
            {
                CvPoint* p = (CvPoint*)cvGetSeqElem(contours, i);
                vertices.push_back(*p);
                
                CvPoint2D32f centered_vertix = centroid_centering_coordinate(*p, contour_center);
                centered_vertices.push_back(centered_vertix);
            }
            
            float vertix_x_new, vertix_y_new;
            float vertix_x_old, vertix_y_old;
            float distance;
            std::vector <float> distance_vec;
                    CvMoments *arrow_moment = &cvm;
        double u11 = cvGetCentralMoment(arrow_moment, 1, 1);
        double u20 = cvGetCentralMoment(arrow_moment, 2, 0);
        double u02 = cvGetCentralMoment(arrow_moment, 0, 2);

        marker_para.thetha = 0.5*atan(2*u11/(u20-u02));

        //turn it into the vehicle frame;
        //this angle is the deflection angle from "vehicle x-axis" to its 1st principal axis;
        marker_para.thetha = -marker_para.thetha;
            //pay attention to the sequence, distance from line0 ("n" to "0"), line1 ("0" to "1")...lineN ("n-1" to "n").
            if(vertices.size()>2)   //no need any more, since we have alread denotes the minimum number before;
            {
                vertix_x_new = vertices[0].x;
                vertix_y_new = vertices[0].y;
                vertix_x_old = vertices.back().x;
                vertix_y_old = vertices.back().y;
                distance = sqrtf((vertix_x_new-vertix_x_old)*(vertix_x_new-vertix_x_old)+(vertix_y_new-vertix_y_old)*(vertix_y_new-vertix_y_old));
                distance_vec.push_back(distance);
            }
            for(unsigned int i=1; i<vertices.size(); i++)
            {
                vertix_x_new = vertices[i].x;
                vertix_y_new = vertices[i].y;
                vertix_x_old = vertices[i-1].x;
                vertix_y_old = vertices[i-1].y;
                
                distance = sqrtf((vertix_x_new-vertix_x_old)*(vertix_x_new-vertix_x_old)+(vertix_y_new-vertix_y_old)*(vertix_y_new-vertix_y_old));
                distance_vec.push_back(distance);
            }
            
            unsigned int longest_serial;
			float temp_distance1;
			longest_serial = find_longest_distance(distance_vec);
			temp_distance1 = distance_vec[longest_serial];
			
			if(temp_distance1<50.0){printf("this marker is not long enough! no angle information;");}
			else
			{
				unsigned int line_front, line_back;
				line_front = longest_serial;
				if(line_front > 0) {line_back = line_front -1;}
				else {line_back = line_front + distance_vec.size()-1;}
				
				unsigned int forward_line_front, backward_line_front;
				
				if(longest_serial < 2 ){backward_line_front = longest_serial + distance_vec.size() -2;}
				else {backward_line_front = longest_serial -2;}
				if(longest_serial + 2 >= distance_vec.size() ){forward_line_front = longest_serial +2 -distance_vec.size();}
				else {forward_line_front = longest_serial+2;}
				
				unsigned int forward_line_back, backward_line_back;
				if(backward_line_front > 0){backward_line_back = backward_line_front - 1;}
				else{backward_line_back = backward_line_front + distance_vec.size()-1;}
				if(forward_line_front > 0){forward_line_back = forward_line_front - 1;}
				else{forward_line_back = forward_line_front + distance_vec.size()-1;}
				
				bool fd_long = false;
				bool fd_angle = false;
				bool bd_long = false;
				bool bd_angle = false;
				
				CvPoint frontPt, backPt, fd_frontPt, fd_backPt, bd_frontPt, bd_backPt;
				frontPt    = vertices[line_front];
				backPt     = vertices[line_back];
				fd_frontPt = vertices[forward_line_front];
				fd_backPt  = vertices[forward_line_back];
				bd_frontPt = vertices[backward_line_front];
				bd_backPt  = vertices[backward_line_back];
				
				//distance criteria 
				if(distance_vec[forward_line_front]/temp_distance1  > 0.5) {fd_long = true;}
				if(distance_vec[backward_line_front]/temp_distance1 > 0.5) {bd_long = true;}

				double angle1 = atan2f((backPt.y-frontPt.y), (backPt.x-frontPt.x));
				double angle2 = atan2f((fd_frontPt.y-fd_backPt.y), (fd_frontPt.x-fd_backPt.x));
				double fd_abs_delt = fabs(fabs(angle1)-fabs(angle2));
				
				double angle3 = atan2f((frontPt.y-backPt.y), (frontPt.x-backPt.x));
				double angle4 = atan2f((bd_backPt.y-bd_frontPt.y), (bd_backPt.x-bd_frontPt.x));
				double bd_abs_delt = fabs(fabs(angle3)-fabs(angle4));
				
				if(fd_abs_delt < M_PI_4) {fd_angle = true;}
				if(bd_abs_delt < M_PI_4) {bd_angle = true;}
				bool fd_satisfy = fd_long && fd_angle;
				bool bd_satisfy = bd_long && bd_angle;
				
				bool find_cor_pts = true;
				
				unsigned int line1_front_serial, line1_back_serial, line2_front_serial, line2_back_serial;
				
				CvPoint line1_frontd, line1_backd, line2_frontd, line2_backd;
				CvPoint2D32f line1_front, line1_back, line2_front, line2_back;
				
				//line1 is the line at the left side of the marker;
				if(fd_satisfy && !bd_satisfy)
				{
					line1_front_serial = line_back;
					line1_back_serial  = line_front;
					line2_front_serial = forward_line_front;
					line2_back_serial  = forward_line_back;
					
					//ROS_DEBUG("use_front_only");
				}
				else if (!fd_satisfy && bd_satisfy)
				{
					line2_front_serial = line_front;
					line2_back_serial  = line_back;
					line1_front_serial = backward_line_back;
					line1_back_serial  = backward_line_front;
					
					//ROS_DEBUG("use_back_only");
				}
				else if (fd_satisfy && bd_satisfy)
				{
					if(distance_vec[forward_line_front]>distance_vec[backward_line_front])
					{
						line1_front_serial = line_back;
						line1_back_serial  = line_front;
						line2_front_serial = forward_line_front;
						line2_back_serial  = forward_line_back;
					}
					else
					{
						line2_front_serial = line_front;
						line2_back_serial  = line_back;
						line1_front_serial = backward_line_back;
						line1_back_serial  = backward_line_front;
					}
				}
				else
				{
					find_cor_pts = false;
				}
				
				if(find_cor_pts)
				{
					line1_frontd = vertices[line1_front_serial];
					line1_backd  = vertices[line1_back_serial];
					line2_frontd = vertices[line2_front_serial];
					line2_backd  = vertices[line2_back_serial];
					
					line1_front  = centered_vertices[line1_front_serial];
					line1_back   = centered_vertices[line1_back_serial];
					line2_front  = centered_vertices[line2_front_serial];
					line2_back   = centered_vertices[line2_back_serial];
					
					double angle1 = atan2f((line1_front.y-line1_back.y), (line1_front.x-line1_back.x));
					double angle2 = atan2f((line2_front.y-line2_back.y), (line2_front.x-line2_back.x));
					
					//remember to filter the case happened in picture 215;
					double abs_delt = fabs(fabs(angle1)-fabs(angle2));
					if(abs_delt>M_PI_2){ROS_DEBUG("noisy situation, do not provide angle");}
					else
					{
						//thetha is in "rad" here;
						double thetha = (angle1+angle2)/2.0;
						//be careful with this case
						if((angle1 > M_PI_2 && angle2 < -M_PI_2) || (angle2 > M_PI_2&& angle1 < -M_PI_2))
						{thetha = thetha + M_PI;}
						marker_para.thetha = thetha;
					}
				}
			}
        }
        */

        cvt_pose_baselink(marker_para);
        ROS_DEBUG("---------marker %lf, %lf, %lf----------\n", marker_para.x, marker_para.y, marker_para.thetha);
        cvReleaseMemStorage(&mem_poly);
    }

    void lane_marker::cvt_pose_baselink(vision_lane_detection::marker_info &marker_para)
    {
        double delt_x =  -(marker_para.y - center_pix_y_)/scale_;
        double delt_y =  -(marker_para.x - center_pix_x_)/scale_;
        marker_para.x = center_x_ + delt_x;
        marker_para.y = center_y_ + delt_y;
    }

	void lane_marker::IpmImage_to_pcl(std::vector <CvPoint2D32f> & pts_image, sensor_msgs::PointCloud &pts_3d)
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


    lane_marker::~lane_marker()
    {
    	//2013-March
    	if(mask_init_)cvReleaseImage(&image_mask_);
    }
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "lane_marker");
	 ros::NodeHandle n;
	 golfcar_vision::lane_marker lane_marker_node;
     ros::spin();
     return 0;
}
