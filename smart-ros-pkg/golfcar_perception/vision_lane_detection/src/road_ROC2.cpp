#include "road_ROC2.h"

namespace golfcar_vision{
  
	road_roc::road_roc():
				private_nh_("~"),
				it_(nh_)
    { 
	  polygon_init_ = false;
	  fixedTf_inited_ = false;
	  ipm_para_init_ = false;

      string road_roc_model_path, road_roc_scale_path;
	  private_nh_.param("road_roc_model_path", road_roc_model_path, std::string("/home/baoxing/workspace/data_and_model/scaled_20120726.model"));
	  private_nh_.param("road_roc_scale_path", road_roc_scale_path, std::string("/home/baoxing/workspace/data_and_model/range_20120726"));
      road_roc_classifier_ = new golfcar_ml::svm_classifier(road_roc_model_path, road_roc_scale_path);
      //image_sub_ = it_.subscribe("/camera_front/ipm_binary", 1, &road_roc::imageCallback, this);

      polygon_sub_ = nh_.subscribe("img_polygon", 10, &road_roc::polygonCallback, this);
      private_nh_.param("scale", scale_, 20.0);
      private_nh_.param("extract_training_image", extract_training_image_, false);
      frame_serial_ = 0;

      private_nh_.param("visualize_word_info", visualize_word_info_, false);
      private_nh_.param("save_word_image", save_word_image_, false);
      image_serial_ = 0;
    }

	void road_roc::polygonCallback(const geometry_msgs::PolygonStamped::ConstPtr& polygon_in)
	{
		if(!polygon_init_)
			for(size_t i=0; i<4; i++) ipm_polygon_.push_back(cvPoint2D32f(polygon_in->polygon.points[i].x, polygon_in->polygon.points[i].y));
		polygon_init_ = true;
	}

    void road_roc::imageCallback (const sensor_msgs::ImageConstPtr& msg, IplImage *visual_ipm, IplImage *visual_ipm_clean)
    {
    	ROS_INFO("Road ROC-- 1---");
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
    	IplImage* color_image, *binary_img, *binary_img_copy;
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

		binary_img = cvCreateImage(cvSize(img_tmp->width,img_tmp->height),IPL_DEPTH_8U, 1);
		cvCvtColor(img_tmp, binary_img, CV_BGR2GRAY);
		Img_preproc_local(binary_img, binary_img);

		binary_img_copy = cvCloneImage(binary_img);

		cvReleaseImage(&img_tmp);
		cvShowImage("roc_binary_image", binary_img);

        CvSeq *contours = 0;            //"contours" is a list of contour sequences, which is the core of "image_proc";
        CvSeq *first_contour = 0;       //always keep one copy of the beginning of this list, for further usage;
        CvMemStorage *mem_contours; 
        mem_contours = cvCreateMemStorage(0);
        
        //CvContourScanner scanner = cvStartFindContours(Itand, mem_contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        //CvContourScanner scanner = cvStartFindContours(binary_img, mem_contours, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
        cvFindContours(binary_img, mem_contours, &contours, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
        //-------------------------------------------------------------------------------------------------------------------------------
        //2. to filter noise at the boundary, and noise too small or too big; to re-write
        //-------------------------------------------------------------------------------------------------------------------------------
        //contours = filter_contours(scanner);
        //first_contour = contours;
		contours = filter_contours2(contours);
		first_contour = contours;

		IplImage *contour_img = cvCreateImage(cvSize(binary_img->width,binary_img->height),IPL_DEPTH_8U, 3);

		cvZero(contour_img);
		//cvCvtColor(binary_img, contour_img, CV_GRAY2BGR);

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
				//DrawBox(cvBox, contour_img, CV_RGB(255,255,0));
				lane_serials.push_back(contour_serial);
			}
			contour_serial ++ ;
		}

        contours = first_contour;

        std::vector<size_t> best_cluster;
        if(contours!=0)  best_cluster =  road_roc::cluster_contours (contours, lane_serials);

        ROS_INFO("-----best_cluster size() %ld", best_cluster.size());

        int vector_length = 27;
        int BOW_feature[27] = {0};

        IplImage *tmp_image = cvCreateImage(cvGetSize(contour_img),8,1);
        cvZero(tmp_image);

        FILE *fp_word;
        if(save_word_image_)
		{
        	fp_word = fopen("/home/baoxing/word/OCR","a");
        	fprintf(fp_word, "%d\t", image_serial_);
		}

        if(best_cluster.size() > 2)
        {
        	for(size_t i=0; i<best_cluster.size();i++)
        	{
        		contours = first_contour;
        		size_t j=0;
				for(; contours != 0; contours = contours->h_next)
				{
					if(j==best_cluster[i])
					{
						cvBox = cvMinAreaRect2(contours, mem_box);

						//characters to be drawn as a whole word later;
						//cvDrawContours(contour_img, contours, CV_RGB(255,0,0), CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));

						int square_side = (int)std::sqrt(cvBox.size.height*cvBox.size.height+ cvBox.size.width*cvBox.size.width);
						IplImage *character_tmp = cvCreateImage(cvSize(square_side+50, square_side+50),IPL_DEPTH_8U, 1);

						cvZero(character_tmp);
						CvPoint offset = cvPoint(square_side/2+25-cvBox.center.x, square_side/2+25 -cvBox.center.y);

						cvDrawContours(character_tmp, contours, cvScalar(255), cvScalar(0), -1, CV_FILLED, 8, offset);
						cvDrawContours(tmp_image, contours, cvScalar(255), cvScalar(0), -1, CV_FILLED, 8, cvPoint(0,0));

						double rotate_angle, rotate_scale;
						rotate_scale = 1.0;
						if(cvBox.size.height > cvBox.size.width) rotate_angle = -(-cvBox.angle); //clockwise rotate;
						else rotate_angle = 90.0 + cvBox.angle; //counter-clockwise;
						CvMat* rotate_mat = cvCreateMat(2,3,CV_32FC1);
						cv2DRotationMatrix(cvBox.center, rotate_angle, rotate_scale, rotate_mat);
						cvWarpAffine(character_tmp, character_tmp, rotate_mat);
						cvReleaseMat(&rotate_mat);

						if(save_word_image_)
						{
							stringstream  name_string_character;
							//pay attention, this path to the folder cannot be too long, or OpenCV will crash;
							name_string_character<< "/home/baoxing/word" <<"/frame"<<image_serial_<<"_"<<i<<".png";
							const char *output_name_character = name_string_character.str().c_str();

							if (!cvSaveImage(output_name_character, character_tmp))
							{
								ROS_ERROR("Cannot save images");
								return;
							}
						}

						/*
						CvMat *thining_mat = cvCreateMat(square_side+50, square_side+50,CV_8UC1);
						CvMat *output_mat = cvCreateMat(square_side+50, square_side+50,CV_8UC1);
						cvZero(character_tmp);
						cvZero(thining_mat);
						CvPoint offset = cvPoint(square_side/2+25-cvBox.center.x, square_side/2+25 -cvBox.center.y);
						cvDrawContours(thining_mat, contours, cvScalar(255), cvScalar(0), -1, CV_FILLED, 8, offset);
						MorphologicalThinning(thining_mat, output_mat);
						cvConvert( output_mat, character_tmp);
						*/

						char letter = 91;
				        if(lane_ocr_.recognize(character_tmp, letter))
				        {
				        	ROS_INFO("huhuhuhuhuhu-----%d", letter);
				        }
				        else
				        {
				        	//ROS_WARN("NONONONONONO");
				        }

				        if(save_word_image_)
				        {
				        	fprintf(fp_word, "%c\t", letter);
				        }

				        //turn to ASCII code;
				        if(letter>=65 && letter<=90) BOW_feature[letter-65]++;
				        else BOW_feature[26]++;

				        cvShowImage("character_tmp", character_tmp);
				        cvWaitKey(1);
				        cvReleaseImage(&character_tmp);
						//cvReleaseMat(&thining_mat);
						//cvReleaseMat(&output_mat);
						break;
					}
					j++;
				}
        	}

        	//Add the save_file function here, to save the word vector, character and the whole image;
            //-------------save word image------------------;
        	if(save_word_image_)
        	{
        		fprintf(fp_word, "\n");
        		fclose(fp_word);

				stringstream  name_string;
				//pay attention, this path to the folder cannot be too long, or OpenCV will crash;
				name_string<< "/home/baoxing/word" <<"/frame"<<image_serial_<<".png";

				const char *output_name = name_string.str().c_str();

				if (!cvSaveImage(output_name, color_image))
				{
					ROS_ERROR("Cannot save images");
					return;
				}
				image_serial_++;
        	}
            //-------------save word image------------------;


			std::string surface_word;
			if(word_detector_.identify(BOW_feature, vector_length, surface_word))
			{
				ROS_INFO("identify words %s", surface_word.c_str());
			}
			else ROS_INFO("identify no words");

			//for visualization purposes;
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
			//to be send to the original view later;
			CvBox2D cvBox_tmp;
			cvBox_tmp = cvMinAreaRect2(contour_tmp, mem_box_tmp);
			DrawBox(cvBox_tmp, contour_img, CV_RGB(255,255,0));

			//to mark all the white pixels (characters) inside the rectangle red;
			CvPoint2D32f pts_tmp[4];
			calc_cvBoxPoints( cvBox_tmp, pts_tmp);
			std::vector<CvPoint2D32f> word_polygon_tmp;
			for(size_t i=0; i<4; i++) word_polygon_tmp.push_back(pts_tmp[i]);
	        int img_height 		= contour_img -> height;
			int img_width  		= contour_img -> width;
			for(int ih=0; ih < img_height; ih++)
			{
				for(int iw=0; iw < img_width; iw++)
				{
					CvPoint pixel;
					pixel.x = iw;
					pixel.y = ih;
					CvScalar s=cvGet2D(binary_img_copy, pixel.y, pixel.x);
					if(pointInPolygon(cvPoint2D32f(iw, ih), word_polygon_tmp))
					{
						if(s.val[0]!=0)cvSet2D(contour_img, pixel.y, pixel.x, CV_RGB(255, 0, 0));
						else
						{
							cvSet2D(visual_ipm_clean, pixel.y, pixel.x, CV_RGB(0, 0, 0));
							cvSet2D(visual_ipm, pixel.y, pixel.x, CV_RGB(0, 0, 0));
						}
					}
				}
			}
			cvReleaseImage(&binary_img_copy);

			merge_images(visual_ipm_clean, contour_img);

			if(visualize_word_info_)
			{
				CvFont font;
				double hScale=0.4;
				double vScale=0.4;
				int lineWidth=1;
				CvPoint origin;
				origin.x = (int)cvBox_tmp.center.x+30;
				origin.y = (int)cvBox_tmp.center.y;
				cvInitFont(&font,CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);
				cvPutText(contour_img, surface_word.c_str(), origin, &font, CV_RGB(0,255,0));
			}

			cvReleaseMemStorage(&mem_box_tmp);
			cvReleaseMemStorage(&mem_contour_tmp);
			cvReleaseImage(&tmp_image);

        }
        else ROS_INFO("identify no words");

		cvShowImage("roc_contour_image",contour_img);
		merge_images(visual_ipm, contour_img);

        if(contour_serial>0) extract_training_image(binary_img);
        ROS_INFO("Road ROC-- 2---");

        cvWaitKey(1);
        cvReleaseMemStorage(&mem_contours);
        cvReleaseMemStorage(&mem_box);
        cvReleaseImage(&tmp_image);
        cvReleaseImage(&contour_img);
    }



    std::vector<size_t> road_roc::cluster_contours (CvSeq* contour, std::vector <size_t> lane_serials)
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



    void road_roc::extract_training_image(IplImage* binary_img)
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
    int road_roc::classify_contour(double weight_input, double perimeter_input,
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
		class_label = road_roc_classifier_->classify_objects(marker_feature_vector, vector_length);

        return class_label;
    }

    //this filtering function can be simply replaced with a svm-classifier;
    CvSeq* road_roc::filter_contours (CvContourScanner &scanner)
    {
        CvSeq* c;
        CvBox2D cvBox;
        CvMemStorage *mem_box;
        mem_box = cvCreateMemStorage(0);
        while((c=cvFindNextContour(scanner))!=NULL)
        {
        	if((c->v_prev)->v_next ==NULL) continue;
            //1st criterion: perimeter should be long enough;
            double len_pixel = cvContourPerimeter(c);
            double len_meter = len_pixel/scale_;
            bool len_criterion = (len_meter > CONTOUR_PERIMETER_THRESH);

            //2nd criterion: long side should exceed certain threshold;
			cvBox = cvMinAreaRect2(c, mem_box);
			float height =  cvBox.size.height;
			float width  =  cvBox.size.width;
			float long_side = max(height, width);
			bool  long_side_criterion = long_side > 1.5*scale_;

            //3rd criterion: short side should exceed certain threshold;
			float short_side = min(height, width);
			bool  short_side_criterion = short_side > SHORT_SIDE_THRESH*scale_;

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

			bool contour_criteria = len_criterion && long_side_criterion && short_side_criterion && inside_polygon;
            if(!contour_criteria) cvSubstituteContour(scanner, NULL);

        }
        CvSeq *contours = cvEndFindContours(&scanner);
        cvReleaseMemStorage(&mem_box);
        return contours;
    }

    CvSeq* road_roc::filter_contours2 (CvSeq* contours)
    {
       	CvSeq *first_contour=NULL;

           CvSeq* c;
           CvBox2D cvBox;
           CvMemStorage *mem_box;
           mem_box = cvCreateMemStorage(0);
           c=contours;

           while(c!=NULL)
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
				bool  long_side_criterion = long_side > 1.5*scale_;

				//3rd criterion: short side should exceed certain threshold;
				float short_side = min(height, width);
				bool  short_side_criterion = short_side > SHORT_SIDE_THRESH*scale_;

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

				bool contour_criteria = len_criterion && long_side_criterion && short_side_criterion && inside_polygon;

				if(!contour_criteria)
				{
					if(c->h_prev){(c->h_prev)->h_next=c->h_next;}
					if(c->h_next){(c->h_next)->h_prev=c->h_prev;}
				}
				else
				{
					if(first_contour==NULL) first_contour = c;
				}
				c=c->h_next;
           }
           cvReleaseMemStorage(&mem_box);
           return first_contour;
    }

	void road_roc::IpmImage_to_pcl(std::vector <CvPoint2D32f> & pts_image, sensor_msgs::PointCloud &pts_3d)
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


	void road_roc::MorphologicalThinning(CvMat *pSrc, CvMat *pDst) {

			ROS_INFO("image thinning");
		        bool bDone = false;
		        int rows = pSrc->rows;
		        int cols = pSrc->cols;
		        /// pad source
		        CvMat *p_enlarged_src = cvCreateMat(rows + 2, cols + 2, CV_8UC1);
		        for(int i = 0; i < (rows+2); i++) {
		                CV_MAT_ELEM(*p_enlarged_src, uchar, i, 0)	 = 0;
		                CV_MAT_ELEM(*p_enlarged_src, uchar, i, cols+1)	= 0;
		        }
		        for(int j = 0; j < (cols+2); j++) {
		                CV_MAT_ELEM(*p_enlarged_src, uchar, 0, j)	 = 0;
		                CV_MAT_ELEM(*p_enlarged_src, uchar, rows+1, j)	= 0;
		        }
		        for(int i = 0; i < rows; i++) {
		                for(int j = 0; j < cols; j++) {
		                        if (CV_MAT_ELEM(*pSrc, uchar, i, j) >= 255/2) {
		                                CV_MAT_ELEM(*p_enlarged_src, uchar, i+1, j+1) = 255;
		                        }
		                        else CV_MAT_ELEM(*p_enlarged_src, uchar, i+1, j+1) = 0;
		                }
		        }
		        /// start to thin
		        CvMat *p_thinMat1	= cvCreateMat(rows + 2, cols + 2, CV_8UC1);
		        CvMat *p_thinMat2	= cvCreateMat(rows + 2, cols + 2, CV_8UC1);
		        CvMat *p_cmp	 = cvCreateMat(rows + 2, cols + 2, CV_8UC1);

		        while (bDone != true) {
		                /// sub-iteration 1
		                ThinSubiteration1(p_enlarged_src, p_thinMat1);
		                /// sub-iteration 2
		                ThinSubiteration2(p_thinMat1, p_thinMat2);


		                /// compare
		                cvCmp(p_enlarged_src, p_thinMat2, p_cmp, CV_CMP_EQ);
		                /// check
		                int num_non_zero = cvCountNonZero(p_cmp);

		                if(num_non_zero == (rows + 2) * (cols + 2)) {
		                        bDone = true;
		                }
		                /// copy
		                cvCopy(p_thinMat2, p_enlarged_src);

		        }
		        /// copy result
		        for(int i = 0; i < rows; i++) {
		                for(int j = 0; j < cols; j++) {
		                        CV_MAT_ELEM(*pDst, uchar, i, j) = CV_MAT_ELEM(*p_enlarged_src, uchar, i+1, j+1);
		                }
		        }
		        /// clean memory
		        cvReleaseMat(&p_enlarged_src);
		        cvReleaseMat(&p_thinMat1);
		        cvReleaseMat(&p_thinMat2);
		        cvReleaseMat(&p_cmp);
		}

		void road_roc::ThinSubiteration1(CvMat *pSrc, CvMat *pDst) {
		        int rows = pSrc->rows;
		        int cols = pSrc->cols;
		        cvCopy(pSrc, pDst);
		        for(int i = 0; i < rows; i++) {
		                for(int j = 0; j < cols; j++) {
		                        if(CV_MAT_ELEM(*pSrc, uchar, i, j) == 255) {
		                                /// get 8 neighbors
		                                /// calculate C(p)
		                                int neighbor0 = (int) CV_MAT_ELEM(*pSrc, uchar, i-1, j-1);
		                                int neighbor1 = (int) CV_MAT_ELEM(*pSrc, uchar, i-1, j);
		                                int neighbor2 = (int) CV_MAT_ELEM(*pSrc, uchar, i-1, j+1);
		                                int neighbor3 = (int) CV_MAT_ELEM(*pSrc, uchar, i, j+1);
		                                int neighbor4 = (int) CV_MAT_ELEM(*pSrc, uchar, i+1, j+1);
		                                int neighbor5 = (int) CV_MAT_ELEM(*pSrc, uchar, i+1, j);
		                                int neighbor6 = (int) CV_MAT_ELEM(*pSrc, uchar, i+1, j-1);
		                                int neighbor7 = (int) CV_MAT_ELEM(*pSrc, uchar, i, j-1);
		                                int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
		                                                 int(~neighbor3 & ( neighbor4 | neighbor5)) +
		                                                 int(~neighbor5 & ( neighbor6 | neighbor7)) +
		                                                 int(~neighbor7 & ( neighbor0 | neighbor1));
		                                if(C == 1*255) {
		                                        /// calculate N
		                                        int N1 = int(neighbor0 | neighbor1) +
		                                                         int(neighbor2 | neighbor3) +
		                                                         int(neighbor4 | neighbor5) +
		                                                         int(neighbor6 | neighbor7);
		                                        int N2 = int(neighbor1 | neighbor2) +
		                                                         int(neighbor3 | neighbor4) +
		                                                         int(neighbor5 | neighbor6) +
		                                                         int(neighbor7 | neighbor0);
		                                        int N = std::min(N1,N2);
		                                        if ((N == 2*255) || (N == 3*255)) {
		                                                /// calculate criteria 3
		                                                int c3 = ( neighbor1 | neighbor2 | ~neighbor4) & neighbor3;
		                                                if(c3 == 0) {
		                                                        CV_MAT_ELEM(*pDst, uchar, i, j) = 0;
		                                                }
		                                        }
		                                }
		                        }
		                }
		        }
		}


		void road_roc::ThinSubiteration2(CvMat *pSrc, CvMat *pDst) {
		        int rows = pSrc->rows;
		        int cols = pSrc->cols;
		        cvCopy(pSrc, pDst);
		        for(int i = 0; i < rows; i++) {
		                for(int j = 0; j < cols; j++) {
		                        if ( CV_MAT_ELEM(*pSrc, uchar, i, j) == 255) {
		                                /// get 8 neighbors
		                                /// calculate C(p)
		                                int neighbor0 = (int) CV_MAT_ELEM(*pSrc, uchar, i-1, j-1);
		                                int neighbor1 = (int) CV_MAT_ELEM(*pSrc, uchar, i-1, j);
		                                int neighbor2 = (int) CV_MAT_ELEM(*pSrc, uchar, i-1, j+1);
		                                int neighbor3 = (int) CV_MAT_ELEM(*pSrc, uchar, i, j+1);
		                                int neighbor4 = (int) CV_MAT_ELEM(*pSrc, uchar, i+1, j+1);
		                                int neighbor5 = (int) CV_MAT_ELEM(*pSrc, uchar, i+1, j);
		                                int neighbor6 = (int) CV_MAT_ELEM(*pSrc, uchar, i+1, j-1);
		                                int neighbor7 = (int) CV_MAT_ELEM(*pSrc, uchar, i, j-1);
		                                int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
		                                        int(~neighbor3 & ( neighbor4 | neighbor5)) +
		                                        int(~neighbor5 & ( neighbor6 | neighbor7)) +
		                                        int(~neighbor7 & ( neighbor0 | neighbor1));
		                                if(C == 1*255) {
		                                        /// calculate N
		                                        int N1 = int(neighbor0 | neighbor1) +
		                                                int(neighbor2 | neighbor3) +
		                                                int(neighbor4 | neighbor5) +
		                                                int(neighbor6 | neighbor7);
		                                        int N2 = int(neighbor1 | neighbor2) +
		                                                int(neighbor3 | neighbor4) +
		                                                int(neighbor5 | neighbor6) +
		                                                int(neighbor7 | neighbor0);
		                                        int N = std::min(N1,N2);
		                                        if((N == 2*255) || (N == 3*255)) {
		                                                int E = (neighbor5 | neighbor6 | ~neighbor0) & neighbor7;
		                                                if(E == 0) {
		                                                        CV_MAT_ELEM(*pDst, uchar, i, j) = 0;
		                                                }
		                                        }
		                                }
		                        }
		                }
		        }
		}

	road_roc::~road_roc()
    {
    }
};
