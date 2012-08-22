#include "image_proc.h"

namespace golfcar_vision{
  
    image_proc::image_proc(string svm_model_path, string svm_scale_path):
        init_flag_(false),
        extract_image_(false)
    { 
      cvNamedWindow("It_image");
      cvNamedWindow("Iat_image");
      cvNamedWindow("binary_image");
      cvNamedWindow("contour_image");
      cvNamedWindow("HistogramEqualized_image");

      string svm_model_file;
      //the name cannot be too long, or it cannot load;
      //svm_model_file = "/home/baoxing/workspace/data_and_model/scaled_20120726.model";
      svm_model_file = svm_model_path;
      svm_model_ = svm_load_model(svm_model_file.c_str());
      //the following line can help to check where the model has been loaded or not;
      cout<<" SVM loaded, type = "<< svm_model_->param.svm_type <<endl;
      
      string svm_scale_file;
      //svm_scale_file = "/home/baoxing/workspace/data_and_model/range_20120726";
      svm_scale_file = svm_scale_path;
      restore_scalefile(svm_scale_file, feature_min_, feature_max_, feature_index_);
      
      // initialize camera intrinsic matrix and distortion coefficients;
      // later may write in a loading function by reading text files;
      intrinsic_A_ = cvCreateMat(3,3,CV_32FC1);
      int row = 0; float *ptr = (float*)(intrinsic_A_->data.ptr + row * intrinsic_A_->step);
	  ptr[0] = 462.55911; ptr[1] = 0.00; ptr[2] = 326.21463;
	  row = 1; ptr = (float*)(intrinsic_A_->data.ptr + row * intrinsic_A_->step);
	  ptr[0] = 0.0; ptr[1] = 472.34580; ptr[2] = 188.72264;
	  row = 2; ptr = (float*)(intrinsic_A_->data.ptr + row * intrinsic_A_->step);
	  ptr[0] = 0.0; ptr[1] = 0.0; ptr[2] = 1.0;
	  
	  distortion_coeffs_= cvCreateMat(5,1,CV_32FC1);
	  ptr = (float*)(distortion_coeffs_->data.ptr + 0 * distortion_coeffs_->step); 	ptr[0] = -0.00791 ;
	  ptr = (float*)(distortion_coeffs_->data.ptr + 1 * distortion_coeffs_->step); 	ptr[0] = -0.05311;
	  ptr = (float*)(distortion_coeffs_->data.ptr + 2 * distortion_coeffs_->step);  ptr[0] = 0.00462;
	  ptr = (float*)(distortion_coeffs_->data.ptr + 3 * distortion_coeffs_->step);  ptr[0] = 0.00101;
	  ptr = (float*)(distortion_coeffs_->data.ptr + 4 * distortion_coeffs_->step);  ptr[0] = 0.0;
		
	  //initilize gndPts for 3 types of markers, which may be extended later;
      M1_gndPts_ = cvCreateMat(4,1,CV_32FC3);
      M2_gndPts_ = cvCreateMat(4,1,CV_32FC3);
      M3_gndPts_ = cvCreateMat(4,1,CV_32FC3);
      
	  row = 0; ptr = (float*)(M1_gndPts_->data.ptr + row * M1_gndPts_->step);
	  ptr[0] = 0.582;	ptr[1] = 0.0833; 	ptr[2] = 0.0;
      row = 1; ptr = (float*)(M1_gndPts_->data.ptr + row * M1_gndPts_->step);
	  ptr[0] = -2.51; 	ptr[1] = 0.167; 	ptr[2] = 0.0;
	  row = 2; ptr = (float*)(M1_gndPts_->data.ptr + row * M1_gndPts_->step);
	  ptr[0] = 0.582;	ptr[1] = -0.0833; 	ptr[2] = 0.0;
      row = 3; ptr = (float*)(M1_gndPts_->data.ptr + row * M1_gndPts_->step);
	  ptr[0] = -2.51; 	ptr[1] = -0.167; 	ptr[2] = 0.0;
	  
	  row = 0; ptr = (float*)(M2_gndPts_->data.ptr + row * M2_gndPts_->step);
	  ptr[0] = 0.664;	ptr[1] = 0.142; 	ptr[2] = 0.0;
      row = 1; ptr = (float*)(M2_gndPts_->data.ptr + row * M2_gndPts_->step);
	  ptr[0] = -2.878; 	ptr[1] = 0.208; 	ptr[2] = 0.0;
	  row = 2; ptr = (float*)(M2_gndPts_->data.ptr + row * M2_gndPts_->step);
	  ptr[0] = 0.664;	ptr[1] = -0.142;  	ptr[2] = 0.0;
      row = 3; ptr = (float*)(M2_gndPts_->data.ptr + row * M2_gndPts_->step);
	  ptr[0] = -2.878; 	ptr[1] = -0.208; 	ptr[2] = 0.0;
	  
	  row = 0; ptr = (float*)(M3_gndPts_->data.ptr + row * M3_gndPts_->step);
	  ptr[0] = -0.304;	ptr[1] = 0.0868; 	ptr[2] = 0.0;
      row = 1; ptr = (float*)(M3_gndPts_->data.ptr + row * M3_gndPts_->step);
	  ptr[0] = -2.354; 	ptr[1] = 0.212; 	ptr[2] = 0.0;
	  row = 2; ptr = (float*)(M3_gndPts_->data.ptr + row * M3_gndPts_->step);
	  ptr[0] = 0.736;	ptr[1] = -0.163; 	ptr[2] = 0.0;
      row = 3; ptr = (float*)(M3_gndPts_->data.ptr + row * M3_gndPts_->step);
	  ptr[0] = -2.354; 	ptr[1] = -0.212; 	ptr[2] = 0.0;
      
    }
  
    void image_proc::Extract_Markers (IplImage* src, float scale, vision_lane_detection::markers_info &markers_para, 
										int &frame_serial, CvPoint2D32f* dst_pointer,
										CvMat* projection_matrix, vision_lane_detection::markers_info &markers_para_2nd)
    {
        markers_para.vec.clear();
        markers_para_2nd.vec.clear();
        //initiation, first calculate four corners used to filter noise;
        if(!init_flag_)
        {
            scale_ = scale;
            init_flag_ = true;
            for (unsigned int i = 0; i<4; i++) corners_[i] = cvPointFrom32f(dst_pointer[i]);
            
            double delt_x1, delt_y1, delt_x2, delt_y2;
            delt_x1= corners_[0].x-corners_[3].x;
            delt_y1= corners_[0].y-corners_[3].y;
            para_A1_ = delt_y1/delt_x1;
            para_C1_ = corners_[0].y-para_A1_*corners_[0].x;
            delt_x2= corners_[1].x-corners_[2].x;
            delt_y2= corners_[1].y-corners_[2].y;
            para_A2_ = delt_y2/delt_x2;
            para_C2_ = corners_[1].y-para_A2_*corners_[1].x;
        }
        
        
        IplImage *HistogramEqualized = 0;
        HistogramEqualized = cvCreateImage(cvSize(src->width,src->height),IPL_DEPTH_8U, 1);
        cvEqualizeHist(src, HistogramEqualized);
        
        cvCircle( HistogramEqualized, cvPointFrom32f(dst_pointer[0]), 6, CV_RGB(0,255,0), 2);
		  cvCircle( HistogramEqualized, cvPointFrom32f(dst_pointer[1]), 6, CV_RGB(0,255,0), 2);
		  cvCircle( HistogramEqualized, cvPointFrom32f(dst_pointer[2]), 6, CV_RGB(0,255,0), 2);
		  cvCircle( HistogramEqualized, cvPointFrom32f(dst_pointer[3]), 6, CV_RGB(0,255,0), 2);
		  cvLine( HistogramEqualized, cvPointFrom32f(dst_pointer[0]), cvPointFrom32f(dst_pointer[1]), cvScalar(255), 1);
		  cvLine( HistogramEqualized, cvPointFrom32f(dst_pointer[1]), cvPointFrom32f(dst_pointer[2]), cvScalar(255), 1);
		  cvLine( HistogramEqualized, cvPointFrom32f(dst_pointer[2]), cvPointFrom32f(dst_pointer[3]), cvScalar(255), 1);
		  cvLine( HistogramEqualized, cvPointFrom32f(dst_pointer[3]), cvPointFrom32f(dst_pointer[0]), cvScalar(255), 1);
		  cvShowImage("HistogramEqualized_image", HistogramEqualized);
	
        //-------------------------------------------------------------------------------------------------------------------------
        //1. thresholding step, combining threshold and adaptive threshold methods, to get binary image;
        //-------------------------------------------------------------------------------------------------------------------------
        //It is short for "image threshold"; Iat "image adaptive threshold"; Itand is operates "and" on these two resulting images; 
        IplImage *It = 0, *Iat = 0, *Itand = 0;
        It = cvCreateImage(cvSize(src->width,src->height),IPL_DEPTH_8U, 1);
        Iat = cvCreateImage(cvSize(src->width,src->height),IPL_DEPTH_8U, 1);
        Itand = cvCreateImage(cvSize(src->width,src->height),IPL_DEPTH_8U, 1);
        //Threshold
        cvThreshold(src,It,BINARY_THRESH,255,CV_THRESH_BINARY);
        cvAdaptiveThreshold(src, Iat, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, BLOCK_SIZE, OFFSET);
        cvAnd(It, Iat, Itand);
        
        cvShowImage("It_image",It);
        cvShowImage("Iat_image",Iat);
        cvShowImage("binary_image",Itand);
        //cvSaveImage("/home/baoxing/It.png", It);
        //cvSaveImage("/home/baoxing/Iat.png", Iat);
        //cvSaveImage("/home/baoxing/Itand.png", Itand);
        //------------------------------------------------------------------------------------------------------------------------------
        //2. to find the contours from image "Itand";
        //------------------------------------------------------------------------------------------------------------------------------
        CvSeq *contours = 0;            //"contours" is a list of contour sequences, which is the core of "image_proc";
        CvSeq *first_contour = 0;       //always keep one copy of the beginning of this list, for further usage;
        CvMemStorage *mem_contours; 
        mem_contours = cvCreateMemStorage(0);
        
        //int n = cvFindContours(Itand, mem_contours, &contours, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
        //ROS_DEBUG("total contours: %d", n);
        
        CvContourScanner scanner = cvStartFindContours(Itand, mem_contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
                
        //-------------------------------------------------------------------------------------------------------------------------------
        //3. to filter noise at the boundary, and noise too small or too big;
        //-------------------------------------------------------------------------------------------------------------------------------
        contours = Filter_candidates(scanner);
        first_contour = contours;
        //-------------------------------------------------------------------------------------------------------------------------------
        //4. classify each remained candidates; visualize the markers detected; 
        //-------------------------------------------------------------------------------------------------------------------------------
        IplImage *contour_img = cvCreateImage(cvSize(src->width,src->height),IPL_DEPTH_8U, 3);
        cvCvtColor(Itand, contour_img, CV_GRAY2BGR);
        CvScalar ext_color;
        
        CvMoments cvm; 
        CvHuMoments cvHM;
        
        CvBox2D cvBox;
        CvMemStorage *mem_box;
        mem_box = cvCreateMemStorage(0);
        
        
		CvSeq *contour_poly;
		CvMemStorage *mem_poly;
		mem_poly = cvCreateMemStorage(0);

        int contour_num_in_this_image = 0;   // to calculate the number of contour candidates in one image;
        for (; contours != 0; contours = contours->h_next)
        {
            contour_num_in_this_image++;
            
            //This denotes how many pixels of one certain object contour;
            //ROS_DEBUG("total pixels %d", contours->total);
            ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 ); 

			//---------------------------------------------------------------------------------------------
			//--Extract features for marker classifcation;
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
            //---------------------------------------------------------------------------------------------
            
            int contour_class = classify_contour (contour_weight, contour_perimeter, cvHM, cvBox, approxPtNum);
            
            if(contour_class==-1){ROS_ERROR("NO CLASSIFICATION!!!");}
            if(contour_class==1||contour_class==2||contour_class==3)
            {   
                //-------------------------------------------------------------------------------------
                //5. calculate their pose relative to "base_link";
                //-------------------------------------------------------------------------------------
                
                //-------------------------------------------------------------------------------------
                //dedicated filtering 3rd criterion: four corners shouldn't appear at the boundary;
				CvPoint2D32f point[4];
				calc_cvBoxPoints(cvBox, point); 
				CvPoint pt[4];
				bool onBoundary = false;
				for (int i=0; i<4; i++)
				{
					pt[i].x = (int)point[i].x;
					pt[i].y = (int)point[i].y;
					if(!CheckPointInside(pt[i])) {onBoundary = true; break;}
				}
				//ignore current contour since it may not be complete; go on to the next on;
				if(onBoundary) continue; 
				//-------------------------------------------------------------------------------------
				
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
					
					markers_para.vec.push_back(marker_output);
				}

                vision_lane_detection::marker_info marker_output_2nd;
                marker_output_2nd.class_label = contour_class;
                marker_output_2nd.thetha = 3*M_PI;
                pose_contour_Extrinsic(contours, contour_class, projection_matrix, marker_output_2nd);
                if(marker_output_2nd.thetha!=3*M_PI){markers_para_2nd.vec.push_back(marker_output_2nd);}
            }
            /*
            else if(contour_class==4)
            {
				float height =  cvBox.size.height;
				float width = cvBox.size.width;
				float center_x=  cvBox.center.x;
				float center_y=  cvBox.center.y;
				//ROS_INFO("center.x, center.y, height, width: %lf, %lf, %lf, %lf, %lf", center_x, center_y, height, width);
				//DrawBox(cvBox,color_img,ext_color);
				
				//------------------------------------------------------------------------------------------------------------------
                //dedicated filtering 3rd criterion for continuous lane: 
                //1)no one side of its bounding box touches two side bounds;
				CvPoint2D32f point[4];
				calc_cvBoxPoints(cvBox, point); 
				CvPoint pt[4];
				bool onSideBounds = false;
				for (int i=0; i<3; i++)
				{
					pt[i].x = (int)point[i].x;
					pt[i].y = (int)point[i].y;
					pt[i+1].x = (int)point[i+1].x;
					pt[i+1].y = (int)point[i+1].y;
					if(!CheckPointOffSideBounds(pt[i]) && !CheckPointOffSideBounds(pt[i+1])) 
					{onSideBounds = true; break;}
				}
				if(onSideBounds) continue; 
				
				//2)lane is long enough;
				if(max(height,width)<100) continue; 
				continuous_lane(contours, contour_img, ext_color);
			}
			*/
			 
			else {}
		}
        cvShowImage("contour_image",contour_img);
        //cvSaveImage("/home/baoxing/contour_img.png", contour_img);
        
        //-----------------------------------------------------------------------------------------------
        //only use when to extract training pictures;
        //once entering this loop, meaning at least one contour exist, save image as training sets;
        //-----------------------------------------------------------------------------------------------
        if(extract_image_ && contour_num_in_this_image>0)
        {                
			stringstream  name_string;       
			int stored_serial= frame_serial/2;
			//pay attention, this path to the folder cannot be too long, or OpenCV will crash;
			name_string<<"/home/baoxing/images/frame"<<stored_serial<<".jpg";
			const char *output_name = name_string.str().c_str();

			if (!cvSaveImage(output_name, src))
			{               
				ROS_ERROR("Cannot save images");
				return;
			}
            frame_serial++; 
        }
    
        cvReleaseMemStorage(&mem_contours);
        cvReleaseMemStorage(&mem_box);
        cvReleaseMemStorage(&mem_poly);
        
        cvWaitKey(50);

        cvReleaseImage(&It);
        cvReleaseImage(&Iat);
        cvReleaseImage(&Itand);
        cvReleaseImage(&contour_img);
    }
    
    //Use libsvm to classify each contour; input features are of type "CvHuMoments" and "CvBox2D";
    //More and better features may be used in the future;
    int image_proc::classify_contour(double weight_input, double perimeter_input, 
									 CvHuMoments &HM_input, CvBox2D &Box_input, int polyNum_input)
    {
        int class_label = -1;
        float boxAngle  =   Box_input.angle;
        float height    =   Box_input.size.height;
        float width     =   Box_input.size.width;
        ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf, %lf", HM_input.hu1, HM_input.hu2, HM_input.hu3, HM_input.hu4, HM_input.hu5, HM_input.hu6, HM_input.hu7);
        ROS_INFO("boxAngle, height, width: %lf, %lf, %lf", boxAngle, height, width);
        //Data scaling here enables svm to get better accuracy;
        
        //keep accordance with "data_formating";
        double long_side_scaled = (max(width, height))/100.0;
        double short_side_scaled = (min(width, height))/100.0;
        
        struct svm_node *x;
        int max_nr_attr = 13;
        x = (struct svm_node *) malloc(max_nr_attr*sizeof(struct svm_node));
        x[0].index = 1;     
        x[1].index = 2;
        x[2].index = 3;
        x[3].index = 4;
        x[4].index = 5;
        x[5].index = 6;
        x[6].index = 7;
        x[7].index = 8;
        x[8].index = 9;
        x[9].index = 10;
        x[10].index = 11;
        x[11].index = 12;
        x[12].index = -1;
         
         // go to "data_formating" for the feature sequences;
        x[0].value = HM_input.hu1;
        x[1].value = HM_input.hu2;
        x[2].value = HM_input.hu3;
        x[3].value = HM_input.hu4;
        x[4].value = HM_input.hu5;
        x[5].value = HM_input.hu6;
        x[6].value = HM_input.hu7;
        x[7].value = short_side_scaled;
        x[8].value = long_side_scaled;
        x[9].value = weight_input;
        x[10].value = perimeter_input;
        x[11].value = polyNum_input;
        
        for(int i=0; i<12; i++) x[i].value = output(x[i].index, x[i].value);
        
        ROS_INFO("try to predict");
        class_label = svm_predict(svm_model_,x);
        ROS_INFO("predict finished");
        free(x);
        return class_label;
    }
    
    CvSeq* image_proc::Filter_candidates (CvContourScanner &scanner)
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
            if(len_meter < CONTOUR_PERIMETER_THRESH)
            {
                cvSubstituteContour(scanner, NULL);
            }
            else
            {
                //2nd criterion: long side should exceed certain threshold;
                cvBox = cvMinAreaRect2(c, mem_box);
                float height =  cvBox.size.height;
                float width  =  cvBox.size.width;
                float long_side = max(height, width);
                if(long_side<LONG_SIDE_THRESH*scale_)
                {
                    cvSubstituteContour(scanner, NULL);
                }
                else
                {
					//this criterion only applies to discrete markers, while not to continuous lanes;
					//shift to their dedicated part;
					
					/*
                    //3rd criterion: four corners shouldn't appear at the boundary;
                    CvPoint2D32f point[4];
                    calc_cvBoxPoints(cvBox, point); 
                    CvPoint pt[4];
                    for (int i=0; i<4; i++)
                    {
                        pt[i].x = (int)point[i].x;
                        pt[i].y = (int)point[i].y;
                        if(!CheckPointInside(pt[i])) 
                        {
                            cvSubstituteContour(scanner, NULL);
                            break;
                        }
                    }
                    */ 
                }   
            }                       
        }
        CvSeq *contours = cvEndFindContours(&scanner);
        cvReleaseMemStorage(&mem_box);
        return contours;
    }
    
    
    void image_proc::pose_contour(CvSeq *contour_raw, CvMoments &cvm, vision_lane_detection::marker_info &marker_para)
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
        
        if(contours->total<5){ROS_INFO("this marker is bad! no angle information;");}
        else
        {
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
					
					//ROS_INFO("use_front_only");
				}
				else if (!fd_satisfy && bd_satisfy)
				{
					line2_front_serial = line_front;
					line2_back_serial  = line_back;
					line1_front_serial = backward_line_back;
					line1_back_serial  = backward_line_front;
					
					//ROS_INFO("use_back_only");
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
					if(abs_delt>M_PI_2){ROS_INFO("noisy situation, do not provide angle");}
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
        cvt_pose_baselink(marker_para);
        ROS_INFO("---------marker %lf, %lf, %lf----------\n", marker_para.x, marker_para.y, marker_para.thetha);
        cvReleaseMemStorage(&mem_poly);
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 2nd method to pose the contour by utilizing corresponding points, which is expected to be more accurate;
    // this method doesn't rely on the assumption that the ground is always flat, which is more realistic;
    // the core of this method is the OpenCV funciton "cvFindExtrinsicCameraParams2";
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    void image_proc::pose_contour_Extrinsic(CvSeq *contour_raw, int contour_class, CvMat* projection_matrix, vision_lane_detection::marker_info &marker_para)
    {
        CvSeq *contours;
        CvMemStorage *mem_poly;
        mem_poly = cvCreateMemStorage(0);
        contours = cvApproxPoly( contour_raw, sizeof(CvContour), mem_poly, CV_POLY_APPROX_DP, 2, 0 );
        
        //////////////////////////////////////////////////////////////
        //// find 4 corresponding points;
        //////////////////////////////////////////////////////////////
        if(contours->total<5){ROS_INFO("this marker is bad! no corresponding points! no pose information!");}
        else
        {
            std::vector <CvPoint> vertices;
            
            for(int i=0; i<contours->total; i++)
            {
                CvPoint* p = (CvPoint*)cvGetSeqElem(contours, i);
                vertices.push_back(*p);
            }
            
            float vertix_x_new, vertix_y_new;
            float vertix_x_old, vertix_y_old;
            float distance;
            std::vector <float> distance_vec;
            
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
				
				//line1 is the line at the left side of the marker;
				if(fd_satisfy && !bd_satisfy)
				{
					line1_front_serial = line_back;
					line1_back_serial  = line_front;
					line2_front_serial = forward_line_front;
					line2_back_serial  = forward_line_back;
					
					//ROS_INFO("use_front_only");
				}
				else if (!fd_satisfy && bd_satisfy)
				{
					line2_front_serial = line_front;
					line2_back_serial  = line_back;
					line1_front_serial = backward_line_back;
					line1_back_serial  = backward_line_front;
					
					//ROS_INFO("use_back_only");
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
					
					CvMat* dstMarkerPts = cvCreateMat(4,1,CV_32FC2);
					CvMat* srcMarkerPts = cvCreateMat(4,1,CV_32FC2);
					
					int row = 0;
					float* ptr = (float*)(dstMarkerPts->data.ptr + row * dstMarkerPts->step);
					ptr[0] = line1_frontd.x;
					ptr[1] = line1_frontd.y;
					row = 1;
					ptr = (float*)(dstMarkerPts->data.ptr + row * dstMarkerPts->step);
					ptr[0] = line1_backd.x;
					ptr[1] = line1_backd.y;
					row = 2;
					ptr = (float*)(dstMarkerPts->data.ptr + row * dstMarkerPts->step);
					ptr[0] = line2_frontd.x;
					ptr[1] = line2_frontd.y;
					row = 3;
					ptr = (float*)(dstMarkerPts->data.ptr + row * dstMarkerPts->step);
					ptr[0] = line2_backd.x;
					ptr[1] = line2_backd.y;
					
					// 4 points in "ipm_image" projected back raw "src_image"; 
					cvPerspectiveTransform(dstMarkerPts, srcMarkerPts, projection_matrix);
					
					tf::Pose markerPose_base;
					corresPoints_extrinCalib(contour_class, srcMarkerPts, markerPose_base);
					marker_para.x = markerPose_base.getOrigin().x();
					marker_para.y = markerPose_base.getOrigin().y();
					double tmp;
					markerPose_base.getBasis().getEulerYPR(marker_para.thetha, tmp, tmp);
					
					cvReleaseMat(&dstMarkerPts);
					cvReleaseMat(&srcMarkerPts);
				}
            }
        }
        cvReleaseMemStorage(&mem_poly);
    }
    
    void image_proc::corresPoints_extrinCalib(int contour_class, CvMat* srcMarkerPts, tf::Pose & markerPose_base)
    {
		CvMat* trans_vec = cvCreateMat(3,1,CV_32FC1);
		CvMat* rot_vec = cvCreateMat(3,1,CV_32FC1);
		CvMat* rot_matrix = cvCreateMat(3,3,CV_32FC1);
		CvMat* marker_gndPts;
		
		if(contour_class == 1) marker_gndPts = M1_gndPts_;
		else if(contour_class == 2) marker_gndPts = M2_gndPts_;
		else if(contour_class == 2) marker_gndPts = M3_gndPts_;
		else {ROS_ERROR("contour class unexpected, please check!"); return;}
		
		cvFindExtrinsicCameraParams2(marker_gndPts, srcMarkerPts, intrinsic_A_, distortion_coeffs_, rot_vec, trans_vec);
		float x_trans, y_trans, z_trans;							
		float *ptr = (float*)(trans_vec->data.ptr + 0 * trans_vec->step);  x_trans = ptr[0];
		ptr = (float*)(trans_vec->data.ptr + 1 * trans_vec->step);  y_trans = ptr[0];
		ptr = (float*)(trans_vec->data.ptr + 2 * trans_vec->step);  z_trans = ptr[0];
		
        cvRodrigues2(rot_vec, rot_matrix, NULL);
							
		int row = 0; ptr = (float*)(rot_matrix->data.ptr + row * rot_matrix->step);
		float M11_rot = ptr[0];			
		row = 1; ptr = (float*)(rot_matrix->data.ptr + row * rot_matrix->step);
		float M21_rot = ptr[0];					
		row = 2; ptr = (float*)(rot_matrix->data.ptr + row * rot_matrix->step);
		float M31_rot = ptr[0]; float M32_rot = ptr[1]; float M33_rot = ptr[2];
		
		//http://planning.cs.uiuc.edu/node102.html#eqn:yprmat
		double yaw, pitch, roll;
		yaw	  = atan2f(M21_rot, M11_rot);
		pitch = atan2f(- M31_rot, sqrtf(M32_rot*M32_rot+M33_rot*M33_rot));
		roll  = atan2f(M32_rot, M33_rot);
		tf::Pose cameraInbaselink (tf::createQuaternionFromRPY(0.0,0.0,0.0), btVector3(1.795, 0.0, 0.97));				
		tf::Pose imageInCamerabase (tf::createQuaternionFromRPY(-1.5707963267949, 0.0, -1.5707963267949), btVector3(0.0, 0.0, 0.0));
		tf::Pose markerCoordinate_inCam (tf::createQuaternionFromRPY(roll,pitch,yaw), btVector3(x_trans,y_trans, z_trans));
		markerPose_base = cameraInbaselink * imageInCamerabase * markerCoordinate_inCam;
		
		cvReleaseMat(&trans_vec);
		cvReleaseMat(&rot_vec);
		cvReleaseMat(&rot_matrix);
	}
    
   /*
	void image_proc::continuous_lane(CvSeq *contours, IplImage *contour_img, CvScalar ext_color)
	{
		std::vector <TPoint2D> contour_points;
		for(int i=0; i<contours->total; i++)
		{
			CvPoint* p = (CvPoint*)cvGetSeqElem(contours, i);
			TPoint2D pt_tmp;
			pt_tmp.x = p -> x;
			pt_tmp.y = p -> y;
			contour_points.push_back(pt_tmp);
		}
		
		//-------------------------------------------------------------------------------------------------------------------------
		//preparation work: to reduce the polygon of the contour into a thin single line;
		//-------------------------------------------------------------------------------------------------------------------------
		ROS_INFO("step1");
		std::vector <TPoint2D> fused_points;
		if(contour_points.size()<4) return;
		
		bool publish =  false;
		for(int serial_tmp =0; serial_tmp < contour_points.size(); serial_tmp++)
		{
			if(publish) printf("serial_tmp %d\t  (%3f, %3f)\t", serial_tmp, contour_points[serial_tmp].x, contour_points[serial_tmp].y);
			
			int prev2_serial= serial_tmp-2;
			int next2_serial= serial_tmp+2;
			if(prev2_serial <0) prev2_serial = prev2_serial + contour_points.size();
			if(next2_serial >= contour_points.size()) next2_serial = next2_serial - contour_points.size();
			
			ASSERT_(prev2_serial<contour_points.size());
			ASSERT_(next2_serial<contour_points.size());

			if(contour_points[prev2_serial] == contour_points[next2_serial]) continue;
			TLine2D line_tmp(contour_points[prev2_serial], contour_points[next2_serial]);
			
			TPoint2D mid_point;
			mid_point.x = (contour_points[prev2_serial].x+contour_points[next2_serial].x)/2.0;
			mid_point.y = (contour_points[prev2_serial].y+contour_points[next2_serial].y)/2.0;
			
			if(publish) printf("mid point: %3f, %3f\t", mid_point.x, mid_point.y);
			
			double a_tmp = line_tmp.coefs[1];
			double b_tmp = -line_tmp.coefs[0];
			double c_tmp = -(a_tmp*mid_point.x+b_tmp*mid_point.y);
			
			if(publish) printf("line parameter: %3f, %3f, %3f\t", a_tmp, b_tmp, c_tmp);
			
			TLine2D line_perpendicular_tmp(a_tmp,b_tmp,c_tmp);

			double nearest_distance = 100.0;
			int nearest_serial = 0;
			TPoint2D fused_pt;
			
			for(int serial_tmpp = 0; serial_tmpp < contour_points.size(); serial_tmpp ++)
			{
				double tmp_distance = line_perpendicular_tmp.distance(contour_points[serial_tmpp]);
				bool another_side = abs(serial_tmpp-serial_tmp)>10;
				if(tmp_distance<nearest_distance && another_side){nearest_distance=tmp_distance;nearest_serial = serial_tmpp;}
			}
			if(publish) printf("nearest point %d\t, (%3f, %3f)\t, distance %3f\n", nearest_serial, contour_points[nearest_serial].x, contour_points[nearest_serial].y, nearest_distance);
			
			if(nearest_distance<20.0)
			{
				fused_pt.x = (contour_points[nearest_serial].x + contour_points[serial_tmp].x)/2;
				fused_pt.y = (contour_points[nearest_serial].y + contour_points[serial_tmp].y)/2;
				
				bool point_exist = false;
				for(size_t fuse_serial = 0; fuse_serial < fused_points.size(); fuse_serial++)
				{
					if(fused_points[fuse_serial]==fused_pt){point_exist = true; break;}
				}
				if(!point_exist)fused_points.push_back(fused_pt);
			}
			//publish =  false;
		}
		
		ROS_INFO("step2");
		vector_double xs,ys;
		for (size_t i=0;i<fused_points.size();i++)
		{	
			const double xx = fused_points[i].x;
			const double yy = fused_points[i].y;
			xs.push_back(xx);
			ys.push_back(yy);
			//cvCircle( contour_img, cvPoint(fused_points[i].x,fused_points[i].y), 2, ext_color, 1);
		}
		ROS_INFO("fused points number %ld", xs.size());
		//--------------------------------------------------------------------------------------------------------------
		
		//--------------------------------------------------------------------------------------------------------------
		// non-lane contours will take much time to process; 
		// add pre-filtering step for those nonlane contours;
		//--------------------------------------------------------------------------------------------------------------
		
		vector<pair<mrpt::vector_size_t, parabola> >   detectedLines;
		//temporarily the unit is still pixel;
		const double DIST_THRESHOLD = 5;
		ransac_detect_parabolas(xs,ys,detectedLines,DIST_THRESHOLD, 100 );
		
		//--------------------------------------------------------------------------------------------------------------
		//try to visualize the detected parabola;
		//--------------------------------------------------------------------------------------------------------------
		for (vector<pair<mrpt::vector_size_t,parabola> >::iterator p=detectedLines.begin();p!=detectedLines.end();++p)
		{
			CvScalar random_color;
			random_color = CV_RGB( rand()&255, rand()&255, rand()&255 );
			if(p->second.coefs[3]==1.0)
			{
				for(unsigned int yPixel=0; yPixel<360; yPixel++)
				{
					unsigned int xPixel = p->second.coefs[0]*yPixel*yPixel+p->second.coefs[1]*yPixel+p->second.coefs[2];
					cvCircle( contour_img, cvPoint(xPixel,yPixel), 2, ext_color, 1);
				}
			}
			else
			{
				for(unsigned int xPixel=0; xPixel<640; xPixel++)
				{
					unsigned int yPixel = p->second.coefs[0]*xPixel*xPixel+p->second.coefs[1]*xPixel+p->second.coefs[2];
					cvCircle( contour_img, cvPoint(xPixel,yPixel), 2, ext_color, 1);
				}
			}
		}
		//---------------------------------------------------------------------------------------------------------------
    }
    */ 
    
    void image_proc::cvt_pose_baselink(vision_lane_detection::marker_info &marker_para)
    {
        double center_x = (RECT_P0_X + RECT_P2_X)/2.0 + DIS_CAM_BASE_X;
        double center_y = 0.0;
        
        double center_pix_x = 320;
        double center_pix_y = 180;
        
        double delt_x =  -(marker_para.y - center_pix_y)/scale_;
        double delt_y =  -(marker_para.x - center_pix_x)/scale_;
        marker_para.x = center_x + delt_x;
        marker_para.y = center_y + delt_y;
    }
    
    bool image_proc::CheckPointInside(CvPoint pt_para)
    {
        bool out_flag1 = (pt_para.y + BOUNDARY_MARGIN >= corners_[0].y);
        bool out_flag2 = (pt_para.y - BOUNDARY_MARGIN <= 0);
        bool out_flag3 = (pt_para.y + BOUNDARY_MARGIN >= para_A1_*pt_para.x+para_C1_);
        bool out_flag4 = (pt_para.y + BOUNDARY_MARGIN >= para_A2_*pt_para.x+para_C2_);
        if(out_flag1||out_flag2||out_flag3||out_flag4) return false;
        else return true;
    }
    
    
    bool image_proc::CheckPointOffSideBounds(CvPoint pt_para)
    {
        bool out_flag3 = (pt_para.y > para_A1_*pt_para.x+para_C1_);
        bool out_flag4 = (pt_para.y > para_A2_*pt_para.x+para_C2_);
        if(out_flag3||out_flag4) return false;
        else return true;
    }
    
    void image_proc::restore_scalefile(string filename, double* &feature_min, double* &feature_max, int &feature_index)
	{
		int idx, c;
		FILE *fp_restore;
		const char *restore_filename = filename.c_str();
		double y_max = -DBL_MAX;
		double y_min = DBL_MAX;
		double y_lower,y_upper;
		int max_index=0;

		fp_restore = fopen(restore_filename,"r");
		if(fp_restore==NULL)
		{
			fprintf(stderr,"can't open file %s\n", restore_filename);
			exit(1);
		}
		cout<<"File opened"<<endl;
		c = fgetc(fp_restore);
		if(c == 'y')
		{
			readline(fp_restore);
			readline(fp_restore);
			readline(fp_restore);
		}
		cout<<readline(fp_restore)<<endl;
		cout<<readline(fp_restore)<<endl;
		cout<<"Retrieving maximum index"<<endl;
		while(fscanf(fp_restore,"%d %*f %*f\n",&idx) == 1)
			max_index = max(idx,max_index);
		rewind(fp_restore);
		cout<<"Max index retrieved "<<max_index<<endl;
		feature_max = (double *)malloc((max_index+1)* sizeof(double));
		feature_min = (double *)malloc((max_index+1)* sizeof(double));

		double fmin, fmax;
		int y_scaling = 0;
		if((c = fgetc(fp_restore)) == 'y')
		{
			fscanf(fp_restore, "%lf %lf\n", &y_lower, &y_upper);
			fscanf(fp_restore, "%lf %lf\n", &y_min, &y_max);
			y_scaling = 1;
		}
		else
			ungetc(c, fp_restore);

		if (fgetc(fp_restore) == 'x') {
			cout<<"got x"<<endl;
			fscanf(fp_restore, "%lf %lf\n", &lower_, &upper_);
			while(fscanf(fp_restore,"%d %lf %lf\n",&idx,&fmin,&fmax)==3)
			{
				if(idx<=max_index)
				{
					feature_min[idx] = fmin;
					feature_max[idx] = fmax;
				}
			}
		}
		feature_index = max_index;
		fclose(fp_restore);
		cout<<"File closed"<<endl;
	}
	
	char* image_proc::readline(FILE *input)
	{
		int len;
		char *line = NULL;
		int max_line_len = 1024;
		line = (char *) malloc(max_line_len*sizeof(char));
		if(fgets(line,max_line_len,input) == NULL)
			return NULL;

		while(strrchr(line,'\n') == NULL)
		{
			max_line_len *= 2;
			line = (char *) realloc(line, max_line_len);
			len = (int) strlen(line);
			if(fgets(line+len,max_line_len-len,input) == NULL)
				break;
		}
		return line;
	}

	double image_proc::output(int index, double value)
	{
		/* skip single-valued attribute */
		if(feature_max_[index] == feature_min_[index])
			return value;

		if(value == feature_min_[index])
			value = lower_;
		else if(value == feature_max_[index])
			value = upper_;
		else
			value = lower_ + (upper_-lower_) *
				(value-feature_min_[index])/
				(feature_max_[index]-feature_min_[index]);

		if(value != 0)
		{
			//printf("%d:%g ",index, value);
		}
		return value;
	}

    image_proc::~image_proc()
    {
        cvDestroyWindow("It_image");
        cvDestroyWindow("Iat_image");
        cvDestroyWindow("binary_image");
        cvDestroyWindow("contour_image");
        cvDestroyWindow("HistogramEqualized_image");
        cvReleaseMat(&intrinsic_A_);
        cvReleaseMat(&distortion_coeffs_);
        cvReleaseMat(&M1_gndPts_);
        cvReleaseMat(&M2_gndPts_);
        cvReleaseMat(&M3_gndPts_);
    }
};
