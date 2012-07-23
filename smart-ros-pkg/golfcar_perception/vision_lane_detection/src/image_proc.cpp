#include "image_proc.h"

namespace golfcar_vision{
  
    image_proc::image_proc():
        init_flag_(false),
        extract_image_(false)
    { 
      cvNamedWindow("It_image");
      cvNamedWindow("Iat_image");
      cvNamedWindow("binary_image");
      cvNamedWindow("contour_image");
      
      string svm_model_file;

      //the name cannot be too long, or it cannot load;
      svm_model_file = "/home/baoxing/workspace/data_and_model/lane_detection.model";
      svm_model_ = svm_load_model(svm_model_file.c_str());
      //the following line can help to check where the model has been loaded or not;
      cout<<" SVM loaded, type = "<< svm_model_->param.svm_type <<endl;
    }
  
    void image_proc::Extract_Markers (IplImage* src, float scale, vision_lane_detection::markers_info &markers_para, int &frame_serial)
    {
        markers_para.vec.clear();
        //initiation, first calculate four corners used to filter noise;
        if(!init_flag_)
        {
            scale_ = scale;
            init_flag_ = true;
            
            //remember, image_coordinate is quite different from other coordinates;
            corners_[0].x = int(src->width/2 - RECT_P0_Y * scale_);    
            corners_[0].y = src->height;
            corners_[1].x = int(src->width/2 - RECT_P1_Y * scale_);
            corners_[1].y = src->height;
            corners_[2].x = src->width;   
            corners_[2].y = 0;
            corners_[3].x = 0;   
            corners_[3].y = 0;
            
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
        
        
        int contour_num_in_this_image = 0;   // to calculate the number of contour candidates in one image;
        for (; contours != 0; contours = contours->h_next)
        {
            contour_num_in_this_image++;
            
            //This denotes how many pixels of one certain object contour;
            //ROS_DEBUG("total pixels %d", contours->total);
            
            ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 ); 
            
            /*
            //Polygon approximation: get vertices of the contour;
            //pay attention here: need allocate another memory to store the new contours;
            contours = cvApproxPoly( contours, sizeof(CvContour), mem_contours, CV_POLY_APPROX_DP, 10, 0 );
            cvDrawContours(contour_img, contours, ext_color, CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));
            //total number of vertices after approximation;
            ROS_INFO("total %d", contours->total);
            */
            
            /*
            for(int i=0; i<contours->total; i++)
            {
                CvPoint* p = (CvPoint*)cvGetSeqElem(contours, i);
                printf("(%d, %d)\n", p->x, p->y);
            }
            */ 

            cvContourMoments(contours, &cvm);
            cvGetHuMoments(&cvm, &cvHM);
            cvBox = cvMinAreaRect2(contours, mem_box);
            

            int contour_class = classify_contour (cvHM, cvBox);
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
				}
                markers_para.vec.push_back(marker_output);
            }
            
            //else if(){}
            else 
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
		}
        cvShowImage("contour_image",contour_img);
        //cvSaveImage("/home/baoxing/contour_img.png", contour_img);
        
        //-----------------------------------------------------------------------------------------------
        //only use when to extract training pictures;
        //once entering this loop, meaning at least one contour exist, save image as training sets;
        //-----------------------------------------------------------------------------------------------
        if(extract_image_ && contour_num_in_this_image>0)
        {
            //here "%2" is just to adjust(reduce) the frequency of image saving;
            if(frame_serial%2==0)
            {
                
                stringstream  name_string;       
                int stored_serial= frame_serial/2;
                //pay attention, this path to the folder cannot be too long, or OpenCV will crash;
                name_string<<"/home/marskahn/training/frame"<<stored_serial<<".jpg";
                const char *output_name = name_string.str().c_str();

                if (!cvSaveImage(output_name, src))
                {               
                    ROS_ERROR("Cannot save images");
                    return;
                }
            }
            frame_serial++; 
        }
    
        cvReleaseMemStorage(&mem_contours);
        cvReleaseMemStorage(&mem_box);
        cvWaitKey(50);

        cvReleaseImage(&It);
        cvReleaseImage(&Iat);
        cvReleaseImage(&Itand);
        cvReleaseImage(&contour_img);
    }
    
    //Use libsvm to classify each contour; input features are of type "CvHuMoments" and "CvBox2D";
    //More and better features may be used in the future;
    int image_proc::classify_contour(CvHuMoments &HM_input, CvBox2D &Box_input )
    {
        int class_label = -1;
        float boxAngle  =   Box_input.angle;
        float height    =   Box_input.size.height;
        float width     =   Box_input.size.width;
        ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf, %lf", HM_input.hu1, HM_input.hu2, HM_input.hu3, HM_input.hu4, HM_input.hu5, HM_input.hu6, HM_input.hu7);
        ROS_INFO("boxAngle, height, width: %lf, %lf, %lf", boxAngle, height, width);
        //Data scaling here enables svm to get better accuracy;
        double long_side_scaled = (max(width, height))/100.0;
        double short_side_scaled = (min(width, height))/100.0;
        struct svm_node *x;
        int max_nr_attr = 10;
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
        x[9].index = -1;
         
        x[0].value = HM_input.hu1;     
        x[1].value = HM_input.hu2;
        x[2].value = HM_input.hu3;
        x[3].value = HM_input.hu4;
        x[4].value = HM_input.hu5;
        x[5].value = HM_input.hu6;
        x[6].value = HM_input.hu7;
        x[7].value = short_side_scaled;
        x[8].value = long_side_scaled;
        
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
            double len_meter = len_pixel * scale_;
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
            
            unsigned int longest_serial, sec_longest_serial;
            float temp_distance1, temp_distance2;
            longest_serial = find_longest_distance(distance_vec);
            temp_distance1 = distance_vec[longest_serial];
            distance_vec[longest_serial]=0.0;
            sec_longest_serial = find_longest_distance(distance_vec);
            temp_distance2 = distance_vec[sec_longest_serial];
            
            if(temp_distance1<50.0||temp_distance2<50.0){ROS_INFO("this marker is not long enough! no angle information;");}
            else
            {
                //to determine the heading side;
                unsigned int serial1 = min(longest_serial, sec_longest_serial);
                unsigned int serial2 = max(longest_serial, sec_longest_serial);
                unsigned int delt_serial_forward_path   = serial2 - serial1 -1;
                unsigned int delt_serial_backward_path  = serial1 + distance_vec.size() -1 -serial2;
                
                CvPoint2D32f line1_front, line1_back, line2_front, line2_back;
                
                CvPoint line1_frontd, line1_backd, line2_frontd, line2_backd;
                //the shorter path is the bottom of the marker;
                if(delt_serial_forward_path <= delt_serial_backward_path)
                {
                    if(serial1 == 0)
                    {
                        line1_front = centered_vertices.back();
                        line1_frontd = vertices.back();
                    }
                    else
                    {
                        line1_front  = centered_vertices[serial1-1];
                        line1_frontd  = vertices[serial1-1];
                    }
                    line1_back  = centered_vertices[serial1];
                    line2_front = centered_vertices[serial2];
                    line2_back  = centered_vertices[serial2-1];
                    
                    line1_backd  = vertices[serial1];
                    line2_frontd = vertices[serial2];
                    line2_backd  = vertices[serial2-1];
                    
                }
                else
                {
                    //invert the sequence of the above senario;
                    if(serial1 == 0)
                    {
                        line1_back = centered_vertices.back();
                        line1_backd = vertices.back();
                    }
                    else
                    {
                        line1_back  = centered_vertices[serial1-1];
                        line1_backd  = vertices[serial1-1];
                    }
                    line1_front  = centered_vertices[serial1];
                    line2_back = centered_vertices[serial2];
                    line2_front  = centered_vertices[serial2-1];
                    
                    line1_frontd  = vertices[serial1];
                    line2_backd = vertices[serial2];
                    line2_frontd  = vertices[serial2-1];
                }
                
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
        cvt_pose_baselink(marker_para);
        ROS_INFO("---------marker %lf, %lf, %lf----------\n", marker_para.x, marker_para.y, marker_para.thetha);
        cvReleaseMemStorage(&mem_poly);
    }
    
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
    
    void image_proc::cvt_pose_baselink(vision_lane_detection::marker_info &marker_para)
    {
        double center_x = (RECT_P0_X + RECT_P2_X)/2.0 + DIS_CAM_BASE_X;
        double center_y = 0.0;
        
        double center_pix_x = 360;
        double center_pix_y = 180;
        
        double delt_x =  -(marker_para.y - center_pix_y)/scale_;
        double delt_y =  -(marker_para.x - center_pix_x)/scale_;
        marker_para.x = center_x + delt_x;
        marker_para.y = center_y + delt_y;
    }
    
    bool image_proc::CheckPointInside(CvPoint pt_para)
    {
        bool out_flag1 = pt_para.y+5 > corners_[0].y;
        bool out_flag2 = pt_para.y-BOUNDARY_MARGIN < 0;
        bool out_flag3 = pt_para.y+BOUNDARY_MARGIN > para_A1_*pt_para.x+para_C1_;
        bool out_flag4 = pt_para.y+BOUNDARY_MARGIN > para_A2_*pt_para.x+para_C2_;
        
        if(out_flag1||out_flag2||out_flag3||out_flag4) return false;
        else return true;
    }
    
    
    bool image_proc::CheckPointOffSideBounds(CvPoint pt_para)
    {
        bool out_flag3 = pt_para.y+BOUNDARY_MARGIN > para_A1_*pt_para.x+para_C1_;
        bool out_flag4 = pt_para.y+BOUNDARY_MARGIN > para_A2_*pt_para.x+para_C2_;
        if(out_flag3||out_flag4) return false;
        else return true;
    }
      
    image_proc::~image_proc()
    {
        cvDestroyWindow("It_image");
        cvDestroyWindow("Iat_image");
        cvDestroyWindow("binary_image");
        cvDestroyWindow("contour_image");
    }
};
