//20120410
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "lane_marker_common.h"
#include <sensor_msgs/PointCloud.h>
#include "ransac/ransac_parabola.h"

using namespace golfcar_vision;
class line_segment
   { 
	public:
	
	//Ax+By+C=0;
	double para_A;
	double para_B;
    double para_C;
    double distance_rho; //here rho is the same with that of houghline.
    double thetha;      //thetha is not the same with that of houghline. 
    
    CvPoint point0;
    CvPoint point1;
   };

void point_to_lines(CvPoint* point_para, line_segment & line_segment_para)
{
	     double delt_x, delt_y;
         delt_x= point_para[1].x-point_para[0].x;
         delt_y= point_para[1].y-point_para[0].y;
         
         ROS_INFO("point0, point1: (%d,%d),(%d,%d); delt_x, delt_y:(%5f,%5f)", point_para[0].x, point_para[0].y, point_para[1].x, point_para[1].y,delt_x,delt_y );
       
         // delt_x and delt_y will not be 0 at the same time.
         if(delt_x==0&&delt_y!=0)
         {
			 line_segment_para.para_A=-1;
			 line_segment_para.para_B=0;
			 line_segment_para.para_C=point_para[1].x;
			 line_segment_para.distance_rho=point_para[1].x;
			 line_segment_para.point0=point_para[0];
			 line_segment_para.point1=point_para[1];
			 line_segment_para.thetha=1.5707963;
	     }
	     
         else if(delt_y==0&&delt_x!=0)
         {
			 line_segment_para.para_A=0;
			 line_segment_para.para_B=-1;
			 line_segment_para.para_C=point_para[1].y;
			 line_segment_para.distance_rho=point_para[1].y;
			 line_segment_para.point0=point_para[0];
			 line_segment_para.point1=point_para[1];
			 line_segment_para.thetha=0;
	     }  
	     
	     else if(delt_x!=0&&delt_y!=0)
	     {
			 line_segment_para.para_A=delt_y/delt_x;
			 line_segment_para.para_B=-1;
			 line_segment_para.para_C=point_para[1].y-line_segment_para.para_A*point_para[1].x;
			 
			 double cos_value;
			 double cot_value, temp_value;
			 line_segment_para.thetha=atan(delt_y/delt_x);
			 cos_value=cos(line_segment_para.thetha);
			 line_segment_para.distance_rho=line_segment_para.para_C*cos_value;
			 line_segment_para.thetha=atan(delt_y/delt_x);
			 line_segment_para.point0=point_para[0];
			 line_segment_para.point1=point_para[1];
		 }
		 
		 else{ROS_INFO("error, two points overlap.");} 
}


void draw_lines(std::vector<line_segment> line_segments_para, IplImage* color_dst_para)
{
	for(std::vector<line_segment>::size_type ic=0; ic!=line_segments_para.size(); ic++)
	{
		CvPoint line_para[2];
		line_para[0]=line_segments_para[ic].point0;
		line_para[1]=line_segments_para[ic].point1;
		
		if(ic==0)
		{cvLine( color_dst_para, line_para[0], line_para[1], CV_RGB(255,0,0), 1, 8 );}
		else if(ic==1)
		{cvLine( color_dst_para, line_para[0], line_para[1], CV_RGB(0,255,0), 1, 8 );}
		else if(ic==2)
		{cvLine( color_dst_para, line_para[0], line_para[1], CV_RGB(0,0,255), 1, 8 );}
		else if(ic==3)
		{cvLine( color_dst_para, line_para[0], line_para[1], CV_RGB(255,0,255), 1, 8 );}
		else if(ic==4)
		{cvLine( color_dst_para, line_para[0], line_para[1], CV_RGB(255,255,255), 1, 8 );}
		else if(ic==5)
		{cvLine( color_dst_para, line_para[0], line_para[1], CV_RGB(255,255,0), 1, 8 );}
		else {cvLine( color_dst_para, line_para[0], line_para[1], CV_RGB(0,255,255), 1, 8 );}
    }
}

int main(int argc, char** argv) 
{
	std::vector<line_segment> line_segments;
	
    IplImage *Igray=0, *It = 0, *Iat = 0, *Itand = 0;
    IplImage *canny=0, *color_dst = 0;
    IplImage *erode = 0, *dilate = 0;
    
    if(argc != 2){return -1;}
    //Command line
    //int lower_canny = atoi(argv[1]);
    //int upper_canny = atoi(argv[2]);
    //int aperture = atoi(argv[3]);
    //Read in gray image
    if((Igray = cvLoadImage( argv[1], CV_LOAD_IMAGE_GRAYSCALE)) == 0){
    return -1;}
    // Create the grayscale output images
    It = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 1);
    Iat = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 1);
    Itand = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 1);
    //canny = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 1);
    //color_dst = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 3);
    //erode = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 1);
    //dilate = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 1);
    
    //Threshold
    cvThreshold(Igray,It,BINARY_THRESH,255,CV_THRESH_BINARY);
    cvAdaptiveThreshold(Igray, Iat, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, BLOCK_SIZE, OFFSET);
    
    cvAnd(It, Iat, Itand);
    cvNamedWindow("Threshold And",1);   
    
    cvNamedWindow("Raw",1);
    //cvNamedWindow("Threshold",1);
    //cvNamedWindow("Adaptive Threshold",1);
    cvShowImage("Raw",Igray);
    //cvShowImage("Threshold",It);
    //cvShowImage("Adaptive Threshold",Iat);
    cvShowImage("Threshold And",Itand);
    
    /*
    cvDilate(Itand,dilate,NULL,3);
    cvNamedWindow("dilate",1);
    cvShowImage("dilate",dilate); 
    cvErode(dilate,erode,NULL,3);
    cvNamedWindow("erode",1);
    cvShowImage("erode",erode);      
    */
    
    //------------------------------------------------------------------------
    //tentative approach: 1) canny; 2) ransac on all points;
    //------------------------------------------------------------------------
    
    /*
     *     
    cvCanny(Itand, canny, lower_canny, upper_canny, aperture);
    cvNamedWindow("canny",1);
    cvShowImage("canny",canny);
    
    //--------to extract candidate points for RANSAC later;----
    int canny_height 	= canny -> height;
    int canny_width  	= canny -> width;
    int canny_step	 	= canny -> widthStep/sizeof(uchar);
    uchar * canny_data 	= (uchar*)canny ->imageData;
    sensor_msgs::PointCloud    candi_points;
    geometry_msgs::Point32	   point_tmp;
    
    for(int ih=0; ih < canny_height; ih++)
    {
		for(int iw=0; iw < canny_width; iw++)
		{
			if(canny_data[ih*canny_step+iw]> 0)
			{
				//cvCircle( color_dst, cvPoint(iw,ih), 6, CV_RGB(0,255,0), 2);
				point_tmp.x = iw;
				point_tmp.y = ih;
				point_tmp.z = 0.0;
				candi_points.points.push_back(point_tmp);
			}
		}
	}
	ROS_INFO("total candi points:%d",candi_points.points.size() );
	
	
	vector_double xs,ys;
	for (size_t i=0;i<candi_points.points.size();i++)
	{
		const double xx = candi_points.points[i].x;
		const double yy = candi_points.points[i].y;
		xs.push_back(xx);
		ys.push_back(yy);
	}
	
	vector<pair<size_t, parabola> >   detectedLines;
	//temporarily the unit is still pixel;
	const double DIST_THRESHOLD = 5;
	
	ransac_detect_parabolas(xs,ys,detectedLines,DIST_THRESHOLD, 400 );
	for (vector<pair<size_t,parabola> >::iterator p=detectedLines.begin();p!=detectedLines.end();++p)
	{
		CvScalar random_color;
		random_color = CV_RGB( rand()&255, rand()&255, rand()&255 );
		if(p->second.coefs[3]==1.0)
		{
			for(unsigned int yPixel=0; yPixel<360; yPixel++)
			{
				unsigned int xPixel = p->second.coefs[0]*yPixel*yPixel+p->second.coefs[1]*yPixel+p->second.coefs[2];
				cvCircle( color_dst, cvPoint(xPixel,yPixel), 2, random_color, 1);
			}
				 
		}
		else
		{
			for(unsigned int xPixel=0; xPixel<640; xPixel++)
			{
				unsigned int yPixel = p->second.coefs[0]*xPixel*xPixel+p->second.coefs[1]*xPixel+p->second.coefs[2];
				cvCircle( color_dst, cvPoint(xPixel,yPixel), 2, random_color, 1);
			}
			
		}
	}
    cvNamedWindow("color_dst",1);
    cvShowImage("color_dst",color_dst);
	cout << " " << detectedLines.size() << " lines detected." << endl;
    
    */
    
    //cvSaveImage("It.png", It);
    //cvSaveImage("Iat.png", Iat);
    //cvSaveImage("Itand.png", Itand);
    /*
    CvMemStorage* line_storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
	lines = cvHoughLines2( canny,
						   line_storage,
						   CV_HOUGH_PROBABILISTIC,
						   10,
						   CV_PI/180*5,
						   10,
						   50,
						   10 );
	ROS_INFO("line number %d",lines->total);
	for( int i = 0; i < lines->total; i++)
	{
		 CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
		 line_segment line_segment_temp;
		 point_to_lines(line, line_segment_temp);  
		 line_segments.push_back(line_segment_temp);

		 ROS_INFO("Number %d line segment finished: %3f, %3f %3f, (%d, %d), (%d, %d), (%5f, %5f)", i, line_segment_temp.para_A, line_segment_temp.para_B, line_segment_temp.para_C, line[0].x, line[0].y, line[1].x, line[1].y,line_segment_temp.distance_rho, line_segment_temp.thetha);         
	}
    draw_lines(line_segments, color_dst);
    cvNamedWindow("color_dst",1);
    cvShowImage("color_dst",color_dst);    
	*/

    CvMemStorage *mem;
    mem = cvCreateMemStorage(0);
    
    CvSeq *contours = 0;

    int n = cvFindContours(Itand, mem, &contours, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
    //int n = cvFindContours(Itand, mem, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
    //different total number when having different parameters;
    
    ROS_INFO("total contours: %d", n);
    
    IplImage *color_img = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 3);
    cvCvtColor(Itand, color_img, CV_GRAY2BGR);
    CvScalar ext_color;
    CvMoments cvm; 
    CvHuMoments cvHM;
    CvBox2D cvBox;
    float boxAngle;
    
    CvMemStorage *mem2;
	mem2 = cvCreateMemStorage(0);
    
    for (; contours != 0; contours = contours->h_next)
    {
        ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 ); 
        ROS_INFO("n %d", contours->total);
        if(contours->total >500 && contours->total <10000) 
        {
            //cvDrawContours(color_img, contours, ext_color, CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));
            
            cvContourMoments(contours, &cvm);
            cvGetHuMoments(&cvm, &cvHM);
            ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf, %lf", cvHM.hu1, cvHM.hu2, cvHM.hu3, cvHM.hu4, cvHM.hu5, cvHM.hu6, cvHM.hu7);
            
            //X=(x1m1+x2m2+‥+ximi)/M, Y=(y1m1+y2m2+‥+yimi)/M
            double pose_x = cvm.m10/cvm.m00;
            double pose_y = cvm.m01/cvm.m00;
            ROS_INFO("pose_x, pose_y: %lf, %lf", pose_x, pose_y);

            cvBox = cvMinAreaRect2(contours, mem2);
            boxAngle= cvBox.angle;
            
            float height =  cvBox.size.height;
            float width = cvBox.size.width;
            float center_x=  cvBox.center.x;
            float center_y=  cvBox.center.y;
            ROS_INFO("boxAngle, center.x, center.y, height, width: %lf, %lf, %lf, %lf, %lf", boxAngle, center_x, center_y, height, width);
            //DrawBox(cvBox,color_img,ext_color);
			
			//add some prefiltering criteria;
            if(max(height,width)>100)
            {
				ROS_INFO("candidate contour");
				std::vector <TPoint2D> contour_points;
				for(int i=0; i<contours->total; i++)
				{
					CvPoint* p = (CvPoint*)cvGetSeqElem(contours, i);
					TPoint2D pt_tmp;
					pt_tmp.x = p -> x;
					pt_tmp.y = p -> y;
					contour_points.push_back(pt_tmp);
				}
				
				//------------------------------------------------------------------------------------------
				//preparation work, to reduce a pair of lines in the contour into one;
				//------------------------------------------------------------------------------------------
				ROS_INFO("step1");
				
				std::vector <TPoint2D> fused_points;
				if(contour_points.size()<4) continue;
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
					
					//to find the perpendicular bisector of these two points;
					//printf("prev2_serial %d\t", prev2_serial);
					//printf("next2_serial %d\t", next2_serial);
					//printf("contour_points[prev2_serial]:%5f\t,%5f\n",contour_points[prev2_serial].x,contour_points[prev2_serial].y);
					//printf("contour_points[next2_serial]:%5f\t,%5f\n",contour_points[next2_serial].x,contour_points[next2_serial].y);
					//this condition does exist;
					
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
					//cvCircle( color_img, cvPoint(fused_points[i].x,fused_points[i].y), 2, CV_RGB( 255, 255, 255 ), 1);
					//ROS_INFO("fused point serial %d, %2f, %2f",i, xx, yy);
				}
				ROS_INFO("fused points number %ld", xs.size() );
				
				//-------------------------------------------------------------------------------------------------
				//approximation by straight lines;
				//-------------------------------------------------------------------------------------------------
				/*
				const double DIST_THRESHOLD = 10;
				vector<pair<size_t, TLine2D> >   detectedLines;
				ransac_detect_2D_lines(xs,ys,detectedLines,DIST_THRESHOLD, 300 );
				for (vector<pair<mrpt::vector_size_t, TLine2D> >::iterator p=detectedLines.begin();p!=detectedLines.end(); ++p)
				{
					ROS_INFO("line detected");
					
					if(p->second.coefs[1]!=0.0)
					{
						for(unsigned int xPixel=0; xPixel<640; xPixel++)
						{
							unsigned int yPixel = -(p->second.coefs[2]+p->second.coefs[0]*xPixel)/p->second.coefs[1];
							cvCircle( color_img, cvPoint(xPixel,yPixel), 2, ext_color, 1);
						}
					}
					else
					{
						for(unsigned int yPixel=0; yPixel<360; yPixel++)
						{
							unsigned int xPixel = -(p->second.coefs[2]+p->second.coefs[1]*yPixel)/p->second.coefs[0];
							cvCircle( color_img, cvPoint(xPixel,yPixel), 2, ext_color, 1);
						}
					}
				}
				*/ 
				
				//-------------------------------------------------------------------------------------------------
				//approximation by parabola lines;
				//-------------------------------------------------------------------------------------------------
				vector<pair<mrpt::vector_size_t, parabola> >   detectedLines;
				//temporarily the unit is still pixel;
				const double DIST_THRESHOLD = 5;
				
				//-------------------------------------------------------------------------------------------------
				// non-lane contours will take much time to process; 
				// add pre-filtering step for those nonlane contours;
				//-------------------------------------------------------------------------------------------------
				
				ransac_detect_parabolas(xs,ys,detectedLines,DIST_THRESHOLD, 100 );
				for (vector<pair<mrpt::vector_size_t,parabola> >::iterator p=detectedLines.begin();p!=detectedLines.end();++p)
				{
					ROS_INFO("---------------------lane detected-------------------");
					CvScalar random_color;
					random_color = CV_RGB( rand()&255, rand()&255, rand()&255 );
					if(p->second.coefs[3]==1.0)
					{
						for(unsigned int yPixel=0; yPixel<360; yPixel++)
						{
							unsigned int xPixel = p->second.coefs[0]*yPixel*yPixel+p->second.coefs[1]*yPixel+p->second.coefs[2];
							cvCircle( color_img, cvPoint(xPixel,yPixel), 2, ext_color, 1);
							//printf("approx point (%d, %d)   ", xPixel, yPixel);
						}
					}
					else
					{
						for(unsigned int xPixel=0; xPixel<640; xPixel++)
						{
							unsigned int yPixel = p->second.coefs[0]*xPixel*xPixel+p->second.coefs[1]*xPixel+p->second.coefs[2];
							cvCircle( color_img, cvPoint(xPixel,yPixel), 2, ext_color, 1);
							//printf("approx point (%d, %d)   ", xPixel, yPixel);
						}
					}
				}
				//-------------------------------------------------------------------------------------------------------------
			}
        }
    }
    
    ROS_INFO("single picture procession finished");
    
    cvNamedWindow("Color Image",1);
    cvShowImage("Color Image",color_img);
    
    cvWaitKey();
    
    //Clean up
    cvReleaseImage(&Igray);
    cvReleaseImage(&It);
    cvReleaseImage(&Iat);
    cvReleaseImage(&Itand);
    cvReleaseImage(&color_img);

    cvDestroyWindow("Raw");
    cvDestroyWindow("Threshold");
    cvDestroyWindow("Adaptive Threshold");
    cvDestroyWindow("Threshold And");
    
    cvReleaseMemStorage(&mem);
    cvReleaseMemStorage(&mem2);
    cvDestroyWindow("Color Image");
    
    return 0;
}

