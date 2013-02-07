#ifndef LANE_MARKER_GENERATE_TRAINING_DATA
#define LANE_MARKER_GENERATE_TRAINING_DATA

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "lane_marker_common.h"
#include <cstdio>
#include <vector>
#include <stdlib.h>
using namespace std;

CvPoint corners_[4];

int height = 360;
int width  = 640;
float scale_ = float(height)/GND_HEIGHT;
    
void cvBoxPoints( CvBox2D box, CvPoint2D32f pt[4] )
{
      double angle = - box.angle*M_PI/180.0;
      float a = (float)cos(angle)*0.5f;
      float b = (float)sin(angle)*0.5f;
 
      pt[0].x = box.center.x - a*box.size.width - b*box.size.height;
      pt[0].y = box.center.y + b*box.size.width - a*box.size.height;
      pt[1].x = box.center.x + a*box.size.width - b*box.size.height;
      pt[1].y = box.center.y - b*box.size.width - a*box.size.height;
      pt[2].x = 2*box.center.x - pt[0].x;
      pt[2].y = 2*box.center.y - pt[0].y;
      pt[3].x = 2*box.center.x - pt[1].x;
      pt[3].y = 2*box.center.y - pt[1].y;
} 

void DrawBox(CvBox2D box, IplImage* img, CvScalar ext_color)
{
      CvPoint2D32f point[4];
      int i;
      for ( i=0; i<4; i++)
      {
          point[i].x = 0;
          point[i].y = 0;
      }
      
     cvBoxPoints(box, point); 
     CvPoint pt[4];
     
     for ( i=0; i<4; i++)
     {
         pt[i].x = (int)point[i].x;
         pt[i].y = (int)point[i].y;
     }
     cvLine( img, pt[0], pt[1], ext_color, 2, 8, 0 );
     cvLine( img, pt[1], pt[2], ext_color, 2, 8, 0 );
     cvLine( img, pt[2], pt[3], ext_color, 2, 8, 0 );
     cvLine( img, pt[3], pt[0], ext_color, 2, 8, 0 );
} 

bool CheckPointInside(CvPoint pt_para)
{
    corners_[0].x = int(width/2 - RECT_P0_Y * scale_);    
    corners_[0].y = height;
    corners_[1].x = int(width/2 - RECT_P1_Y * scale_);
    corners_[1].y = height;
    corners_[2].x = width;   
    corners_[2].y = 0;
    corners_[3].x = 0;   
    corners_[3].y = 0;
    
    double delt_x1, delt_y1, delt_x2, delt_y2;
    delt_x1= corners_[0].x-corners_[3].x;
    delt_y1= corners_[0].y-corners_[3].y;
    delt_x2 = corners_[1].x-corners_[2].x;
    delt_y2 = corners_[1].y-corners_[2].y;

    double para_A1_ = delt_y1/delt_x1;
    double para_C1_ = corners_[0].y-para_A1_*corners_[0].x;
    double para_A2_ = delt_y2/delt_x2;
    double para_C2_ = corners_[1].y-para_A2_*corners_[1].x;

    //The boundary at the upper line is tricky, put the margin to "5";
    //see the "upper_boundary" picture;
    bool out_flag1 = pt_para.y+5 > corners_[0].y;
    bool out_flag2 = pt_para.y-BOUNDARY_MARGIN < 0;
    bool out_flag3 = pt_para.y+BOUNDARY_MARGIN > para_A1_*pt_para.x+para_C1_;
    bool out_flag4 = pt_para.y+BOUNDARY_MARGIN > para_A2_*pt_para.x+para_C2_;
    
    //printf("pt_para.x, pt_para.y (%d, %d)", pt_para.x, pt_para.y);
        
    if(out_flag1||out_flag2||out_flag3||out_flag4) 
    {
        //printf("-------outside------\n");
        return false;
    }
    else 
    {
        //printf("-------inside------\n");
        return true;
    }
}
    
 CvSeq* Filter_candidates (CvContourScanner &scanner)
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
				/*
                //3rd criterion: four corners shouldn't appear at the boundary;
                CvPoint2D32f point[4];
                cvBoxPoints(cvBox, point); 
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

int main(int argc, char** argv) 
{
    IplImage *It = 0, *Iat = 0, *Itand = 0;
    It = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1);
    Iat = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1);
    Itand = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1);
    
    IplImage *Igray = 0;
    CvSeq *contours = 0;
    CvMemStorage *mem_contours;
    mem_contours = cvCreateMemStorage(0);
    CvMemStorage *mem_box;
    mem_box = cvCreateMemStorage(0);
    CvMoments cvm; 
    CvHuMoments cvHM;
    CvBox2D cvBox;
    
    CvSeq *contour_poly;
	CvMemStorage *mem_poly;
	mem_poly = cvCreateMemStorage(0);
	double contour_weight;
	double contour_perimeter;
	int approxPtNum;
	
    float boxshortside;
    float boxlongside;
    unsigned int class_type;
    
    //key in image range; interact with the code from terminal;
    unsigned int start_num = 0, end_num = 0;
    int image_range_OK = 0;
    while(image_range_OK==0)
    {
        printf("Please key in the the start number, the end number\n");
        scanf("%u:%u", &start_num, &end_num);
        printf("begin to extract training data from images: %u:%u\n", start_num, end_num);
        printf("Redo type 0, Go on type 1\n");
        scanf("%d", &image_range_OK);
    }
    
    //cvNamedWindow("raw",1);
    cvNamedWindow("contour_image",1);
    
    for(unsigned int image_serial =start_num; image_serial<=end_num; image_serial++)
    {
        //try to open the file to be written;
        FILE *fp;
        if((fp=fopen("./training_data", "a"))==NULL){printf("cannot open file\n");return 0;}
    
        printf("----------------------------------------------------------------------\n");
        printf("process image %u\n", image_serial);
        
        stringstream  name_string;
        name_string<<"./images/frame"<<image_serial<<".jpg";
        const char *input_name = name_string.str().c_str();
        
        //try to load the image;
        if((Igray = cvLoadImage( input_name, CV_LOAD_IMAGE_GRAYSCALE)) == 0) 
        {return -1;printf("cannot load the required picture");continue;}
        
        cvThreshold(Igray,It,BINARY_THRESH,255,CV_THRESH_BINARY);
        cvAdaptiveThreshold(Igray, Iat, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, BLOCK_SIZE, OFFSET);
        cvAnd(It, Iat, Itand);
        
        printf("thresholding finished\n");
        
        IplImage *contour_img = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 3);
        cvCvtColor(Itand, contour_img, CV_GRAY2BGR);
        CvScalar ext_color;
        
        CvContourScanner scanner = cvStartFindContours(Itand, mem_contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        contours = Filter_candidates(scanner);
        
        printf("contours extracted\n\n");
        
        std::vector <unsigned int>  class_vector;
        std::vector <float >        shortside_vector;
        std::vector <float >        longside_vector;
        std::vector <CvHuMoments>   CvHuMoments_vector;
        std::vector <double>  		weight_vector;
        std::vector <double >       perimeter_vector;
        std::vector <int >        	approxNum_vector;
        
        unsigned int contour_serial_in_this_image = 0;
        
        //to record the first address of the contours;
        CvSeq *first_contours = contours;
        
        for (; contours != 0; contours = contours->h_next)
        {   
            ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 ); 
            cvDrawContours(contour_img, contours, ext_color, CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));
            
            cvLine( contour_img, corners_[0], corners_[1], ext_color, 1, 8 );
            cvLine( contour_img, corners_[1], corners_[2], ext_color, 1, 8 );
            cvLine( contour_img, corners_[2], corners_[3], ext_color, 1, 8 );
            cvLine( contour_img, corners_[3], corners_[0], ext_color, 1, 8 );
            
            cvContourMoments(contours, &cvm);
            cvGetHuMoments(&cvm, &cvHM);
            
            contour_weight = cvm.m00;
            contour_perimeter = cvContourPerimeter(contours);
            
            contour_poly = cvApproxPoly( contours, sizeof(CvContour), mem_poly, CV_POLY_APPROX_DP, 2, 0 );
			approxPtNum = int (contour_poly->total);
			
            cvBox = cvMinAreaRect2(contours, mem_box);
            float boxAngle= cvBox.angle;
            float boxheight =  cvBox.size.height;
            float boxwidth = cvBox.size.width;
            //to sort the height and width according to their length;
            boxshortside = min(boxheight,boxwidth);
            boxlongside  = max(boxheight,boxwidth);
            DrawBox(cvBox,contour_img, ext_color);
            
            //cvShowImage("raw",Igray);
            cvShowImage("contour_image",contour_img);
                        
            printf("contour %u\n", contour_serial_in_this_image);
            cvWaitKey(100);
            //cvWaitKey(0);                   //it seems not working properly;
            
            class_type = 1;
            printf("please key in the type of this contour\n");
            scanf("%u", &class_type);
            
            //fprintf(fp, "%u\t%u\n", image_serial, class_type);
            //fprintf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t", cvHM.hu1, cvHM.hu2, cvHM.hu3, cvHM.hu4, cvHM.hu5, cvHM.hu6, cvHM.hu7);
            //fprintf(fp, "%lf\t%lf\n", boxshortside, boxlongside);
            
            printf("image serial: %u, contour serial: %u, contour class: %u\n", image_serial, contour_serial_in_this_image, class_type);
            printf("weigth %lf\t perimeter %lf\t approxNum %d\n",contour_weight,contour_perimeter,approxPtNum);
            printf("H-Moment: %lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", cvHM.hu1, cvHM.hu2, cvHM.hu3, cvHM.hu4, cvHM.hu5, cvHM.hu6, cvHM.hu7);
            printf("Bounding box %lf\t%lf\t%lf\n\n", boxAngle, boxshortside, boxlongside);
            
            contour_serial_in_this_image++;
            
            //store the results temporally in four vectors;
            class_vector.push_back(class_type);
            shortside_vector.push_back(boxshortside);
            longside_vector.push_back(boxlongside);
            CvHuMoments_vector.push_back(cvHM);
            weight_vector.push_back(contour_weight);
			perimeter_vector.push_back(contour_perimeter);
         	approxNum_vector.push_back(approxPtNum);
        }
         
        int keep_this_image = 0;
        printf("keep the results of this image? Press 1 to keep, 0 to skip\n");
        scanf("%d", &keep_this_image);
        
        if(keep_this_image == 0){continue; printf("results of this image will be skiped\n");}
        if(keep_this_image == 1)
        {
            printf("results of this image will be kept\n");
            contour_serial_in_this_image = 0;
            
            contours = first_contours;
            for (; contours != 0; contours = contours->h_next)
            {    
                class_type      =   class_vector[contour_serial_in_this_image];
                boxshortside    =   shortside_vector [contour_serial_in_this_image];
                boxlongside     =   longside_vector [contour_serial_in_this_image];
                cvHM            =   CvHuMoments_vector[contour_serial_in_this_image];
                contour_weight 	= 	weight_vector[contour_serial_in_this_image];
				contour_perimeter = perimeter_vector[contour_serial_in_this_image];
				approxPtNum 	= 	approxNum_vector[contour_serial_in_this_image];
				
                fprintf(fp, "%u\t%u\t", image_serial, class_type);
                fprintf(fp, "%lf\t%lf\t%d\t",contour_weight,contour_perimeter,approxPtNum);
                fprintf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t", cvHM.hu1, cvHM.hu2, cvHM.hu3, cvHM.hu4, cvHM.hu5, cvHM.hu6, cvHM.hu7);
                fprintf(fp, "%lf\t%lf\n", boxshortside, boxlongside);
                
                printf("image serial: %u, contour serial: %u, contour class: %u\n", image_serial, contour_serial_in_this_image, class_type);
                printf("H-Moment: %lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", cvHM.hu1, cvHM.hu2, cvHM.hu3, cvHM.hu4, cvHM.hu5, cvHM.hu6, cvHM.hu7);
                printf("Bounding box %lf\t%lf\n", boxshortside, boxlongside);
                
                contour_serial_in_this_image++;
            }
        }
        printf("This image is processed, wait 0.1s to process the next image\n");
        cvWaitKey(100);
        cvReleaseImage(&contour_img);
        fclose(fp);
    }
    
    cvReleaseImage(&Igray);
    cvReleaseImage(&It);
    cvReleaseImage(&Iat);
    cvReleaseMemStorage(&mem_contours);
    cvReleaseMemStorage(&mem_box);
    cvReleaseMemStorage(&mem_poly);
    //cvDestroyWindow("Raw");
    cvDestroyWindow("contour_image");
    
    return 0;
}

#endif
