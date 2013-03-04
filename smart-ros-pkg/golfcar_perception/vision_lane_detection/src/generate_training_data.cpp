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
using namespace golfcar_vision;
CvPoint corners_[4];
float scale_;
std::vector<CvPoint> ipm_polygon_;

void load_parameter_file()
{
	FILE *fp_restore;
	fp_restore = fopen("./images/parameter_file","r");
	if(fp_restore==NULL)
	{
		fprintf(stderr,"can't open file \n");
		exit(1);
	}
	cout<<"File opened"<<endl;

	for(int i=0; i<4; i++) ipm_polygon_.push_back(cvPoint(0, 0));
	fscanf(fp_restore, "%f\t", &scale_);
	fscanf(fp_restore, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t", &ipm_polygon_[0].x, &ipm_polygon_[0].y, &ipm_polygon_[1].x, &ipm_polygon_[1].y,
														   &ipm_polygon_[2].x, &ipm_polygon_[2].y, &ipm_polygon_[3].x, &ipm_polygon_[3].y);
	fclose(fp_restore);
}

CvSeq* Filter_candidates (CvContourScanner &scanner, unsigned int module_type)
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

		//3nd criterion: no touching polygon boundary;
		//this criterion only applies to discrete markers, while not to continuous lanes;
		bool inside_polygon = true;
		CvPoint2D32f point[4];
		calc_cvBoxPoints(cvBox, point);
		for (int i=0; i<4; i++)
		{
			for(int a = -1; a<=1; a++)
			{
				for(int b = -1; b<=1; b++)
				{
					CvPoint tmppoint = cvPoint(point[i].x+a, point[i].y+b);

					if(pointInPolygon <CvPoint> (tmppoint,ipm_polygon_))
					{
						inside_polygon = false;
						break;
					}
				}
				if(!inside_polygon) break;
			}
		}

		bool contour_criteria;
		if(module_type == 1 || module_type == 3 || module_type == 4)
		{
			contour_criteria = len_criterion && long_side_criterion && inside_polygon;
		}
		else if(module_type == 2 )
		{
			contour_criteria = len_criterion && long_side_criterion;
		}

		if(!contour_criteria) cvSubstituteContour(scanner, NULL);
    }
    CvSeq *contours = cvEndFindContours(&scanner);
    cvReleaseMemStorage(&mem_box);
    return contours;
  }


int main(int argc, char** argv) 
{
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

    load_parameter_file();

    //--------------------------------------------------------------------------------------
    //there are multiple modules to be trained, 4 right now.
    unsigned int module_type;
    printf("Please key in the module type\n");
    scanf("%u", &module_type);
    printf("module type: %u\n", module_type);
    //--------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------
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
    //--------------------------------------------------------------------------------------

    cvNamedWindow("contour_image",1);

    for(unsigned int image_serial = start_num; image_serial<=end_num; image_serial++)
    {
        //try to open the file to be written;
        FILE *fp;
        stringstream  training_data_string;
        training_data_string<<"./images/training_data"<<module_type;
        const char *training_data_name = training_data_string.str().c_str();

        if((fp=fopen(training_data_name, "a"))==NULL){printf("cannot open file\n");return 0;}
    
        printf("----------------------------------------------------------------------\n");
        printf("process image %u\n", image_serial);
        
        stringstream  name_string;
        name_string<<"./images/frame"<<image_serial<<".jpg";
        const char *input_name = name_string.str().c_str();
        
        //try to load the image;
        if((Igray = cvLoadImage( input_name, CV_LOAD_IMAGE_GRAYSCALE)) == 0) 
        {return -1;printf("cannot load the required picture");continue;}

        IplImage *contour_img = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 3);
        cvCvtColor(Igray, contour_img, CV_GRAY2BGR);
        Img_preproc(Igray, Igray);

        cvShowImage("contour_image",contour_img);
        cvWaitKey(100);

        CvScalar ext_color;
        
        CvContourScanner scanner = cvStartFindContours(Igray, mem_contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        contours = Filter_candidates(scanner, module_type);
        
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
                fprintf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t", cvHM.hu1, cvHM.hu2, cvHM.hu3, cvHM.hu4, cvHM.hu5, cvHM.hu6, cvHM.hu7);
                fprintf(fp, "%lf\t%lf\t", boxshortside, boxlongside);
                fprintf(fp, "%lf\t%lf\t%d\n",contour_weight,contour_perimeter,approxPtNum);
                
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
    cvReleaseMemStorage(&mem_contours);
    cvReleaseMemStorage(&mem_box);
    cvReleaseMemStorage(&mem_poly);
    //cvDestroyWindow("Raw");
    cvDestroyWindow("contour_image");
    return 0;
}

#endif
