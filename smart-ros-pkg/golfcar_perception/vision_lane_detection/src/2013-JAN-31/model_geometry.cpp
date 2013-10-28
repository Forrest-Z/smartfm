//20120410
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "lane_marker_common.h"
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define MODEL_SCALE 60.0

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
   
int main(int argc, char** argv) 
{
    IplImage *Igray=0, *Itand = 0;
    
    if(argc != 2){return -1;}

    if((Igray = cvLoadImage( argv[1], CV_LOAD_IMAGE_GRAYSCALE)) == 0){
    return -1;}
    
    Itand = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 1);
    
    //Threshold
    cvThreshold(Igray, Itand, BINARY_THRESH, 255, CV_THRESH_BINARY_INV);

    CvMemStorage *mem;
    mem = cvCreateMemStorage(0);
    
    CvSeq *contours = 0;
    // Use either this:
    
    int n = cvFindContours(Itand, mem, &contours, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
    //int n = cvFindContours(Itand, mem, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
    //different total number when having different parameters;
    
    //ROS_INFO("total contours: %d", n);
    
    IplImage *color_img = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 3);
    cvCvtColor(Itand, color_img, CV_GRAY2BGR);
    CvScalar ext_color;
    CvMoments cvm; 
    CvHuMoments cvHM;
    CvBox2D cvBox;
    float boxAngle;
    
    CvSeq *contour_poly;
	CvMemStorage *mem_poly;
	
    for (; contours != 0; contours = contours->h_next)
    {
		ROS_INFO("contour loop");
        ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 ); 
        ROS_INFO("n %d", contours->total);
        if(contours->total >300 && contours->total <5000) 
        {
            contour_poly = cvApproxPoly( contours, sizeof(CvContour), mem_poly, CV_POLY_APPROX_DP, 3, 0 );
            cvDrawContours(color_img, contour_poly, ext_color, CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));
            
            cvContourMoments(contour_poly, &cvm);
            cvGetHuMoments(&cvm, &cvHM);
            //ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf, %lf", cvHM.hu1, cvHM.hu2, cvHM.hu3, cvHM.hu4, cvHM.hu5, cvHM.hu6, cvHM.hu7);
            
            //X=(x1m1+x2m2+‥+ximi)/M, Y=(y1m1+y2m2+‥+yimi)/M
            double pose_x = cvm.m10/cvm.m00;
            double pose_y = cvm.m01/cvm.m00;
            ROS_INFO("pose_x, pose_y: %lf, %lf", pose_x, pose_y);
            
            double center_pix_x = 320;
			double center_pix_y = 180;
			double delt_x =  -(pose_y - center_pix_y)/30.0;
			double delt_y =  -(pose_x - center_pix_x)/30.0;
            double ipm_x = 9.0 + 1.975 + delt_x;
			double ipm_y = 0.0 + delt_y;
			
            CvMemStorage *mem2;
            mem2 = cvCreateMemStorage(0);
            
            cvBox = cvMinAreaRect2(contour_poly, mem2);
            boxAngle= cvBox.angle;
            
            float height =  cvBox.size.height;
            float width = cvBox.size.width;
            float center_x=  cvBox.center.x;
            float center_y=  cvBox.center.y;
            //ROS_INFO("boxAngle, center.x, center.y, height, width: %lf, %lf, %lf, %lf, %lf", boxAngle, center_x, center_y, height, width);
            DrawBox(cvBox,color_img,ext_color);
                
            //ROS_INFO("total %d", contour_poly->total);
            if(contour_poly->total<5){printf("this marker is bad! no angle information;");}
            else
            {
                std::vector <CvPoint> vertices;
                std::vector <CvPoint2D32f> centered_vertices;
                
                for(int i=0; i<contour_poly->total; i++)
                {
                    CvPoint* p = (CvPoint*)cvGetSeqElem(contour_poly, i);
                    //printf("(%d, %d)\n", p->x, p->y);
                    vertices.push_back(*p);
                    cvCircle( color_img, *p, 3, CV_RGB(0,255,0), 2);
                    CvPoint2D32f centered_vertix = centroid_centering_coordinate(*p, cvBox.center);
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
                    //printf("%f\n", distance);
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
					
					float fd_angle_delta, bd_angle_delta;
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
						
						printf("-(%d, %d), (%d, %d)---(%d, %d), (%d, %d)-\n", 
                                line1_frontd.x, line1_frontd.y, 
                                line1_backd.x , line1_backd.y,
                                line2_frontd.x, line2_frontd.y, 
                                line2_backd.x , line2_backd.y
                                );
                                 
                        cvCircle( color_img, cvPoint((int)pose_x, (int)pose_y), 6, CV_RGB(0,255,0), 2); 
                        
						cvCircle( color_img, line1_frontd, 6, CV_RGB(0,255,0), 2);
						cvCircle( color_img, line1_backd, 6, CV_RGB(0,255,0), 2);
						cvCircle( color_img, line2_frontd, 6, CV_RGB(0,255,0), 2);
						cvCircle( color_img, line2_backd, 6, CV_RGB(0,255,0), 2);
						
						
						printf("-(%3f, %3f), (%3f, %3f)---(%3f, %3f), (%3f, %3f)-\n", 
						
                                ((float)line1_frontd.x - pose_x)/MODEL_SCALE , ((float)line1_frontd.y - pose_y)/MODEL_SCALE, 
                                ((float)line1_backd.x - pose_x)/MODEL_SCALE , ((float)line1_backd.y - pose_y)/MODEL_SCALE, 
                                ((float)line2_frontd.x - pose_x)/MODEL_SCALE , ((float)line2_frontd.y - pose_y)/MODEL_SCALE, 
                                ((float)line2_backd.x - pose_x)/MODEL_SCALE , ((float)line2_backd.y - pose_y)/MODEL_SCALE 
                                );
					
					
						angle1 = atan2f((line1_front.y-line1_back.y), (line1_front.x-line1_back.x));
						angle2 = atan2f((line2_front.y-line2_back.y), (line2_front.x-line2_back.x));
						
						//remember to filter the case happened in picture 215;
						double abs_delt = fabs(fabs(angle1)-fabs(angle2));
						if(abs_delt>M_PI_2){printf("noisy situation, do not provide angle");}
						else
						{
							//thetha is in "rad" here;
							double thetha = (angle1+angle2)/2.0;
							//be careful with this case
							if((angle1 > M_PI_2 && angle2 < -M_PI_2) || (angle2 > M_PI_2&& angle1 < -M_PI_2))
							{thetha = thetha + M_PI;}
							//printf("---------marker ( x, y, angle) : %5f, %5f, %5f----------\n", ipm_x, ipm_y, thetha);
							
						}
					}
                }
            }
        
            cvReleaseMemStorage(&mem2);
        }
    }
    
    cvNamedWindow("Color Image",1);
	cvShowImage("Color Image",color_img);

	cvWaitKey();
   
    //Clean up
    cvReleaseImage(&Igray);
    cvReleaseImage(&color_img);

    cvDestroyWindow("Raw");
    cvDestroyWindow("Threshold");
    cvDestroyWindow("Adaptive Threshold");
    cvDestroyWindow("Threshold And");
    
    cvReleaseMemStorage(&mem);
    
    return 0;
}

