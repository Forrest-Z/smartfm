//20120410
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "lane_marker_common.h"

using namespace golfcar_vision;

int main(int argc, char** argv) 
{
    IplImage *Igray=0, *It = 0, *Iat = 0, *Itand = 0;
    if(argc != 7){return -1;}
    //Command line
    double threshold = (double)atof(argv[1]);
    int threshold_type = atoi(argv[2])? CV_THRESH_BINARY : CV_THRESH_BINARY_INV;
    int adaptive_method = atoi(argv[3])? CV_ADAPTIVE_THRESH_MEAN_C : CV_ADAPTIVE_THRESH_GAUSSIAN_C;
    int block_size = atoi(argv[4]);
    double offset = (double)atof(argv[5]);
    //Read in gray image
    if((Igray = cvLoadImage( argv[6], CV_LOAD_IMAGE_GRAYSCALE)) == 0){
    return -1;}
    // Create the grayscale output images
    It = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 1);
    Iat = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 1);
    Itand = cvCreateImage(cvSize(Igray->width,Igray->height),IPL_DEPTH_8U, 1);
    
    //Threshold
    cvThreshold(Igray,It,threshold,255,threshold_type);
    cvAdaptiveThreshold(Igray, Iat, 255, adaptive_method, threshold_type, block_size, offset);
    
    cvAnd(It, Iat, Itand);
    cvNamedWindow("Threshold And",1);
    
    
    cvNamedWindow("Raw",1);
    cvNamedWindow("Threshold",1);
    cvNamedWindow("Adaptive Threshold",1);
    cvShowImage("Raw",Igray);
    cvShowImage("Threshold",It);
    cvShowImage("Adaptive Threshold",Iat);
    cvShowImage("Threshold And",Itand);
    
    //cvSaveImage("It.png", It);
    //cvSaveImage("Iat.png", Iat);
    //cvSaveImage("Itand.png", Itand);
    
    CvMemStorage *mem;
    mem = cvCreateMemStorage(0);
    
    
    CvSeq *contours = 0;
    // Use either this:
    
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
    
    for (; contours != 0; contours = contours->h_next)
    {
        ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 ); 
        ROS_INFO("n %d", contours->total);
        if(contours->total >300 && contours->total <1000) 
        {
            //Polygon approximation: get vertices of the contour;
            contours = cvApproxPoly( contours, sizeof(CvContour), mem, CV_POLY_APPROX_DP, 2, 0 );
            cvDrawContours(color_img, contours, ext_color, CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));
            
            cvContourMoments(contours, &cvm);
            cvGetHuMoments(&cvm, &cvHM);
            ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf, %lf", cvHM.hu1, cvHM.hu2, cvHM.hu3, cvHM.hu4, cvHM.hu5, cvHM.hu6, cvHM.hu7);
            
            //X=(x1m1+x2m2+‥+ximi)/M, Y=(y1m1+y2m2+‥+yimi)/M
            double pose_x = cvm.m10/cvm.m00;
            double pose_y = cvm.m01/cvm.m00;
            ROS_INFO("pose_x, pose_y: %lf, %lf", pose_x, pose_y);
            
            CvMemStorage *mem2;
            mem2 = cvCreateMemStorage(0);
            
            cvBox = cvMinAreaRect2(contours, mem2);
            boxAngle= cvBox.angle;
            
            float height =  cvBox.size.height;
            float width = cvBox.size.width;
            float center_x=  cvBox.center.x;
            float center_y=  cvBox.center.y;
            ROS_INFO("boxAngle, center.x, center.y, height, width: %lf, %lf, %lf, %lf, %lf", boxAngle, center_x, center_y, height, width);
            DrawBox(cvBox,color_img,ext_color);
            
            
            ROS_INFO("total %d", contours->total);
            if(contours->total<5){printf("this marker is bad! no angle information;");}
            else
            {
                std::vector <CvPoint> vertices;
                std::vector <CvPoint2D32f> centered_vertices;
                
                for(int i=0; i<contours->total; i++)
                {
                    CvPoint* p = (CvPoint*)cvGetSeqElem(contours, i);
                    printf("(%d, %d)\n", p->x, p->y);
                    vertices.push_back(*p);
                    
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
                    printf("%f\n", distance);
                }
                
                unsigned int longest_serial, sec_longest_serial;
                float temp_distance1, temp_distance2;
                longest_serial = find_longest_distance(distance_vec);
                temp_distance1 = distance_vec[longest_serial];
                distance_vec[longest_serial]=0.0;
                sec_longest_serial = find_longest_distance(distance_vec);
                temp_distance2 = distance_vec[sec_longest_serial];
                
                if(temp_distance1<50.0||temp_distance2<50.0){printf("this marker is not long enough! no angle information;");}
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
                    if(abs_delt>M_PI_2){printf("noisy situation, do not provide angle");}
                    else
                    {
                        //thetha is in "rad" here;
                        double thetha = (angle1+angle2)/2.0;
                        //be careful with this case
                        if((angle1 > M_PI_2 && angle2 < -M_PI_2) || (angle2 > M_PI_2&& angle1 < -M_PI_2))
                        {thetha = thetha + M_PI;}
                        
                        printf("-(%d, %d), (%d, %d)---(%d, %d), (%d, %d)-\n", 
                                line1_frontd.x, line1_frontd.y, 
                                line1_backd.x , line1_backd.y,
                                line2_frontd.x, line2_frontd.y, 
                                line2_backd.x , line2_backd.y
                                );
                        printf("---------marker angle: %lf----------\n", thetha);
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
    cvReleaseImage(&It);
    cvReleaseImage(&Iat);
    cvReleaseImage(&Itand);
    cvReleaseImage(&color_img);
    cvDestroyWindow("Raw");
    cvDestroyWindow("Threshold");
    cvDestroyWindow("Adaptive Threshold");
    cvDestroyWindow("Threshold And");
    
    cvReleaseMemStorage(&mem);
    
    return 0;
}

