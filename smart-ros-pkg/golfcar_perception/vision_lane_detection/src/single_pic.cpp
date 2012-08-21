//20120410
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "lane_marker_common.h"
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace golfcar_vision;

IplImage* DrawHistogram(CvHistogram *hist, float scaleX=1, float scaleY=1)
{
	 float histMax = 0;
    cvGetMinMaxHistValue(hist, 0, &histMax, 0, 0);
    IplImage* imgHist = cvCreateImage(cvSize(256*scaleX, 64*scaleY), 8 ,1);
    cvZero(imgHist);
    for(int i=0;i<255;i++)
    {
        float histValue = cvQueryHistValue_1D(hist, i);
        float nextValue = cvQueryHistValue_1D(hist, i+1);
        CvPoint pt1 = cvPoint(i*scaleX, 64*scaleY);
        CvPoint pt2 = cvPoint(i*scaleX+scaleX, 64*scaleY);
        CvPoint pt3 = cvPoint(i*scaleX+scaleX, (64-nextValue*64/histMax)*scaleY);
        CvPoint pt4 = cvPoint(i*scaleX, (64-histValue*64/histMax)*scaleY);

        int numPts = 5;
        CvPoint pts[] = {pt1, pt2, pt3, pt4, pt1};
 
        cvFillConvexPoly(imgHist, pts, numPts, cvScalar(255));
    }
    return imgHist;
}

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
    IplImage *It = 0, *Iat = 0, *Itand = 0;
    IplImage *gray_image=0, *src_image=0, *ipm_image=0;
    int depth;
    CvSize size;
    
    /*
    IplImage *color_ipm;
    IplImage *img, *hsv, *hue, *sat, *val;
    */
    
    if(argc != 2){return -1;}

    if((src_image = cvLoadImage( argv[1], 1)) == 0){
    return -1;}
    size = cvGetSize(src_image);
    depth = src_image->depth;
   
 
    int numBins = 256;
    float range[] = {0, 255};
    float *ranges[] = { range };
 
    CvHistogram *hist = cvCreateHist(1, &numBins, CV_HIST_ARRAY, ranges, 1);
    cvClearHist(hist);
    
    /*
    IplImage* imgRed = cvCreateImage(size, 8, 1);
    IplImage* imgGreen = cvCreateImage(size, 8, 1);
    IplImage* imgBlue = cvCreateImage(size, 8, 1);
 
    cvSplit(src_image, imgBlue, imgGreen, imgRed, NULL);
    cvCalcHist(&imgRed, hist, 0, 0);
    
    IplImage* imgHistRed = DrawHistogram(hist);
    cvClearHist(hist);
    cvCalcHist(&imgGreen, hist, 0, 0);
    IplImage* imgHistGreen = DrawHistogram(hist);
    cvClearHist(hist);
    cvCalcHist(&imgBlue, hist, 0, 0);
    IplImage* imgHistBlue = DrawHistogram(hist);
    cvClearHist(hist);
    */
    
    /*
    color_ipm = cvCreateImage(cvGetSize(src_image),8,3);
    cvWarpPerspective( src_image, color_ipm, warp_matrix);
    hue = cvCreateImage(size, depth, 1);
    sat = cvCreateImage(size, depth, 1);
    val = cvCreateImage(size, depth, 1);
    hsv = cvCreateImage(size, depth, 3);
    cvZero(hue);
    cvZero(sat);
    cvZero(val);
    cvZero(hsv);
    cvCvtColor( color_ipm, hsv, CV_BGR2HSV );
    cvSplit(hsv, hue, sat, val, 0);
    cvNamedWindow("hue", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("saturation", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("value", CV_WINDOW_AUTOSIZE);
    cvShowImage("hue", hue);
    cvShowImage("saturation", sat);
    cvShowImage("value", val);
	 */
    
    CvPoint2D32f gndQuad_[4], srcQuad_[4], dstQuad_[4];
	 CvPoint origin_keyPts[4];
	
	srcQuad_[0].x = 41.358669; 		srcQuad_[0].y = 328.892059;
	srcQuad_[1].x = 612.351196; 		srcQuad_[1].y = 328.892059;
	srcQuad_[2].x = 627.685974; 		srcQuad_[2].y = 173.960587;
	srcQuad_[3].x = 26.023849; 		srcQuad_[3].y = 173.960587;
	
	dstQuad_[0].x = 260.000000; 		dstQuad_[0].y = 360.000000;
	dstQuad_[1].x = 380.000000; 		dstQuad_[1].y = 360.000000;
	dstQuad_[2].x = 620.000000; 		dstQuad_[2].y = 0.000000;
	dstQuad_[3].x = 20.000000; 		dstQuad_[3].y = 0.000000;
	
	CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);
    cvGetPerspectiveTransform(srcQuad_, dstQuad_,warp_matrix);
    
    gray_image = cvCreateImage(cvGetSize(src_image),8,1);
     cvCvtColor(src_image, gray_image, CV_BGR2GRAY);
    
	ipm_image = cvCreateImage(cvGetSize(gray_image),8,1);
    cvWarpPerspective( gray_image, ipm_image, warp_matrix);
    
    cvCalcHist(&gray_image, hist, 0, 0);
    IplImage* imgHist = DrawHistogram(hist);
    
	CvMat* projection_matrix = cvCreateMat(3,3,CV_32FC1);
    cvGetPerspectiveTransform(dstQuad_,srcQuad_,projection_matrix);
    
    CvMat* dstMarkerPts = cvCreateMat(4,1,CV_32FC2);
    CvMat* srcMarkerPts = cvCreateMat(4,1,CV_32FC2);
	
    // Create the grayscale output images
    It = cvCreateImage(size,IPL_DEPTH_8U, 1);
    Iat = cvCreateImage(size,IPL_DEPTH_8U, 1);
    Itand = cvCreateImage(size,IPL_DEPTH_8U, 1);
    
    //Threshold
    cvThreshold(ipm_image,It,BINARY_THRESH,255,CV_THRESH_BINARY);
    cvAdaptiveThreshold(ipm_image, Iat, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, BLOCK_SIZE, OFFSET);
    
    cvAnd(It, Iat, Itand);
    //cvNamedWindow("Threshold And",1);   
    
    //cvNamedWindow("Raw",1);
    //cvNamedWindow("Threshold",1);
    //cvNamedWindow("Adaptive Threshold",1);
    //cvShowImage("Raw",Igray);
    //cvShowImage("Threshold",It);
    //cvShowImage("Adaptive Threshold",Iat);
    //cvShowImage("Threshold And",Itand);


    CvMemStorage *mem;
    mem = cvCreateMemStorage(0);
    
    CvSeq *contours = 0;
    // Use either this:
    
    int n = cvFindContours(Itand, mem, &contours, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
    //int n = cvFindContours(Itand, mem, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
    //different total number when having different parameters;
    
    ROS_INFO("total contours: %d", n);
    
    IplImage *color_img = cvCreateImage(size,IPL_DEPTH_8U, 3);
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
        ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 ); 
        ROS_INFO("n %d", contours->total);
        if(contours->total >300 && contours->total <1000) 
        {
            //Polygon approximation: get vertices of the contour;
            contour_poly = cvApproxPoly( contours, sizeof(CvContour), mem_poly, CV_POLY_APPROX_DP, 2, 0 );
            cvDrawContours(color_img, contour_poly, ext_color, CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));
            
            cvContourMoments(contour_poly, &cvm);
            cvGetHuMoments(&cvm, &cvHM);
            ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf, %lf", cvHM.hu1, cvHM.hu2, cvHM.hu3, cvHM.hu4, cvHM.hu5, cvHM.hu6, cvHM.hu7);
            
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
            ROS_INFO("boxAngle, center.x, center.y, height, width: %lf, %lf, %lf, %lf, %lf", boxAngle, center_x, center_y, height, width);
            DrawBox(cvBox,color_img,ext_color);
                
            ROS_INFO("total %d", contour_poly->total);
            if(contour_poly->total<5){printf("this marker is bad! no angle information;");}
            else
            {
                std::vector <CvPoint> vertices;
                std::vector <CvPoint2D32f> centered_vertices;
                
                for(int i=0; i<contour_poly->total; i++)
                {
                    CvPoint* p = (CvPoint*)cvGetSeqElem(contour_poly, i);
                    printf("(%d, %d)\n", p->x, p->y);
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
                    printf("%f\n", distance);
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
						
						ROS_INFO("use_front_only");
					}
					else if (!fd_satisfy && bd_satisfy)
					{
						line2_front_serial = line_front;
						line2_back_serial  = line_back;
						line1_front_serial = backward_line_back;
						line1_back_serial  = backward_line_front;
						
						ROS_INFO("use_back_only");
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
                            
						cvCircle( color_img, line1_frontd, 6, CV_RGB(0,255,0), 2);
						cvCircle( color_img, line1_backd, 6, CV_RGB(0,255,0), 2);
						cvCircle( color_img, line2_frontd, 6, CV_RGB(0,255,0), 2);
						cvCircle( color_img, line2_backd, 6, CV_RGB(0,255,0), 2);
					
					
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
							printf("---------marker ( x, y, angle) : %5f, %5f, %5f----------\n", ipm_x, ipm_y, thetha);
							printf("-(%d, %d), (%d, %d)---(%d, %d), (%d, %d)-\n", 
                                line1_frontd.x, line1_frontd.y, 
                                line1_backd.x , line1_backd.y,
                                line2_frontd.x, line2_frontd.y, 
                                line2_backd.x , line2_backd.y
                                );
                                
                        
							printf("---------marker ( x, y, angle) : %5f, %5f, %5f----------\n", ipm_x, ipm_y, thetha);
							
							
							cvCircle( color_img, line1_frontd, 6, CV_RGB(0,255,0), 2);
							cvCircle( color_img, line1_backd, 6, CV_RGB(0,255,0), 2);
							cvCircle( color_img, line2_frontd, 6, CV_RGB(0,255,0), 2);
							cvCircle( color_img, line2_backd, 6, CV_RGB(0,255,0), 2);
							
							
							//(317, 213), (315, 348)---(320, 215), (324, 348)
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
							cvPerspectiveTransform(dstMarkerPts, srcMarkerPts, projection_matrix);
							
							row = 0;
							ptr = (float*)(srcMarkerPts->data.ptr + row * dstMarkerPts->step);
							origin_keyPts[0].x = ptr[0];
							origin_keyPts[0].y = ptr[1];
							row = 1;
							ptr = (float*)(srcMarkerPts->data.ptr + row * dstMarkerPts->step);
							origin_keyPts[1].x = ptr[0];
							origin_keyPts[1].y = ptr[1];
							row = 2;
							ptr = (float*)(srcMarkerPts->data.ptr + row * dstMarkerPts->step);
							origin_keyPts[2].x = ptr[0];
							origin_keyPts[2].y = ptr[1];
							row = 3;
							ptr = (float*)(srcMarkerPts->data.ptr + row * dstMarkerPts->step);
							origin_keyPts[3].x = ptr[0];
							origin_keyPts[3].y = ptr[1];
							
							cvCircle( src_image, origin_keyPts[0], 6, CV_RGB(0,255,0), 2);
							cvCircle( src_image, origin_keyPts[1], 6, CV_RGB(0,255,0), 2);
							cvCircle( src_image, origin_keyPts[2], 6, CV_RGB(0,255,0), 2);
							cvCircle( src_image, origin_keyPts[3], 6, CV_RGB(0,255,0), 2);
							
							CvMat* gndMarkerPts = cvCreateMat(4,1,CV_32FC3);
							row = 0;
							ptr = (float*)(gndMarkerPts->data.ptr + row * gndMarkerPts->step);
							ptr[0] = 0.46;
							ptr[1] = 0.05;
							ptr[2] = 0.0;
							row = 1;
							ptr = (float*)(gndMarkerPts->data.ptr + row * gndMarkerPts->step);
							ptr[0] = -2.779;
							ptr[1] = 0.14;
							ptr[2] = 0.0;
							row = 2;
							ptr = (float*)(gndMarkerPts->data.ptr + row * gndMarkerPts->step);
							ptr[0] = 0.46;
							ptr[1] = -0.05;
							ptr[2] = 0.0;
							
							row = 3;
							ptr = (float*)(gndMarkerPts->data.ptr + row * gndMarkerPts->step);
							ptr[0] = -2.779;
							ptr[1] = -0.14;
							ptr[2] = 0.0;
							
							CvMat* Intrinsic_A = cvCreateMat(3,3,CV_32FC1);
							row = 0;
							ptr = (float*)(Intrinsic_A->data.ptr + row * Intrinsic_A->step);
							ptr[0] = 462.55911;
							ptr[1] = 0.00;
							ptr[2] = 326.21463;
							row = 1;
							ptr = (float*)(Intrinsic_A->data.ptr + row * Intrinsic_A->step);
							ptr[0] = 0.0;
							ptr[1] = 472.34580;
							ptr[2] = 188.72264;
							row = 2;
							ptr = (float*)(Intrinsic_A->data.ptr + row * Intrinsic_A->step);
							ptr[0] = 0.0;
							ptr[1] = 0.0;
							ptr[2] = 1.0;
							
							CvMat* distortion_coeffs = cvCreateMat(5,1,CV_32FC1);
							ptr = (float*)(distortion_coeffs->data.ptr + 0 * distortion_coeffs->step); 	ptr[0] = -0.00791 ;
							ptr = (float*)(distortion_coeffs->data.ptr + 1 * distortion_coeffs->step); 	ptr[0] = -0.05311;
							ptr = (float*)(distortion_coeffs->data.ptr + 2 * distortion_coeffs->step);  ptr[0] = 0.00462;
							ptr = (float*)(distortion_coeffs->data.ptr + 3 * distortion_coeffs->step);  ptr[0] = 0.00101;
							ptr = (float*)(distortion_coeffs->data.ptr + 4 * distortion_coeffs->step);  ptr[0] = 0.0;
							
							CvMat* trans_vec = cvCreateMat(3,1,CV_32FC1);
							CvMat* rot_vec = cvCreateMat(3,1,CV_32FC1);
							CvMat* rot_matrix = cvCreateMat(3,3,CV_32FC1);
							
							float x_trans, y_trans, z_trans;
							cvFindExtrinsicCameraParams2(gndMarkerPts, srcMarkerPts, Intrinsic_A, distortion_coeffs, rot_vec, trans_vec);
							
							ptr = (float*)(trans_vec->data.ptr + 0 * trans_vec->step);  x_trans = ptr[0];
							ptr = (float*)(trans_vec->data.ptr + 1 * trans_vec->step);  y_trans = ptr[0];
							ptr = (float*)(trans_vec->data.ptr + 2 * trans_vec->step);  z_trans = ptr[0];
							printf("---------x_trans, y_trans, z_trans (%5f, %5f, %5f)----------\n", x_trans, y_trans, z_trans);
							
							cvRodrigues2(rot_vec, rot_matrix, NULL);
							
							row = 0;
							ptr = (float*)(rot_matrix->data.ptr + row * rot_matrix->step);
							float M11_rot = ptr[0];
							
							row = 1;
							ptr = (float*)(rot_matrix->data.ptr + row * rot_matrix->step);
							float M21_rot = ptr[0];
							
							row = 2;
							ptr = (float*)(rot_matrix->data.ptr + row * rot_matrix->step);
							float M31_rot = ptr[0];
							float M32_rot = ptr[1];
							float M33_rot = ptr[2];
							
							double yaw, pitch, roll;
							yaw	  = atan2f(M21_rot, M11_rot);
							pitch = atan2f(- M31_rot, sqrtf(M32_rot*M32_rot+M33_rot*M33_rot));
							roll  = atan2f(M32_rot, M33_rot);
							
							printf("---------yaw, pitch, roll (%5f, %5f, %5f)----------\n", yaw, pitch, roll);
							
							tf::Pose cameraInbaselink (tf::createQuaternionFromRPY(0.0,0.0,0.0), 
										btVector3(1.795, 0.0, 0.97));
										
							tf::Pose imageInCamerabase (tf::createQuaternionFromRPY(-1.5707963267949, 0.0, -1.5707963267949), 
										btVector3(0.0, 0.0, 0.0));
							
							tf::Pose markerCoordinate_inCam (tf::createQuaternionFromRPY(roll,pitch,yaw), 
										btVector3(x_trans,y_trans, z_trans));
										
							//tf:: Pose markerInBaselink = cameraInbaselink * imageInCamerabase * markerCoordinate_inCam;
							tf:: Pose markerInBaselink = imageInCamerabase * markerCoordinate_inCam;
							
							x_trans = markerInBaselink.getOrigin().x();
							y_trans = markerInBaselink.getOrigin().y();
							z_trans = markerInBaselink.getOrigin().z();
							
							printf("---------x_trans, y_trans, z_trans (%5f, %5f, %5f)----------\n", x_trans, y_trans, z_trans);
							markerInBaselink.getBasis().getEulerYPR(yaw, pitch, roll);
							printf("---------yaw, pitch, roll (%5f, %5f, %5f)----------\n", yaw, pitch, roll);
							float dis_to_camera = sqrtf(x_trans*x_trans+y_trans*y_trans+z_trans*z_trans);
							printf("dis_to_camera %5f\n", dis_to_camera);
							float dis_to_baselink = sqrtf(dis_to_camera*dis_to_camera-1.32*1.32)+ 1.795;
							printf("dis_to_baselink %5f\n", dis_to_baselink);
							
						}
                   
					}
				}
			}
            cvReleaseMemStorage(&mem2);
        }
    }
    
    cvNamedWindow("Color Image",1);
	cvShowImage("Color Image",color_img);

	cvNamedWindow("src_image",1);
	cvShowImage("src_image",src_image);
						
	/*
	cvNamedWindow("color_ipm",1);
	cvShowImage("color_ipm",color_ipm);
	*/
	
	/*
	 cvNamedWindow("Red");
    cvNamedWindow("Green");
	 cvNamedWindow("Blue");
 
    cvShowImage("Red", imgHistRed);
    cvShowImage("Green", imgHistGreen);
    cvShowImage("Blue", imgHistBlue);
	*/
	cvShowImage("histogram", imgHist);
	cvWaitKey();
   
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

