//20121112

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <sensor_msgs/PointCloud.h>

#define BINARY_THRESH 200

//morphology code;
//http://opencv-users.1802565.n2.nabble.com/Morphological-thinning-operation-from-Z-Guo-and-R-W-Hall-quot-Parallel-Thinn-td4225544.html
//pay attention here the input is float image 0.0 ~ 1.0;
void ThinSubiteration1(CvMat *pSrc, CvMat *pDst);
void ThinSubiteration2(CvMat *pSrc, CvMat *pDst);

void MorphologicalThinning(CvMat *pSrc, CvMat *pDst) { 
	
        bool bDone = false; 
        int rows = pSrc->rows; 
        int cols = pSrc->cols; 
        /// pad source 
        CvMat *p_enlarged_src = cvCreateMat(rows + 2, cols + 2, CV_32FC1); 
        for(int i = 0; i < (rows+2); i++) { 
                CV_MAT_ELEM(*p_enlarged_src, float, i, 0)	 = 0.0f; 
                CV_MAT_ELEM(*p_enlarged_src, float, i, cols+1)	= 0.0f; 
        } 
        for(int j = 0; j < (cols+2); j++) { 
                CV_MAT_ELEM(*p_enlarged_src, float, 0, j)	 = 0.0f; 
                CV_MAT_ELEM(*p_enlarged_src, float, rows+1, j)	= 0.0f; 
        } 
        for(int i = 0; i < rows; i++) { 
                for(int j = 0; j < cols; j++) { 
                        if (CV_MAT_ELEM(*pSrc, float, i, j) >= 0.5f) { 
                                CV_MAT_ELEM(*p_enlarged_src, float, i+1, j+1) = 1.0f; 
                        } 
                        else 
                                CV_MAT_ELEM(*p_enlarged_src, float, i+1, j+1) = 0.0f; 
                } 
        } 
        /// start to thin 
        CvMat *p_thinMat1	= cvCreateMat(rows + 2, cols + 2, CV_32FC1); 
        CvMat *p_thinMat2	= cvCreateMat(rows + 2, cols + 2, CV_32FC1); 
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
                        CV_MAT_ELEM(*pDst, float, i, j) = CV_MAT_ELEM(*p_enlarged_src, float, i+1, j+1); 
                } 
        } 
        /// clean memory 
        cvReleaseMat(&p_enlarged_src); 
        cvReleaseMat(&p_thinMat1); 
        cvReleaseMat(&p_thinMat2); 
        cvReleaseMat(&p_cmp); 
} 

void ThinSubiteration1(CvMat *pSrc, CvMat *pDst) { 
        int rows = pSrc->rows; 
        int cols = pSrc->cols; 
        cvCopy(pSrc, pDst); 
        for(int i = 0; i < rows; i++) { 
                for(int j = 0; j < cols; j++) { 
                        if(CV_MAT_ELEM(*pSrc, float, i, j) == 1.0f) { 
                                /// get 8 neighbors 
                                /// calculate C(p) 
                                int neighbor0 = (int) CV_MAT_ELEM(*pSrc, float, i-1, j-1); 
                                int neighbor1 = (int) CV_MAT_ELEM(*pSrc, float, i-1, j); 
                                int neighbor2 = (int) CV_MAT_ELEM(*pSrc, float, i-1, j+1); 
                                int neighbor3 = (int) CV_MAT_ELEM(*pSrc, float, i, j+1); 
                                int neighbor4 = (int) CV_MAT_ELEM(*pSrc, float, i+1, j+1); 
                                int neighbor5 = (int) CV_MAT_ELEM(*pSrc, float, i+1, j); 
                                int neighbor6 = (int) CV_MAT_ELEM(*pSrc, float, i+1, j-1); 
                                int neighbor7 = (int) CV_MAT_ELEM(*pSrc, float, i, j-1); 
                                int C = int(~neighbor1 & ( neighbor2 | neighbor3)) + 
                                                 int(~neighbor3 & ( neighbor4 | neighbor5)) + 
                                                 int(~neighbor5 & ( neighbor6 | neighbor7)) + 
                                                 int(~neighbor7 & ( neighbor0 | neighbor1)); 
                                if(C == 1) { 
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
                                        if ((N == 2) || (N == 3)) { 
                                                /// calculate criteria 3 
                                                int c3 = ( neighbor1 | neighbor2 | ~neighbor4) & neighbor3;	
                                                if(c3 == 0) { 
                                                        CV_MAT_ELEM(*pDst, float, i, j) = 0.0f; 
                                                } 
                                        } 
                                } 
                        } 
                } 
        } 
} 


void ThinSubiteration2(CvMat *pSrc, CvMat *pDst) { 
        int rows = pSrc->rows; 
        int cols = pSrc->cols; 
        cvCopy(pSrc, pDst); 
        for(int i = 0; i < rows; i++) { 
                for(int j = 0; j < cols; j++) { 
                        if ( CV_MAT_ELEM(*pSrc, float, i, j) == 1.0f) { 
                                /// get 8 neighbors 
                                /// calculate C(p) 
                                int neighbor0 = (int) CV_MAT_ELEM(*pSrc, float, i-1, j-1); 
                                int neighbor1 = (int) CV_MAT_ELEM(*pSrc, float, i-1, j); 
                                int neighbor2 = (int) CV_MAT_ELEM(*pSrc, float, i-1, j+1); 
                                int neighbor3 = (int) CV_MAT_ELEM(*pSrc, float, i, j+1); 
                                int neighbor4 = (int) CV_MAT_ELEM(*pSrc, float, i+1, j+1); 
                                int neighbor5 = (int) CV_MAT_ELEM(*pSrc, float, i+1, j); 
                                int neighbor6 = (int) CV_MAT_ELEM(*pSrc, float, i+1, j-1); 
                                int neighbor7 = (int) CV_MAT_ELEM(*pSrc, float, i, j-1); 
                                int C = int(~neighbor1 & ( neighbor2 | neighbor3)) + 
                                        int(~neighbor3 & ( neighbor4 | neighbor5)) + 
                                        int(~neighbor5 & ( neighbor6 | neighbor7)) + 
                                        int(~neighbor7 & ( neighbor0 | neighbor1)); 
                                if(C == 1) { 
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
                                        if((N == 2) || (N == 3)) { 
                                                int E = (neighbor5 | neighbor6 | ~neighbor0) & neighbor7; 
                                                if(E == 0) { 
                                                        CV_MAT_ELEM(*pDst, float, i, j) = 0.0f; 
                                                } 
                                        } 
                                } 
                        } 
                } 
        } 
} 



int main(int argc, char** argv) 
{
    IplImage *src=0, *img = 0, *temp = 0, *eroded = 0;
    IplImage *skel =0;
    
    if(argc != 2){return -1;}
    if((src = cvLoadImage( argv[1], CV_LOAD_IMAGE_GRAYSCALE)) == 0){
    return -1;}
    
    int percent = 25;
    IplImage *destination = cvCreateImage( cvSize((int)((src->width*percent)/100) , (int)((src->height*percent)/100)),IPL_DEPTH_8U, 1);
    cvResize(src, destination);
    
    // Create the grayscale output images
    img = cvCreateImage(cvSize(destination->width,destination->height),IPL_DEPTH_8U, 1);
    temp = cvCreateImage(cvSize(destination->width,destination->height),IPL_DEPTH_8U, 1);
    eroded = cvCreateImage(cvSize(destination->width,destination->height),IPL_DEPTH_8U, 1);
    skel = cvCreateImage(cvSize(destination->width,destination->height),IPL_DEPTH_8U, 1);
    
    cvThreshold(destination, img, BINARY_THRESH, 255, CV_THRESH_BINARY);
	 
	 cvDilate(img, img, NULL, 5);
	 cvErode(img, img, NULL, 5);
	 
    cvNamedWindow("binary",1);
    cvShowImage("binary", img);
    
    cvSaveImage( "/home/baoxing/bindary.jpg", img );
    
    CvMat *mat = cvCreateMat( img->height, img->width, CV_32FC1);
    CvMat *Output_mat = cvCreateMat( img->height, img->width, CV_32FC1);
	 cvConvert( img, mat);
	 
	 int rows = mat->rows; 
	 int cols = mat->cols;
	 for(int i = 0; i < rows; i++) { 
			 for(int j = 0; j < cols; j++) { 
								  float tmp_value = CV_MAT_ELEM(*mat, float, i, j) ; 
								  
								  CV_MAT_ELEM(*mat, float, i, j) = tmp_value/255.0;
			 } 
    }
        
	 MorphologicalThinning(mat, Output_mat);
	 
	 for(int i = 0; i < rows; i++) { 
		 for(int j = 0; j < cols; j++) { 
			  float tmp_value = CV_MAT_ELEM(*Output_mat, float, i, j) ; 
			  CV_MAT_ELEM(*Output_mat, float, i, j) = floorf(tmp_value*255.0);
		 } 
    }
    cvConvert( Output_mat, skel);
    
    
    /*
	 IplConvKernel* kernel = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_CROSS, NULL);
	 
	 bool done;		
	 do
	 {
		cvErode(img, eroded, kernel);
		cvDilate(eroded, temp, kernel); // temp = open(img)
		cvSub(img, temp, temp);
		cvOr(skel, temp, skel);
		cvCopy(eroded, img);
	 
	  done = (cvNorm(img) == 0);
	 } while (!done);
	 
	 */
    cvSaveImage( "/home/baoxing/skel.jpg", skel );

	 cvNamedWindow("skeleton",1);
    cvShowImage("skeleton", skel);
	 
	 cvWaitKey();

    cvReleaseImage(&src);
    cvReleaseImage(&img);
    cvReleaseImage(&temp);
    cvReleaseImage(&eroded);
    cvReleaseImage(&skel);
	 //cvReleaseStructuringElement(&kernel);
	 
    cvDestroyWindow("skeleton");
    
    return 0;
}

