//20121112

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <fmutil/fm_math.h>

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



void Graph_Extraction(CvMat *pSrc, CvMat *pDst, CvMat *pDst2, std::vector<CvPoint2D32f> & node_points) 
{ 
			node_points.clear();
        int rows = pSrc->rows; 
        int cols = pSrc->cols; 
        /// pad source 
        CvMat *p_enlarged_src = cvCreateMat(rows + 2, cols + 2, CV_32FC1); 
        CvMat *p_enlarged_dst = cvCreateMat(rows + 2, cols + 2, CV_32FC1); 
        CvMat *p_enlarged_dst2 = cvCreateMat(rows + 2, cols + 2, CV_32FC1); 
        
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
        
        for(int i = 0; i < rows; i++) { 
                for(int j = 0; j < cols; j++) { 
                        if ( CV_MAT_ELEM(*p_enlarged_src, float, i, j) == 1.0f) { 
                                /// get 8 neighbors 
                                /// calculate C(p) 
                                int neighbor0 = (int) CV_MAT_ELEM(*p_enlarged_src, float, i-1, j-1); 
                                int neighbor1 = (int) CV_MAT_ELEM(*p_enlarged_src, float, i-1, j); 
                                int neighbor2 = (int) CV_MAT_ELEM(*p_enlarged_src, float, i-1, j+1); 
                                int neighbor3 = (int) CV_MAT_ELEM(*p_enlarged_src, float, i, j+1); 
                                int neighbor4 = (int) CV_MAT_ELEM(*p_enlarged_src, float, i+1, j+1); 
                                int neighbor5 = (int) CV_MAT_ELEM(*p_enlarged_src, float, i+1, j); 
                                int neighbor6 = (int) CV_MAT_ELEM(*p_enlarged_src, float, i+1, j-1); 
                                int neighbor7 = (int) CV_MAT_ELEM(*p_enlarged_src, float, i, j-1); 
                                
                                int Count = neighbor0 + neighbor1 + neighbor2 + neighbor3 + neighbor4 +
															neighbor5 + neighbor6 + neighbor7;

										  if(Count > 2) {
											  CV_MAT_ELEM(*p_enlarged_dst, float, i, j) = 1.0f;
											  
											  node_points.push_back(cvPoint2D32f(j, i));
											  
											  } 
										  else
										  {
											  //only record the remained edge points;
											  CV_MAT_ELEM(*p_enlarged_dst2, float, i, j) = 1.0f;
										  }
                        } 
                } 
        }

        /// copy result 
        for(int i = 0; i < rows; i++) { 
                for(int j = 0; j < cols; j++) { 
                        CV_MAT_ELEM(*pDst, float, i, j) = CV_MAT_ELEM(*p_enlarged_dst, float, i+1, j+1); 
                        CV_MAT_ELEM(*pDst2, float, i, j) = CV_MAT_ELEM(*p_enlarged_dst2, float, i+1, j+1); 
                        
                } 
        } 
        /// clean memory 
        cvReleaseMat(&p_enlarged_src); 
        cvReleaseMat(&p_enlarged_dst); 
        cvReleaseMat(&p_enlarged_dst2); 
} 


//http://nghiaho.com/?p=1102
void Extract_Edges(CvMat* pSrc, std::vector < std::vector<CvPoint> > & edges)
{
    edges.clear();
    CvMat *label_image = cvCloneMat( pSrc );

    int label_count = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y < label_image->rows; y++) {
        for(int x=0; x < label_image->cols; x++) {
            if((int)CV_MAT_ELEM(*label_image, float, y, x) != 1) {
                continue;
            }
            
            cvFloodFill(label_image, cvPoint(x,y), cvScalar(label_count), cvScalar(0), cvScalar(0), NULL, 8, NULL);

            std::vector <CvPoint> edge;

            for(int i= 0 ; i < label_image->rows; i++) {
                for(int j=0; j < label_image->cols; j++) {
						 
                    if((int)CV_MAT_ELEM(*label_image, float, i, j)!= label_count) 
                    {
                        continue;
                    }
							
                    edge.push_back(cvPoint(j,i));
                    //CV_MAT_ELEM(*label_image, float, i, j) = 0.0;
                }
            }

            edges.push_back(edge);

            label_count++;
        }
    }
}


int main(int argc, char** argv) 
{
    IplImage *src=0, *img = 0, *temp = 0, *eroded = 0;
    IplImage *skel =0, *node =0, *edge = 0, *color_edge = 0;
    
    if(argc != 2){return -1;}
    if((src = cvLoadImage( argv[1], CV_LOAD_IMAGE_GRAYSCALE)) == 0){
    return -1;}

    int percent = 100;

    IplImage *destination = cvCreateImage( cvSize((int)((src->width*percent)/100) , (int)((src->height*percent)/100)),IPL_DEPTH_8U, 1);
    cvResize(src, destination);
    
    // Create the grayscale output images
    img = cvCreateImage(cvSize(destination->width,destination->height),IPL_DEPTH_8U, 1);
    temp = cvCreateImage(cvSize(destination->width,destination->height),IPL_DEPTH_8U, 1);
    eroded = cvCreateImage(cvSize(destination->width,destination->height),IPL_DEPTH_8U, 1);
    skel = cvCreateImage(cvSize(destination->width,destination->height),IPL_DEPTH_8U, 1);
    node = cvCreateImage(cvSize(destination->width,destination->height),IPL_DEPTH_8U, 1);
    edge = cvCreateImage(cvSize(destination->width,destination->height),IPL_DEPTH_8U, 1);
    color_edge = cvCreateImage(cvSize(destination->width,destination->height),IPL_DEPTH_8U, 3); 
    
    cvThreshold(destination, img, BINARY_THRESH, 255, CV_THRESH_BINARY);

    cvErode(img, img, NULL, 1);
    cvDilate(img, img, NULL, 3);
    cvErode(img, img, NULL, 2);

    cvNamedWindow("binary",1);
    cvShowImage("binary", img);
    
    cvSaveImage( "binary.jpg", img );
    
    CvMat *mat = cvCreateMat( img->height, img->width, CV_32FC1);
    CvMat *Output_mat = cvCreateMat( img->height, img->width, CV_32FC1);
    CvMat *node_mat = cvCreateMat( img->height, img->width, CV_32FC1);
    CvMat *edge_mat = cvCreateMat( img->height, img->width, CV_32FC1);
    
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
	 
	 std::vector<CvPoint2D32f> node_points;
	 Graph_Extraction(Output_mat, node_mat, edge_mat, node_points);
	 
	 //do simple clustering for the node points;
	 std::vector <std::vector<CvPoint2D32f> > node_clusters;
	 for(size_t i=0; i<node_points.size(); i++)
	 {
		 CvPoint2D32f point_tmp = node_points[i];
		 int cluster_serial = -1;
		 
		 for(size_t j=0; j<node_clusters.size(); j++)
		 {
			   if(cluster_serial !=-1) break;
				for(size_t k=0; k<node_clusters[j].size(); k++)
				{
					CvPoint2D32f point_tmpp;
					if(fmutil::distance(point_tmp, point_tmpp) < 5.0) 
					{
						cluster_serial = j;
						break;
					}
				}
		 }
		 if(cluster_serial == -1)
		 {
				std::vector<CvPoint2D32f> cluster_tmp;
				cluster_tmp.push_back(point_tmp);
				node_clusters.push_back(cluster_tmp);
		 }
		 else
		 {
				node_clusters[cluster_serial].push_back(point_tmp);
		 }
	 }
	 
	 std::vector<CvPoint>  merged_nodes;
	 for(size_t i=0; i<node_clusters.size(); i++)
	 {
		 float points_number = (float)node_clusters[i].size();
		 float x_tmp =0.0; float  y_tmp = 0.0;
		 for(size_t j=0; j<node_clusters[i].size(); j++)
		 {
			 x_tmp = x_tmp + node_clusters[i][j].x;
			 y_tmp = y_tmp + node_clusters[i][j].y;
		 }
		 x_tmp = x_tmp / points_number;
		 y_tmp = y_tmp / points_number;
		 merged_nodes.push_back(cvPoint(x_tmp, y_tmp));

		 cvCircle( color_edge, cvPoint(x_tmp, y_tmp), 5, CV_RGB(0,255,0), 2);
	 }
	 
	 
	 	 
	 
	 std::vector < std::vector<CvPoint> > edges;
    Extract_Edges(edge_mat, edges);
	 
	 for(size_t i=0; i < edges.size(); i++) {
					CvScalar ext_color;
					ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 );
					ext_color = CV_RGB( 0, 0, 255 );
        
        for(size_t j=0; j < edges[i].size(); j++) {
            int x = edges[i][j].x;
            int y = edges[i][j].y;
				cvSet2D(color_edge, y, x, ext_color);
        }
    }	
   
	 for(int i = 0; i < rows; i++) { 
		 for(int j = 0; j < cols; j++) { 
			  float tmp_value = CV_MAT_ELEM(*Output_mat, float, i, j) ; 
			  CV_MAT_ELEM(*Output_mat, float, i, j) = floorf(tmp_value*255.0);
		 } 
    }
    cvConvert( Output_mat, skel);
    
    
	for(int i = 0; i < rows; i++) { 
		 for(int j = 0; j < cols; j++) { 
			  float tmp_value = CV_MAT_ELEM(*node_mat, float, i, j) ; 
			  CV_MAT_ELEM(*node_mat, float, i, j) = floorf(tmp_value*255.0);
		 } 
    }
    cvConvert( node_mat, node);
    
   for(int i = 0; i < rows; i++) { 
		 for(int j = 0; j < cols; j++) { 
			  float tmp_value = CV_MAT_ELEM(*edge_mat, float, i, j) ; 
			  CV_MAT_ELEM(*edge_mat, float, i, j) = floorf(tmp_value*255.0);
		 } 
    }
    cvConvert( edge_mat, edge);
    
    
    
    
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
    cvSaveImage( "skel.jpg", skel );
	 cvNamedWindow("skeleton",1);
    cvShowImage("skeleton", skel);
    
    cvSaveImage( "node.jpg", node );
	 cvNamedWindow("node",1);
    cvShowImage("node", node);
    
    cvSaveImage( "edge.jpg", edge );
	 cvNamedWindow("edge",1);
    cvShowImage("edge", edge);
    
    cvSaveImage( "color_edge.jpg", color_edge );
	 cvNamedWindow("color_edge",1);
    cvShowImage("color_edge", color_edge);
    
    
	 
	 cvWaitKey();

    cvReleaseImage(&src);
    cvReleaseImage(&img);
    cvReleaseImage(&temp);
    cvReleaseImage(&eroded);
    cvReleaseImage(&skel);
    cvReleaseImage(&node);
    cvReleaseImage(&edge);
	 //cvReleaseStructuringElement(&kernel);
	 
    cvDestroyWindow("skeleton");
    cvDestroyWindow("node");
    cvDestroyWindow("edge");
    return 0;
}

