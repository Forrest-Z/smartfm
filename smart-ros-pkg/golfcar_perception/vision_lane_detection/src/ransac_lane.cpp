#include "ransac_lane.h"

namespace golfcar_vision{
	
	ransac_lane::ransac_lane()
	{
		printf("ransac_lane constructed");
	}
		
	void ransac_lane::multiple_lanes(CvSeq *contours, double scale, IplImage *thining_img, IplImage *contour_img, int contour_serial, vision_lane_detection::lanes_info & lanes_inImg)
	{
		IplImage *tmp_img = cvCreateImage(cvSize(thining_img->width,thining_img->height),IPL_DEPTH_8U, 1);
		CvMat *thining_mat = cvCreateMat( thining_img->height, thining_img->width, CV_8UC1);
		CvMat *output_mat = cvCreateMat( thining_img->height, thining_img->width, CV_8UC1);
		cvZero(tmp_img);
		cvZero(thining_mat);
		cvZero(output_mat);

		cvDrawContours(thining_mat, contours, cvScalar(255),  cvScalar(0), -1, CV_FILLED, 8, cvPoint(0,0));
		MorphologicalThinning(thining_mat, output_mat);
		cvConvert( output_mat, tmp_img);
		cvOr(thining_img, tmp_img, thining_img);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		cloud->clear();
		cloud->is_dense = false;
		cloud->height   = 1;

		for(int i = 0; i < output_mat->rows; i++) {
			for(int j = 0; j < output_mat->cols; j++) {
				if (CV_MAT_ELEM(*output_mat, uchar, i, j) == 255)
				{
					pcl::PointXYZ pointtmp(j, i, 0);
					cloud->push_back(pointtmp);
				}
			}
		}

		float total_points_number = (float)cloud->points.size();
		float residual_sample_ratio = 1.0;
		int loop_number = 0;

		while(residual_sample_ratio>0.3 && loop_number < 5){

			std::vector<int> inliers;
			pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(new pcl::SampleConsensusModelLine <pcl::PointXYZ> (cloud));
			pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_line);
			Eigen::VectorXf line_coefficients;

			ransac.setDistanceThreshold (0.1*scale);
			ransac.computeModel();
			ransac.getInliers(inliers);
			ransac.getModelCoefficients(line_coefficients);


			pcl::ExtractIndices<pcl::PointXYZ> extract;
			pcl::PointIndices::Ptr inline_indices (new pcl::PointIndices);
			inline_indices->indices = inliers;

			/*
			printf("\n line coefficient \n");
			for(int i=0; i<line_coefficients.rows(); i++)
			{
				printf("%3f\t", line_coefficients[i]);
			}
			*/

			pcl::PointCloud<pcl::PointXYZ>::Ptr inline_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			extract.setInputCloud (cloud->makeShared());
			extract.setIndices (inline_indices);
			extract.setNegative (false);
			extract.filter (*inline_cloud);
			extract.setNegative (true);
			extract.filter (*cloud);

			loop_number++;
			residual_sample_ratio = ((float)cloud->points.size())/total_points_number;
			float sample_ratio = ((float)inliers.size())/total_points_number;

			bool good_line = false;
			if(sample_ratio > 0.3) good_line = true;

			if(good_line)
			{
				vision_lane_detection::lane_info current_line;
				current_line.contour_serial = contour_serial;
				current_line.params[0] = line_coefficients[3];
				current_line.params[1] = line_coefficients[4];
				current_line.params[2] = -line_coefficients[3]*line_coefficients[0]-line_coefficients[4]*line_coefficients[1];
				for(size_t i=0; i< inline_cloud->points.size(); i++)
				{
					geometry_msgs::Point32 pointtmp;
					pointtmp.x=inline_cloud->points[i].x;
					pointtmp.y=inline_cloud->points[i].y;
					pointtmp.z=0.0;
					current_line.points.push_back(pointtmp);
				}
				lanes_inImg.lanes.push_back(current_line);


				CvPoint lower_left, upper_right;
				lower_left.x = 1000;
				lower_left.y = 1000;
				upper_right.x = 0;
				upper_right.y = 0;
				for(size_t j=0; j<inline_cloud->points.size(); j++)
				{
					if((inline_cloud->points[j].x < lower_left.x)||(inline_cloud->points[j].x == lower_left.x && inline_cloud->points[j].y < lower_left.y))
					{
						lower_left.x = inline_cloud->points[j].x;
						lower_left.y = inline_cloud->points[j].y;
					}
					if((inline_cloud->points[j].x > upper_right.x)||(inline_cloud->points[j].x == upper_right.x && inline_cloud->points[j].y > upper_right.y))
					{
						upper_right.x = inline_cloud->points[j].x;
						upper_right.y = inline_cloud->points[j].y;
					}
				}
				cvLine( contour_img, lower_left, upper_right, CV_RGB(0,0,255), 2);
			}
		}

		/*
		//use HoughLine: however, Hough-line may yield out several lines which are actually one line,
		//due to the discrete angle and rho in Hough Transform;

		std::vector<CvPoint> points;
		for(int i = 0; i < output_mat->rows; i++) {
			for(int j = 0; j < output_mat->cols; j++) {
				if (CV_MAT_ELEM(*output_mat, uchar, i, j) == 255) {
					points.push_back(cvPoint(j,i));
				}
			}
		}

		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq* lines = 0;
		int i;
		lines = cvHoughLines2( tmp_img,
							   storage,
							   CV_HOUGH_STANDARD,
							   1,
							   CV_PI/180*1,
							   100,
							   0,
							   0 );

		ROS_INFO("lines number %d", lines->total );
		for( i = 0; i < MIN(lines->total,10); i++ )
		{

			float* line = (float*)cvGetSeqElem(lines,i);
			float rho = line[0];
			float theta = line[1];
			vision_lane_detection::lane_info lane_tmp;
			lane_tmp.contour_serial = contour_serial;
			double line_params[3];

			//pay attention to the angle representation;
			//http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/hough_lines/hough_lines.html
			line_params[0] = cos(theta);
			line_params[1] = sin(theta);
			line_params[2] = -rho;
			lane_tmp.params[0] = line_params[0];
			lane_tmp.params[1] = line_params[1];
			lane_tmp.params[2] = line_params[2];

			printf("line parameter: %5f, %5f, %5f\n", line_params[0], line_params[1], line_params[2]);

			CvPoint lower_left, upper_right;
			lower_left.x = 1000;
			lower_left.y = 1000;
			upper_right.x = 0;
			upper_right.y = 0;

			for(int j=0; j<points.size(); j++)
			{
				double dis_point_to_line = fabs(line_params[0]*points[j].x + line_params[1]*points[j].y+ line_params[2]);
				if(dis_point_to_line < 6)
				{
					geometry_msgs::Point32 point_current;
					point_current.x = points[j].x;
					point_current.y = points[j].y;
					point_current.z = 0;
					lane_tmp.points.push_back(point_current);
					if((points[j].x < lower_left.x)||(points[j].x == lower_left.x && points[j].y < lower_left.y))lower_left = points[j];
					if((points[j].x > upper_right.x)||(points[j].x == upper_right.x && points[j].y > upper_right.y))upper_right = points[j];
				}
			}
			cvLine( contour_img, lower_left, upper_right, CV_RGB(0,0,255), 1);
		}
		cvReleaseMemStorage(&storage);
		*/


        cvWaitKey(1);
		cvReleaseImage(&tmp_img);
		cvReleaseMat(&thining_mat);
		cvReleaseMat(&output_mat);

	}

	void ransac_lane::MorphologicalThinning(CvMat *pSrc, CvMat *pDst) {

			ROS_INFO("image thinning");
		        bool bDone = false;
		        int rows = pSrc->rows;
		        int cols = pSrc->cols;
		        /// pad source
		        CvMat *p_enlarged_src = cvCreateMat(rows + 2, cols + 2, CV_8UC1);
		        for(int i = 0; i < (rows+2); i++) {
		                CV_MAT_ELEM(*p_enlarged_src, uchar, i, 0)	 = 0;
		                CV_MAT_ELEM(*p_enlarged_src, uchar, i, cols+1)	= 0;
		        }
		        for(int j = 0; j < (cols+2); j++) {
		                CV_MAT_ELEM(*p_enlarged_src, uchar, 0, j)	 = 0;
		                CV_MAT_ELEM(*p_enlarged_src, uchar, rows+1, j)	= 0;
		        }
		        for(int i = 0; i < rows; i++) {
		                for(int j = 0; j < cols; j++) {
		                        if (CV_MAT_ELEM(*pSrc, uchar, i, j) >= 255/2) {
		                                CV_MAT_ELEM(*p_enlarged_src, uchar, i+1, j+1) = 255;
		                        }
		                        else CV_MAT_ELEM(*p_enlarged_src, uchar, i+1, j+1) = 0;
		                }
		        }
		        /// start to thin
		        CvMat *p_thinMat1	= cvCreateMat(rows + 2, cols + 2, CV_8UC1);
		        CvMat *p_thinMat2	= cvCreateMat(rows + 2, cols + 2, CV_8UC1);
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
		                        CV_MAT_ELEM(*pDst, uchar, i, j) = CV_MAT_ELEM(*p_enlarged_src, uchar, i+1, j+1);
		                }
		        }
		        /// clean memory
		        cvReleaseMat(&p_enlarged_src);
		        cvReleaseMat(&p_thinMat1);
		        cvReleaseMat(&p_thinMat2);
		        cvReleaseMat(&p_cmp);
		}

		void ransac_lane::ThinSubiteration1(CvMat *pSrc, CvMat *pDst) {
		        int rows = pSrc->rows;
		        int cols = pSrc->cols;
		        cvCopy(pSrc, pDst);
		        for(int i = 0; i < rows; i++) {
		                for(int j = 0; j < cols; j++) {
		                        if(CV_MAT_ELEM(*pSrc, uchar, i, j) == 255) {
		                                /// get 8 neighbors
		                                /// calculate C(p)
		                                int neighbor0 = (int) CV_MAT_ELEM(*pSrc, uchar, i-1, j-1);
		                                int neighbor1 = (int) CV_MAT_ELEM(*pSrc, uchar, i-1, j);
		                                int neighbor2 = (int) CV_MAT_ELEM(*pSrc, uchar, i-1, j+1);
		                                int neighbor3 = (int) CV_MAT_ELEM(*pSrc, uchar, i, j+1);
		                                int neighbor4 = (int) CV_MAT_ELEM(*pSrc, uchar, i+1, j+1);
		                                int neighbor5 = (int) CV_MAT_ELEM(*pSrc, uchar, i+1, j);
		                                int neighbor6 = (int) CV_MAT_ELEM(*pSrc, uchar, i+1, j-1);
		                                int neighbor7 = (int) CV_MAT_ELEM(*pSrc, uchar, i, j-1);
		                                int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
		                                                 int(~neighbor3 & ( neighbor4 | neighbor5)) +
		                                                 int(~neighbor5 & ( neighbor6 | neighbor7)) +
		                                                 int(~neighbor7 & ( neighbor0 | neighbor1));
		                                if(C == 1*255) {
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
		                                        if ((N == 2*255) || (N == 3*255)) {
		                                                /// calculate criteria 3
		                                                int c3 = ( neighbor1 | neighbor2 | ~neighbor4) & neighbor3;
		                                                if(c3 == 0) {
		                                                        CV_MAT_ELEM(*pDst, uchar, i, j) = 0;
		                                                }
		                                        }
		                                }
		                        }
		                }
		        }
		}


		void ransac_lane::ThinSubiteration2(CvMat *pSrc, CvMat *pDst) {
		        int rows = pSrc->rows;
		        int cols = pSrc->cols;
		        cvCopy(pSrc, pDst);
		        for(int i = 0; i < rows; i++) {
		                for(int j = 0; j < cols; j++) {
		                        if ( CV_MAT_ELEM(*pSrc, uchar, i, j) == 255) {
		                                /// get 8 neighbors
		                                /// calculate C(p)
		                                int neighbor0 = (int) CV_MAT_ELEM(*pSrc, uchar, i-1, j-1);
		                                int neighbor1 = (int) CV_MAT_ELEM(*pSrc, uchar, i-1, j);
		                                int neighbor2 = (int) CV_MAT_ELEM(*pSrc, uchar, i-1, j+1);
		                                int neighbor3 = (int) CV_MAT_ELEM(*pSrc, uchar, i, j+1);
		                                int neighbor4 = (int) CV_MAT_ELEM(*pSrc, uchar, i+1, j+1);
		                                int neighbor5 = (int) CV_MAT_ELEM(*pSrc, uchar, i+1, j);
		                                int neighbor6 = (int) CV_MAT_ELEM(*pSrc, uchar, i+1, j-1);
		                                int neighbor7 = (int) CV_MAT_ELEM(*pSrc, uchar, i, j-1);
		                                int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
		                                        int(~neighbor3 & ( neighbor4 | neighbor5)) +
		                                        int(~neighbor5 & ( neighbor6 | neighbor7)) +
		                                        int(~neighbor7 & ( neighbor0 | neighbor1));
		                                if(C == 1*255) {
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
		                                        if((N == 2*255) || (N == 3*255)) {
		                                                int E = (neighbor5 | neighbor6 | ~neighbor0) & neighbor7;
		                                                if(E == 0) {
		                                                        CV_MAT_ELEM(*pDst, uchar, i, j) = 0;
		                                                }
		                                        }
		                                }
		                        }
		                }
		        }
		}

};
