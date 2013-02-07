#include "ped_crossing_detection.h"

namespace golfcar_vision{
  
    ped_cross_detect::ped_cross_detect(double scale)
    {
		 lane_extractor_ = new continuous_lane();
		 scale_ = scale;
	 }
  
    void ped_cross_detect::Detect_PdCrossing (IplImage* src)
    {
        IplImage *Iat = 0;
        Iat = cvCreateImage(cvSize(src->width,src->height),IPL_DEPTH_8U, 1);
        cvAdaptiveThreshold(src, Iat, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, BLOCK_SIZE, OFFSET);
        cvShowImage("PDC: Iat_image",Iat);

        CvSeq *contours = 0;            //"contours" is a list of contour sequences, which is the core of "image_proc";
        CvSeq *first_contour = 0;       //always keep one copy of the beginning of this list, for further usage;
        CvMemStorage *mem_contours; 
        mem_contours = cvCreateMemStorage(0);
        
        CvContourScanner scanner = cvStartFindContours(Iat, mem_contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
                
        contours = Filter_candidates(scanner);
        first_contour = contours;
      
        IplImage *contour_img = cvCreateImage(cvSize(src->width,src->height),IPL_DEPTH_8U, 3);
        cvCvtColor(Iat, contour_img, CV_GRAY2BGR);
        CvScalar ext_color;
        
        vision_lane_detection::conti_lanes lines;
        for (; contours != 0; contours = contours->h_next)
        {
            ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 ); 
				lane_extractor_ ->ransac_lane(contours, contour_img, ext_color, lines);  
		  }
		  contours = first_contour;
		  
		  std::vector<unsigned int> crossing_lanes =  ped_cross_detect::cluster_lines (lines);
		  if(crossing_lanes.size() >= 4) 
		  {
			 CvFont font;
			 double hScale=1.0;
			 double vScale=1.0;
			 int lineWidth=2;
			 CvPoint origin;
			 stringstream  vis_string;
			 vis_string<<"crossing detection: lanes "<< crossing_lanes.size(); 
			 const char *vis_chr = vis_string.str().c_str();
			 origin.x = 320;
			 origin.y = 180;
			 cvInitFont(&font,CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);
			 cvPutText(contour_img, vis_chr, origin, &font, CV_RGB(0,255,0));
		  }
		  
        cvShowImage("PDC: contour_image", contour_img);
        cvReleaseMemStorage(&mem_contours);
        cvWaitKey(10);
        cvReleaseImage(&Iat);
        cvReleaseImage(&contour_img);
    }
    
    std::vector<unsigned int> ped_cross_detect::cluster_lines (vision_lane_detection::conti_lanes &lines)
    {
		 std::vector<std::vector< std::pair< unsigned int, double> > > clusters;

		 for(size_t i=0; i<lines.lanes.size(); i++)
		 {
			 double angle_tmp;
			 if(lines.lanes[i].params[1]==0) angle_tmp = M_PI_2;
			 else {
				 angle_tmp = atan2(lines.lanes[i].params[0], -lines.lanes[i].params[1]);
				 if(angle_tmp < 0) angle_tmp = angle_tmp+M_PI_2;
			 }
			 std::pair< unsigned int, double> line_angle_tmp = make_pair(i, angle_tmp);
			 
			 bool find_cluster = false;
			 for(size_t j=0; j<clusters.size(); j++)
			 {
				 double max_delta_angle = 0.0;
				 for(size_t k=0; k < clusters[j].size(); k++)
				 {
					 double delta_angle = fabs(angle_tmp - clusters[j][k].second);
					 if(max_delta_angle < delta_angle) max_delta_angle = delta_angle;
				 }
				 //ROS_WARN("Maximum delta angle: %5f", max_delta_angle);
				 if(max_delta_angle < LANES_CLUSTER_ANGLE_THRESH)
				 {
					 clusters[j].push_back(line_angle_tmp);
					 break;
					 //ROS_WARN("cluster now number-- %ld", clusters[j].size() );
				 }
			 }
			 
			 if(!find_cluster)
			 {
				 std::vector< std::pair< unsigned int, double> > cluster_tmp;
				 cluster_tmp.push_back(line_angle_tmp);
				 clusters.push_back(cluster_tmp);
			 }
		 }
		 
		 unsigned int largest_size = 0;
		 bool cluster_exist = false;
		 unsigned int largest_cluster_serial;
		 
		 for(size_t i=0; i<clusters.size(); i++)
		 {
			 if(clusters[i].size() > largest_size)
			 { largest_cluster_serial = i; largest_size = clusters[i].size(); cluster_exist = true;}
		 }
		 std::vector<unsigned int> crossing_lanes_serial;
		 
		 if(cluster_exist)
		 {
			 ROS_WARN("crossing_lanes number %ld",  clusters[largest_cluster_serial].size());
			 for(size_t i=0; i<clusters[largest_cluster_serial].size(); i++)
			 {
				 crossing_lanes_serial.push_back(clusters[largest_cluster_serial][i].first);
			 }
		 }
		 return crossing_lanes_serial;
	 }
    
    CvSeq* ped_cross_detect::Filter_candidates (CvContourScanner &scanner)
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
                float short_side = min(height, width);
                if(long_side<LONG_SIDE_THRESH*scale_ &&  short_side > 0.3 * scale_)
                {
                    cvSubstituteContour(scanner, NULL);
                }
                else
                {
						 CvSeq *contours;
						 CvMemStorage *mem_poly;
                   mem_poly = cvCreateMemStorage(0);
						 contours = cvApproxPoly( c, sizeof(CvContour), mem_poly, CV_POLY_APPROX_DP, 5, 0 );
						 if(contours->total != 4) cvSubstituteContour(scanner, NULL);
                }   
            }                       
        }
        CvSeq *contours = cvEndFindContours(&scanner);
        cvReleaseMemStorage(&mem_box);
        return contours;
    }

    ped_cross_detect::~ped_cross_detect()
    {
    }
};
