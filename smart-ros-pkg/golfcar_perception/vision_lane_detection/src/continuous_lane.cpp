#include "continuous_lane.h"

namespace golfcar_vision{
	
	continuous_lane::continuous_lane()
	{
		printf("continous_lane constructed");
	}
		
	void continuous_lane::ransac_lane(CvSeq *contours, IplImage *contour_img, CvScalar ext_color, vision_lane_detection::conti_lanes & lanes_inImg)
	{
		std::vector <TPoint2D> contour_points;
		for(int i=0; i<contours->total; i++)
		{
			CvPoint* p = (CvPoint*)cvGetSeqElem(contours, i);
			TPoint2D pt_tmp;
			pt_tmp.x = p -> x;
			pt_tmp.y = p -> y;
			contour_points.push_back(pt_tmp);
		}
		
		//-------------------------------------------------------------------------------------------------------------------------
		//preparation work: to reduce the polygon of the contour into a thin single line;
		//-------------------------------------------------------------------------------------------------------------------------
		ROS_INFO("step1");
		std::vector <TPoint2D> fused_points;
		if(contour_points.size()<4) return;
		
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
			//cvCircle( contour_img, cvPoint(fused_points[i].x,fused_points[i].y), 2, ext_color, 1);
		}
		ROS_INFO("fused points number %ld", xs.size());
		//--------------------------------------------------------------------------------------------------------------
		
		//--------------------------------------------------------------------------------------------------------------
		// non-lane contours will take much time to process; 
		// add pre-filtering step for those nonlane contours;
		//--------------------------------------------------------------------------------------------------------------
		
		/*
		vector<pair<mrpt::vector_size_t, parabola> >   detectedLines;
		//temporarily the unit is still pixel;
		const double DIST_THRESHOLD = 5;
		ransac_detect_parabolas(xs,ys,detectedLines,DIST_THRESHOLD, 100 );
		//--------------------------------------------------------------------------------------------------------------
		//try to visualize the detected parabola;
		//--------------------------------------------------------------------------------------------------------------
		for (vector<pair<mrpt::vector_size_t,parabola> >::iterator p=detectedLines.begin();p!=detectedLines.end();++p)
		{
			CvScalar random_color;
			random_color = CV_RGB( rand()&255, rand()&255, rand()&255 );
			if(p->second.coefs[3]==1.0)
			{
				for(unsigned int yPixel=0; yPixel<360; yPixel++)
				{
					unsigned int xPixel = p->second.coefs[0]*yPixel*yPixel+p->second.coefs[1]*yPixel+p->second.coefs[2];
					cvCircle( contour_img, cvPoint(xPixel,yPixel), 2, ext_color, 1);
				}
			}
			else
			{
				for(unsigned int xPixel=0; xPixel<640; xPixel++)
				{
					unsigned int yPixel = p->second.coefs[0]*xPixel*xPixel+p->second.coefs[1]*xPixel+p->second.coefs[2];
					cvCircle( contour_img, cvPoint(xPixel,yPixel), 2, ext_color, 1);
				}
			}
		}
		*/
		
		vector<pair<mrpt::vector_size_t,TLine2D > >   detectedLines; 
		const double DIST_THRESHOLD = 5;
		ransac_detect_2dLines(xs,ys,detectedLines,DIST_THRESHOLD, 100);
		for (vector<pair<mrpt::vector_size_t,TLine2D> >::iterator p=detectedLines.begin();p!=detectedLines.end();++p)
		{
			vision_lane_detection::conti_lane lanetmp;
			lanetmp.points.clear();
			
			CvScalar random_color;
			random_color = CV_RGB( rand()&255, rand()&255, rand()&255 );
			for(unsigned int id =0; id < p->first.size(); id++)
			{
				geometry_msgs::Point32 pttmp;
				pttmp.x = (float)xs[p->first[id]];
				pttmp.y = (float)ys[p->first[id]];
				lanetmp.points.push_back(pttmp);
				
				int xPixel = (int)xs[p->first[id]];
				int yPixel = (int)ys[p->first[id]];
				cvCircle( contour_img, cvPoint(xPixel,yPixel), 2, ext_color, 1);
			}
			lanes_inImg.lanes.push_back(lanetmp);
		}
	}

};
