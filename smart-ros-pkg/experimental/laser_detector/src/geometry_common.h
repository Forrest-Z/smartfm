#ifndef SMART_GEOMETRY_COMMON_H
#define SMART_GEOMETRY_COMMON_H

#include "pcl/point_types.h"
using namespace geometry_msgs;
using namespace std;

namespace golfcar_pcl{

	//http://alienryderflex.com/polygon/
	inline bool pointInPolygon(pcl::PointXYZ p, vector<pcl::PointXYZ> poly)
	{
		int polySides = poly.size();
		int      i, j=polySides-1 ;
		bool  oddNodes = false      ;

		for (i=0; i<polySides; i++) {
			if (((poly[i].y< p.y && poly[j].y>=p.y)
			      ||   (poly[j].y< p.y && poly[i].y>=p.y))
					&&  (poly[i].x<=p.x || poly[j].x<=p.x)) {
				          if(poly[i].x+(p.y-poly[i].y)/(poly[j].y-poly[i].y)*(poly[j].x-poly[i].x)<p.x)
							{oddNodes=!oddNodes;} 			}
			j=i; }

		return oddNodes;
	}
	

};

#endif
