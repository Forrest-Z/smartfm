#ifndef LANER_MARKER_COMMON_H
#define LANER_MARKER_COMMON_H

#include <geometry_msgs/Point.h>
using namespace geometry_msgs;
using namespace std;

namespace golfcar_pcl{

	template<class T> 
	bool pointInPolygon(T p, vector<Point32> poly)
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
