/*
 * pnpoly.h
 *
 *  Created on: Jun 1, 2012
 *      Author: demian
 */

#include <geometry_msgs/Point.h>
using namespace geometry_msgs;
using namespace std;
#define NPTS(v) sizeof(v)/sizeof(v[0])

// adapted from http://alienryderflex.com/polygon/
//  Globals which should be set before calling this function:
//
//  int    polySides  =  how many corners the polygon has
//  float  polyX[]    =  horizontal coordinates of corners
//  float  polyY[]    =  vertical coordinates of corners
//  float  x, y       =  point to be tested
//
//  (Globals are used in this example for purposes of speed.  Change as
//  desired.)
//
//  The function will return YES if the point x,y is inside the polygon, or
//  NO if it is not.  If the point is exactly on the edge of the polygon,
//  then the function may return YES or NO.
//
//  Note that division by zero is avoided because the division is protected
//  by the "if" clause which surrounds it.

bool pointInPolygon(Point p, vector<Point> poly)
{
	int polySides = poly.size();
	int      i, j=polySides-1 ;
	bool  oddNodes=false      ;

	for (i=0; i<polySides; i++) {
		if (((poly[i].y< p.y && poly[j].y>=p.y)
				||   (poly[j].y< p.y && poly[i].y>=p.y))
				&&  (poly[i].x<=p.x || poly[j].x<=p.x)) {
			oddNodes^=(poly[i].x+(p.y-poly[i].y)/(poly[j].y-poly[i].y)*(poly[j].x-poly[i].x)<p.x); }
		j=i; }

	return oddNodes;
}
namespace pnpoly
{
	// adapted from: (Changes made: vector is used with double point)
	// note: Doesn't work
	// Copyright 2001, softSurfer (www.softsurfer.com)
	// This code may be freely used and modified for any purpose
	// providing that this copyright notice is included with it.
	// SoftSurfer makes no warranty for this code, and cannot be held
	// liable for any real or imagined damage resulting from its use.
	// Users of this code must verify correctness for their application.

	//    a Point is defined by its coordinates {int x, y;}
	//===================================================================

	// isLeft(): tests if a point is Left|On|Right of an infinite line.
	//    Input:  three points P0, P1, and P2
	//    Return: >0 for P2 left of the line through P0 and P1
	//            =0 for P2 on the line
	//            <0 for P2 right of the line
	//    See: the January 2001 Algorithm "Area of 2D and 3D Triangles and Polygons"
	inline double
	isLeft( Point P0, Point P1, Point P2 )
	{
	    return ( (P1.x - P0.x) * (P2.y - P0.y)
	            - (P2.x - P0.x) * (P1.y - P0.y) );
	}
	//===================================================================

	// cn_PnPoly(): crossing number test for a point in a polygon
	//      Input:   P = a point,
	//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
	//      Return:  0 = outside, 1 = inside
	// This code is patterned after [Franklin, 2000]
	int
	cn_PnPoly( Point P, vector<Point>& V)
	{
	    int    cn = 0;    // the crossing number counter
	    cout << "Evaluate point "<< P.x << ' '<< P.y<< ": ";
	    // loop through all edges of the polygon
	    for (size_t i=0; i<V.size(); i++) {    // edge from V[i] to V[i+1]
	       if (((V[i].y <= P.y) && (V[i+1].y > P.y))    // an upward crossing
	        || ((V[i].y > P.y) && (V[i+1].y <= P.y))) { // a downward crossing
	            // compute the actual edge-ray intersect x-coordinate
	            float vt = (float)(P.y - V[i].y) / (V[i+1].y - V[i].y);
	            if (P.x < V[i].x + vt * (V[i+1].x - V[i].x)) // P.x < intersect
	                ++cn;   // a valid crossing of y=P.y right of P.x
	        }
	    }
	    cout << int(cn&1)<<endl;
	    return (cn&1);    // 0 if even (out), and 1 if odd (in)

	}
	//===================================================================

	// wn_PnPoly(): winding number test for a point in a polygon
	//      Input:   P = a point,
	//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
	//      Return:  wn = the winding number (=0 only if P is outside V[])
	int
	wn_PnPoly( Point P, vector<Point>& V)
	{
	    int    wn = 0;    // the winding number counter

	    // loop through all edges of the polygon
	    cout << "Evaluate point "<< P.x << ' '<< P.y<< ": ";
	    for (size_t i=0; i<V.size(); i++) {   // edge from V[i] to V[i+1]
	        if (V[i].y <= P.y) {         // start y <= P.y
	            if (V[i+1].y > P.y)      // an upward crossing
	                if (isLeft( V[i], V[i+1], P) > 0)  // P left of edge
	                    ++wn;            // have a valid up intersect
	        }
	        else {                       // start y > P.y (no test needed)
	            if (V[i+1].y <= P.y)     // a downward crossing
	                if (isLeft( V[i], V[i+1], P) < 0)  // P right of edge
	                    --wn;            // have a valid down intersect
	        }
	    }
	    cout << wn<<endl;
	    return wn;
	}
	//===================================================================
};
