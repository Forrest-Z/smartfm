#ifndef ROAD_SEMANTICS_TOPO_GRAPH_H
#define ROAD_SEMANTICS_TOPO_GRAPH_H

#include <vector>
#include <opencv/cv.h>
#include "../spline_fitting/spline_fitting.h"

using namespace std;

//http://alienryderflex.com/polygon/
//ATTENTION: this function is used for float type;
template <class T>
bool pointInPolygon(T p, std::vector<T> poly)
{
	int polySides = poly.size();
	int      i, j=polySides-1 ;
	bool  oddNodes = false;

	for (i=0; i<polySides; i++)
	{
		if (((poly[i].y< p.y && poly[j].y>=p.y) || (poly[j].y< p.y && poly[i].y>=p.y)) &&  (poly[i].x<=p.x || poly[j].x<=p.x))
		{
			if(poly[i].x+(p.y-poly[i].y)/(poly[j].y-poly[i].y)*(poly[j].x-poly[i].x)<p.x)
			{oddNodes=!oddNodes;}
		}
		j=i;
	}

	return oddNodes;
}


class topo_graph {
public:

	class node {
	public:
		int 	ID;
		CvPoint position;
	};

	class node_cluster
	{
		public:
		//record its containing node IDs;
		int 	ID;
		vector<int> nodeIDs;
		CvPoint2D32f cluster_center;
		//record its connecting edge IDs;
		vector<int> edgeIDs;
	};

	class edge {
	public:
		int 	ID;
		int head_nodeCluster, end_nodeCluster;
		vector<CvPoint> points;
		double edge_length;
		golfcar_semantics::spline_fitting* cubic_spline;
	};

	vector<node> nodes;
	vector<node_cluster> nodeClusters;
	vector<edge> edges;

public:
	//function template to find the position of certain ID in the vector;
	template < class T >
	int find_ID_position (vector < T> a, int ID)
	{
		for(size_t i=0; i<a.size(); i++)
		{
			if(a[i].ID == ID)
			{
				return (int)i;
				break;
			}
		}

		// make sure that it is always able to find the queried ID.
		return -1;
	}

	void clear()
	{
		nodes.clear();
		nodeClusters.clear();
		edges.clear();
	}
};  


#endif
