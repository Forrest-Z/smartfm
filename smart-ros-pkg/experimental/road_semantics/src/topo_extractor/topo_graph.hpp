#ifndef ROAD_SEMANTICS_TOPO_GRAPH_H
#define ROAD_SEMANTICS_TOPO_GRAPH_H

#include <vector>
#include <opencv/cv.h>

using namespace std;

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
