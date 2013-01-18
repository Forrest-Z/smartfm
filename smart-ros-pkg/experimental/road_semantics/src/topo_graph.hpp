#ifndef ROAD_SEMANTICS_TOPO_GRAPH
#define ROAD_SEMANTICS_TOPO_GRAPH

#include <vector>
#include <opencv/cv.h>

using namespace std;

class topo_graph {
public:
	class node_cluster
	{
		public:
		//record its containing node IDs;
		vector<size_t> nodeIDs;
		CvPoint2D32f cluster_center;
		//record its connecting edge IDs;
		vector<size_t> edgeIDs;
	};

	class edge {
	public:
		vector<CvPoint> points;
		int head_nodeCluster, end_nodeCluster;
	};

	vector<CvPoint> nodes;
	vector<node_cluster> nodeClusters;
	vector<edge> edges;
};  


#endif
