#include "TopoSemantic.h"

topo_semantic::topo_semantic(topo_extractor& topo_extractor_object)
{
	topo_extractor_pt_ = new topo_extractor(topo_extractor_object);
}

void topo_semantic::analyze_semantic()
{
	//analyse semantics from "nodeClusters", to find "places" like intersections and roundabout;
	place_analyze();
	link_analyze();
}

void place_analyze()
{
	roundabout_analyze();
	intersection_analyze();
}

//to recognize roundabout from road network;
void roundabout_analyze()
{
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	//// 1st step: loop through all of the node-clusters, to find cycles;
	///////////////////////////////////////////////////////////////////////////////////////////////////////


	//2nd step: perform classification for the cycles, based on the network features;

	//3rd step: determine the area of extracted roundabouts;
}

void cycle_extraction()
{
	vector<topo_graph::node_cluster> &nodeClusters_input = topo_extractor_pt_->road_graph_.nodeClusters;

	//to store the nodeCluster that has been processed, to avoid some unnecessary duplicate searchings;
	vector <size_t> proc_nodeIDs;
	//A cycle is extracted as connected edges as a list, and all the cycles are stored in a vector;
	vector <list <topo_semantic::path_segment> > extracted_cycles;

	topo_graph::node_cluster seedNode = &nodeClusters_input[0];
	//initialize paths growing from the seed nodeCluster;
	vector < list <topo_semantic::path_segment> > growing_paths;

	for(size_t ie=0; ie<seedNode.edgeIDs.size();ie++)
	{
		list <topo_semantic::path_segment> path_tmp;
		topo_semantic::path_segment segment_tmp;
		size_t edgeID_tmp = seedNode.edgeIDs[ie];
		topo_graph::edge edge_segment = topo_extractor_pt_->road_graph_.edges[edgeID_tmp];
		segment_tmp.edgeID = edgeID_tmp;

		if(edge_segment.head_nodeCluster == in)
		{
			segment_tmp.head_nodeID = edge_segment.head_nodeCluster;
			segment_tmp.tail_nodeID = edge_segment.end_nodeCluster;
		}
		else if(edge_segment.end_nodeCluster == in)
		{
			segment_tmp.head_nodeID = edge_segment.end_nodeCluster;
			segment_tmp.tail_nodeID = edge_segment.head_nodeCluster;
		}
		else
		{
			ROS_ERROR("BUG: edge connection error!");
			return;
		}
		path_tmp.push_back(segment_tmp);
		growing_paths.push_back(path_tmp);
	}
}

void recursive_path_search(vector <list <topo_semantic::path_segment> > &growing_paths, vector <size_t> &proc_nodeIDs, vector <list <topo_graph::edge> > &extracted_cycles)
{
	//1st: check all the paths: extract cycle path, and remove cycle path, duplicate path (whose tail_nodeID has been a proc_nodeID), and stopping path from the growing_paths;
	if(growing_paths.size()==0)return;
	else recursive_path_search();
}

void path_growing_branching(list <topo_semantic::path_segment> &path_father, vector < list <topo_semantic::path_segment> > & path_sons)
{
	topo_graph::node_cluster seedNode = path_father.back().tail_nodeID;
	for(size_t ie=0; ie<seedNode.edgeIDs.size();ie++)
	{
		list <topo_semantic::path_segment> path_son = path_father;
		topo_semantic::path_segment segment_tmp;
		size_t edgeID_tmp = seedNode.edgeIDs[ie];
		topo_graph::edge edge_segment = topo_extractor_pt_->road_graph_.edges[edgeID_tmp];
		segment_tmp.edgeID = edgeID_tmp;

		if(edge_segment.head_nodeCluster == in)
		{
			segment_tmp.head_nodeID = edge_segment.head_nodeCluster;
			segment_tmp.tail_nodeID = edge_segment.end_nodeCluster;
		}
		else if(edge_segment.end_nodeCluster == in)
		{
			segment_tmp.head_nodeID = edge_segment.end_nodeCluster;
			segment_tmp.tail_nodeID = edge_segment.head_nodeCluster;
		}
		else
		{
			ROS_ERROR("BUG: edge connection error!");
			return;
		}
		path_son.push_back(segment_tmp);
		path_sons.push_back(path_son);
	}
}

void path_check_trim_remove(vector < list <topo_semantic::path_segment> > & paths)
{

}




void intersection_analyze()
{

}

void link_analyze()
{

}



