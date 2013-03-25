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

void topo_semantic::place_analyze()
{
	roundabout_analyze();
	intersection_analyze();
}

//to recognize roundabout from road network;
void topo_semantic::roundabout_analyze()
{
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	//// 1st step: loop through all of the node-clusters, to find cycles;
	///////////////////////////////////////////////////////////////////////////////////////////////////////


	//2nd step: perform classification for the cycles, based on the network features;

	//3rd step: determine the area of extracted roundabouts;
}

void topo_semantic::cycle_extraction()
{
	//to store the growing paths;
	vector < vector <path_segment> > growing_paths;

	topo_semantic::path_segment virtual_origin_path;
	virtual_origin_path.head_nodeID = -1;
	virtual_origin_path.tail_nodeID = 0;
	path_growing_branching(virtual_origin_path, growing_paths);
	for(size_t i=0; i<growing_paths.size(); i++)
	{
		//remove the virtual original path;
		growing_paths[i].pop_front();
	}
	recursive_path_search(growing_paths, proc_nodeIDs);
}

void topo_semantic::recursive_path_search(vector <vector <path_segment> > &growing_paths)
{
	//1st: check all the growing paths:
	//extract cycle paths; remove cycle paths and stopping paths from the growing_paths;
	path_check_remove(growing_paths);

	//2nd: recursively perform path search until all the growing paths are removed from the vector;
	if(growing_paths.size()==0)return;
	else
	{
		vector < vector <path_segment> > growing_paths_new;
		for(size_t i=0; i<growing_paths.size(); i++)
		{
			path_growing_branching(growing_paths[i], growing_paths_new);
		}
		growing_paths = growing_paths_new;
		recursive_path_search(growing_paths);
	}
}

void topo_semantic::path_growing_branching(vector <path_segment> &path_father, vector < vector <path_segment> > & path_sons)
{
	int tail_nodeID = path_father.back().tail_nodeID;
	topo_graph::node_cluster seedNode = topo_extractor_pt_->road_graph_.nodeClusters[tail_nodeID];
	for(size_t ie=0; ie<seedNode.edgeIDs.size();ie++)
	{
		vector <path_segment> path_son = path_father;
		path_segment segment_tmp;
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

void topo_semantic::path_check_remove(vector < vector <path_segment> > & growing_paths)
{
	for(size_t i=0; i<growing_paths.size(); i++)
	{
		int path_tail_nodeID = growing_paths[i].back().tail_nodeID;
		if(path_tail_nodeID==-1)
		{
			growing_paths.erase(growing_paths.begin()+i);
			i=i-1;
		}
		else
		{
			for(size_t j=0; j<growing_paths[i].size(); j++)
			{
				int seg_tail_nodeID = growing_paths[i][j].tail_nodeID;
				if(seg_tail_nodeID == path_tail_nodeID)
				{
					//only keep the loop part, by erasing the irrelevant head part;
					growing_paths[i].erase(growing_paths[i].begin(), growing_paths[i].begin()+j);

					if(!cycle_duplicate(growing_paths[i]))extracted_cycles_.push_back(growing_paths[i]);
					growing_paths.erase(growing_paths.begin()+i);
					break;
				}
			}
		}
	}
}

bool topo_semantic::cycle_duplicate(vector <path_segment> &cycle_under_check)
{
	for(size_t i=0; i<extracted_cycles_.size(); i++)
	{
		vector <path_segment> cycle_component = extracted_cycles_[i];

		bool cycle_duplicate = true;
		//check every segment to find possible exactly same cycle;
		if(cycle_component.size()==cycle_under_check.size())
		{
			for(size_t j=0; j<cycle_under_check.size(); j++)
			{
				path_segment segment_under_check = cycle_under_check[j];
				bool segment_equal = false;
				for(size_t a=0; a<cycle_component.size(); a++)
				{
					if(cycle_component[a].edgeID == segment_under_check.edgeID)
					{
						segment_equal = true;
						break;
					}
				}
				if(!segment_equal) cycle_duplicate = false;
			}
		}
		else
		{
			cycle_duplicate = false;
		}

		if(cycle_duplicate)
		{
			return true;
		}
	}
}


void topo_semantic::intersection_analyze()
{

}

void topo_semantic::link_analyze()
{

}



