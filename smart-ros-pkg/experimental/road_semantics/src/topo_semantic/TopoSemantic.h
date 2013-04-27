#ifndef TOPO_SEMANTIC_H
#define TOPO_SEMANTIC_H

#include <ros/ros.h>
#include "../topo_extractor/TopoExtractor.h"
#include "semantic_datatypes.hpp"

using namespace std;

class topo_semantic {

public:

	class path_segment
	{
		public:
		int head_nodeID, tail_nodeID;
		size_t edgeID;
	};

	topo_semantic(topo_extractor& topo_extractor_object);
	void analyze_semantic();

	//the input of the whole semantic analyses;
	topo_extractor *topo_extractor_pt_;

	//the intermediate information in the semantic analyses;
	//A cycle is extracted as connected edges as a list, and all the cycles are stored in a vector;
	vector <vector <topo_semantic::path_segment> > extracted_cycles_;

	//output assembly for road semantics from topology;
	semantic_assembly road_topo_semantics_;

private:
	void place_analyze();
	void roundabout_analyze();

	void cycle_extraction();
	void recursive_path_search(vector <vector <path_segment> > &growing_paths);
	void path_growing_branching(vector <path_segment> &path_father, vector < vector <path_segment> > & path_sons);
	void path_check_remove(vector < vector <path_segment> > & growing_paths);
	bool cycle_duplicate(vector <path_segment> &cycle_under_check);
	void intersection_analyze();
	void link_analyze();
	void visualization();
	void visualize_growing(vector < vector <path_segment> > & growing_paths);
	void cycle_filter();
};  

#endif //TopoSemantic.h
