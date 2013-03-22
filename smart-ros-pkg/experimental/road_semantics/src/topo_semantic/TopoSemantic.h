#ifndef TOPO_SEMANTIC_H
#define TOPO_SEMANTIC_H

#include "../topo_extractor/TopoExtractor.h"
#include "semantic_datatypes.hpp"

using namespace std;

class topo_semantic {

public:
	topo_semantic(topo_extractor& topo_extractor_object);
	void analyze_semantic();

	topo_extractor *topo_extractor_pt_;
	semantic_assembly road_topo_semantics_;

private:
	void place_analyze();
	void roundabout_analyze();
	void intersection_analyze();

	void link_analyze();

	void visualization();
};  

#endif //TopoSemantic.h
