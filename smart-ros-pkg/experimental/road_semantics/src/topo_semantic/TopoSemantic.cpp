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
	//1st step: loop through all of the node-clusters, to find cycles;

	//2nd step: perform classification for the cycles, based on the network features;

	//3rd step: determine the area of extracted roundabouts;
}

void intersection_analyze()
{

}

void link_analyze()
{

}



