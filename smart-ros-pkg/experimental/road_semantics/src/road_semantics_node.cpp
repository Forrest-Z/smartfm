#include "road_semantics.h"

int main(int argc, char** argv)
{
	string para_file = "./launch/parameters.yaml";
	golfcar_semantics::road_semantics road_semantics_node(para_file);
	road_semantics_node.network_semantics();
	return 0;
}
