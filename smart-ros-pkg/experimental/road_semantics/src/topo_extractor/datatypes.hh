#ifndef ROAD_SEMANTICS_DATATYPES
#define ROAD_SEMANTICS_DATATYPES

#include <vector>

using namespace std;

// Each cell belongs to one of three states.
typedef enum {Occupied, Unknown, Free} cell_type;
typedef vector<cell_type> column_type;
typedef vector<column_type> grid_type;



#endif
