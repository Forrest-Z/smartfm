#ifndef PED_STATE_H
#define PED_STATE_H
#include "coord.h"
#include "param.h"
#include <vector>
#include <utility>
using namespace std;

struct PedStruct {
	PedStruct(){}
	PedStruct(Coord a, int b, int c) {
		pos = a;
		goal = b;
		id = c;
	}
	Coord pos; //pos
	int goal;  //goal
	int id;   //id
};

struct CarStruct {
	Coord pos;
	double vel;
	double dist_travelled;
};

class PomdpState : public State {
public:
	CarStruct car;
	int num;
	PedStruct peds[ModelParams::N_PED_IN];
	PomdpState() {}
};

#endif
