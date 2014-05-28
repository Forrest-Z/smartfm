#ifndef PED_STATE_H
#define PED_STATE_H
#include <vector>
#include <utility>
#include "coord.h"
#include "param.h"
#include "pomdp.h"
using namespace std;

struct PedStruct {
	PedStruct(){}
	PedStruct(COORD a, int b, int c) {
		pos = a;
		goal = b;
		id = c;
	}
	COORD pos; //pos
	int goal;  //goal
	int id;   //id
};

class Pedestrian
{
public:
	Pedestrian() {}
	Pedestrian(int _w,int _h,int _goal,int _id) {w=_w;h=_h;goal=_goal;id=_id;ts=0;}
	Pedestrian(int _w,int _h) {w=_w;h=_h;}

	int w,h,goal;
	int id;   //each pedestrian has a unique identity
	int ts;
	int last_update;
};

struct CarStruct {
	int pos;
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
