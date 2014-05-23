#ifndef PED_STATE_H
#define PED_STATE_H
#include "coord.h"
#include "param.h"
#include <vector>
#include <utility>
using namespace std;

struct  PedStruct 
{
	PedStruct(){}
	PedStruct(COORD a,int b,int c) {pos=a;goal=b;id=c;}
	COORD pos; //pos
	int goal;  //goal
	int id;   //id
};
struct CarStruct
{
	COORD pos;
	double vel;
	double dist_travelled;
};
class PomdpState 
{
	public:
		CarStruct car;
		int num;
		PedStruct peds[ModelParams::N_PED_IN];
		PedestrianState() {}
};

#endif
