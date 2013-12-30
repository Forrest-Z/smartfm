#ifndef PED_STATE_H
#define PED_STATE_H
#include "param.h"
#include "coord.h"
#include "simulator.h"

struct  PedStruct 
{
	PedStruct(){}
	PedStruct(COORD a,int b,int c) {first=a;second=b;third=c;}
	COORD first; //pos
	int second;  //goal
	int third;   //id
};
class PedestrianState:public STATE
{
public:
    COORD RobPos;
    COORD PedPos;
    //vector<pair<COORD,int> > PedPoses;    //(coor,goal)
    PedStruct PedPoses[ModelParams::N_PED_IN];
    int   Vel;
    int   Goal;
    int num;
    int Lane;
};

#endif
