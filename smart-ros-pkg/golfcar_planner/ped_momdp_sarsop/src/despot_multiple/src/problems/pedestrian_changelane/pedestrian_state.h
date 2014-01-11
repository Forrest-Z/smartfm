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
	PedStruct(COORD a,int b,int c) {first=a;second=b;third=c;}
	COORD first; //pos
	int second;  //goal
	int third;   //id
};
class PedestrianState
{
	public:
		COORD RobPos;
		//vector<pair<COORD,int>> PedPoses;    //(coor,goal)
		//pair<COORD,int> PedPoses[ModelParams::N_PED];
		PedStruct PedPoses[ModelParams::N_PED_IN];
		COORD PedPos;
		int Vel;
		int Lane;
		int num;
		int id;


		PedestrianState() {}

		PedestrianState(int id);

		/* TODO: bug due to wrong mapping - UpperBound table*/
		operator int() const ;
};

#endif
