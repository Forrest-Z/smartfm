#include "pedestrian_state.h"
#include "param.h"
#include <iostream>
using namespace std;
PedestrianState::PedestrianState(int id) {

	cerr<<"inside cal id!"<<endl;
	/*
	int X_SIZE=ModelParams::XSIZE;
	int Y_SIZE=ModelParams::YSIZE;
	int N_GOAL=ModelParams::NGOAL;
	if(id == X_SIZE * Y_SIZE * Y_SIZE * 3 * N_GOAL) {
		Vel = -1;
		return;
	}

	RobPos.X = 1;

	int N = id;
	Goal = (N/(X_SIZE*Y_SIZE*Y_SIZE*3));
	N = N%(X_SIZE*Y_SIZE*Y_SIZE*3);

	Vel = N/(X_SIZE*Y_SIZE*Y_SIZE);
	N = N%(X_SIZE*Y_SIZE*Y_SIZE);

	RobPos.Y = N/(X_SIZE*Y_SIZE);
	N = N%(X_SIZE*Y_SIZE);

	PedPos.Y = N/X_SIZE;
	N = N%X_SIZE;

	PedPos.X = N;*/
}

/* TODO: bug due to wrong mapping - UpperBound table*/
PedestrianState::operator int() const {
	cerr<<"inside operator int!"<<endl;

	/*
	int X_SIZE=ModelParams::XSIZE;
	int Y_SIZE=ModelParams::YSIZE;
	int N_GOAL=ModelParams::NGOAL;
	if(Vel == -1) return X_SIZE * Y_SIZE * Y_SIZE * 3 * N_GOAL;

	int add_goal = (X_SIZE*Y_SIZE*Y_SIZE*3)*(Goal);
	int add_vel = (X_SIZE*Y_SIZE*Y_SIZE)*Vel;
	int add_Y1 = (X_SIZE*Y_SIZE)*RobPos.Y;
	int add_Y2 = X_SIZE*PedPos.Y;
	int add_X = PedPos.X;

	int N = add_X+add_Y2+add_Y1+add_vel+add_goal;

	return N;*/
}
