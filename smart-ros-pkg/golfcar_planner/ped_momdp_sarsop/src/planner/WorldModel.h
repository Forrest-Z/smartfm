#include"Path.h"

class WorldModel  {

    WorldModel();

	double inCollision(PomdpState state, int action);
	double isGoal(PomdpState state);
	void PedStep(PedStruct &ped, UtilUniform &unif);
	double RobStep(CarStruct &car, int action, UtilUniform &unif); 

	Path path;
    std:vector<COORD> goals;
    double freq;
}

class WorldStateTracker {
    
    void updatePed(Pedestrian& ped);
    void updateCar(COORD& car);
    void updateVel(double vel);

    PomdpState getPomdpState();

    double timestamp;
    CarStruct car;
    
    std:vector<Pedestrian> ped_list;


}
