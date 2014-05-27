#pragma once
#include"state.h"
#include"Path.h"

struct PedBelief {
    COORD pos;
    std::vector<double> prob_goals;
    int sample_goal();
};

class WorldModel  {
public:

    WorldModel();

	double inCollision(PomdpState state, int action);
    bool isLocalGoal(PomdpState state);
    bool isGlobalGoal(CarStruct car);
    double minStepToGoal(PomdpState state);

	void PedStep(PedStruct &ped, Random& random);
	void RobStep(CarStruct &car, Random& random);
    void RobVelStep(CarStruct &car, double acc, Random& random);

    double pedMoveProb(COORD p0, COORD p1, int goal_id);
    void setPath(Path path);
    void updatePedBelief(PedBelief& b, const PedStruct& curr_ped);
    PedBelief& initPedBelief(const PedStruct& ped);


	Path path;
    std::vector<COORD> goals;
    double freq;
};

class WorldStateTracker {
public:

    void updatePed(Pedestrian& ped);
    void updateCar(COORD& car);
    void updateVel(double vel);
    void cleanPed();

    bool emergency();

    PomdpState getPomdpState();

    CarStruct car;

    std::vector<Pedestrian> ped_list;

};

class WorldBeliefTracker {
public:
    WorldBeliefTracker(WorldModel& _model): model(_model) {}

    void update(const PomdpState& s);
    PomdpState sample();
    vector<PomdpState> sample(int num);

    WorldModel& model;
    CarStruct car;
    std::map<int, PedBelief> peds;
};

