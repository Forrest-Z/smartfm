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

	double inCollision(const PomdpState& state, int action);
	int defaultPolicy(const vector<State*>& particles);
    bool isLocalGoal(const PomdpState& state);
    bool isGlobalGoal(const CarStruct& car);
	int aligned(COORD ped_pos, int car);
    double minStepToGoal(const PomdpState& state);

	void PedStep(PedStruct &ped, Random& random);
	void RobStep(CarStruct &car, Random& random);
    void RobVelStep(CarStruct &car, double acc, Random& random);

    double pedMoveProb(COORD p0, COORD p1, int goal_id);
    void setPath(Path path);
    void updatePedBelief(PedBelief& b, const PedStruct& curr_ped);
    PedBelief initPedBelief(const PedStruct& ped);


	Path path;
    std::vector<COORD> goals;
    double freq;
};

class WorldStateTracker {
public:
    typedef pair<float, Pedestrian> PedDistPair;

    WorldStateTracker(WorldModel& _model): model(_model) {}

    void updatePed(const Pedestrian& ped);
    void updateCar(const COORD& car);
    void updateVel(double vel);
    void cleanPed();

    bool emergency();

    std::vector<PedDistPair> getSortedPeds();

    PomdpState getPomdpState();

    COORD carpos;
    double carvel;

    std::vector<Pedestrian> ped_list;

    WorldModel& model;
};

class WorldBeliefTracker {
public:
    WorldBeliefTracker(WorldModel& _model, WorldStateTracker& _stateTracker): model(_model), stateTracker(_stateTracker) {}

    void update();
    PomdpState sample();
    vector<PomdpState> sample(int num);

    WorldModel& model;
    WorldStateTracker& stateTracker;
    CarStruct car;
    std::map<int, PedBelief> peds;
};

