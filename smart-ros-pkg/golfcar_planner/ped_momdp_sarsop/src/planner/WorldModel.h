#include"ped_pomdp.h"
#include"Path.h"

class WorldModel  {
public:

    WorldModel();

	double inCollision(PomdpState state, int action);
	double isGoal(PomdpState state);
	void PedStep(PedStruct &ped, Random& random);
	double RobStep(CarStruct &car, int action, Random& random); 

	Path path;
    std::vector<COORD> goals;
    double freq;
};

struct PedBelief {
    COORD pos;
    std::vector<double> prob_goals;
    int sample_goal();
};

class WorldStateTracker {
public:

    void updatePed(Pedestrian& ped);
    void updateCar(COORD& car);
    void updateVel(double vel);

    bool emergency();

    PomdpState getPomdpState();

    double timestamp;
    CarStruct car;

    std::vector<Pedestrian> ped_list;

};

class WorldBeliefTracker {
public:
    WorldBeliefTracker(WorldModel& _model): model(_model) {}

    void update(const PomdpState& s);

    WorldModel& model;
    CarStruct car;
    std::map<int, PedBelief> peds;
};

