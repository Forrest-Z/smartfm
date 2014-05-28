#ifndef PED_POMDP_H
#define PED_POMDP_H

#include <cmath>
#include <utility>
#include "pomdp.h"
#include "globals.h"
#include "lower_bound.h"
#include "upper_bound.h"
#include "string.h"
#include "util/coord.h"
#include "param.h"
#include "state.h"
#include "WorldModel.h"
// #include "SFM.h"

int Y_SIZE;
int X_SIZE;
int N_GOAL;

/*
struct PathNode { // TODO
	vector<PathNode> children;
	int x,y;
};
*/

class PedPomdp : public DSPOMDP {
public:
	PedPomdp(WorldModel &);
	void UpdateVel(int& vel, int action, Random& random) const;
	void RobStep(int &robY,int &rob_vel, int action, Random& random) const;
	void PedStep(PomdpState& state, Random& random) const;
	bool Step(State& state_, double rNum, int action, double& reward, uint64_t& obs) const;
    State* CreateStartState(string type = "DEFAULT") const {
		return 0;	
	}
	double TransitionProbability(const PomdpState& curr, const PomdpState& next, int action) const;

	uint64_t Observe(const State& ) const;
	vector<int> ObserveVector(const State& )   const;
	double ObsProb(uint64_t z, const State& s, int action) const;

	int NumStates() const;
	inline int NumActions() const { return 3; }

	PomdpState RandomState(unsigned& seed, PomdpState obs_state) const;
	PomdpState* GreateStartState(string type) const;

	void EnumerateBelief(vector<State*> &belief, State state,int depth) const;
	vector<vector<double>> GetBeliefVector(const vector<State*> particles) const;
	Belief* InitialBelief(const State* start, string type) const;

	ValuedAction GetMinRewardAction() const;
	void InitializeScenarioLowerBound(string name, RandomStreams& streams);

	double GetMaxReward() const;
	void InitializeParticleUpperBound(string name, RandomStreams& streams);
	void InitializeScenarioUpperBound(string name, RandomStreams& streams);

	void Statistics(const vector<PomdpState*> particles) const;
	void ModifyObsStates(const vector<PomdpState*> &particles,PomdpState&new_state,unsigned & seed) const;

	void PrintState(const State& state, ostream& out = cout) const;
	void PrintObs(const State & state, uint64_t obs, ostream& out = cout) const;
	void PrintAction(int action, ostream& out = cout) const;
	void PrintBelief(const Belief& belief, ostream& out = cout) const;

	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;

	vector<State*> ConstructParticles(vector<PomdpState> & samples); 
	static int action_vel[3];
	static const int CRASH_PENALTY;
	static const int GOAL_REWARD;

	WorldModel &world;
protected:
	double OBSTACLE_PROB;

	enum {
		ACT_CUR,
		ACT_ACC,
		ACT_DEC
	};

private:
	int** map;
	PomdpState startState;
	mutable MemoryPool<PomdpState> memory_pool_;
	mutable Random random_;

	double robotNoisyMove[3][3]; /*vel,move*/
	double robotMoveProbs[3][3];
	double robotVelUpdate[3][3][3]; /*action,vel,new vel*/
	double robotUpdateProb[3][3][3];

	int lookup(const double probs[], double prob) const {
		int pos = 0;
		double sum = probs[0];
		while(sum < prob) {
			pos ++;
			sum += probs[pos];
		}
		return pos;
	}

};

#endif
