#ifndef POMDP_H
#define POMDP_H

#include "globals.h"
#include "belief.h"
#include "random_streams.h"
#include "history.h"
#include "lower_bound.h"
#include "policy.h"
#include "upper_bound.h"
#include "util/memorypool.h"
#include "util/seeds.h"
#include "util/util.h"
#include <mutex>

using namespace Globals;

class State : public MemoryObject {
public:
	int state_id;
	int scenario_id;
	double weight;

	State();
	State(int _state_id, double weight);
	virtual ~State();

	friend ostream& operator<<(ostream& os, const State& state);

	virtual string text() const;

	static double Weight(const vector<State*>& particles);

	State* operator()(int state_id, double weight) {
		this->state_id = state_id;
		this->weight = weight;
		return this;
	}
};

class StateIndexer {
public:
	virtual int NumStates() const = 0;
	//virtual int operator()(const State* state) const = 0;
	//virtual State* operator()(const int index) const = 0;
	virtual int GetIndex(const State* state) const = 0;
	virtual const State* GetState(int index) const = 0;
};

class StatePolicy {
public:
	virtual int GetAction(const State& state) const = 0;
};

class MMAPInferencer {
public:
	virtual const State* GetMMAP(const vector<State*>& particles) const = 0;
};

class DSPOMDP { // Deterministic Simulative model for POMDP
protected:
	ParticleLowerBound* particle_lower_bound_;
	ScenarioLowerBound* scenario_lower_bound_;
	BeliefLowerBound* belief_lower_bound_;
	ParticleUpperBound* particle_upper_bound_;
	ScenarioUpperBound* scenario_upper_bound_;
	BeliefUpperBound* belief_upper_bound_;

public:
	static mutex mem_lock; // TODO: Experimental

	DSPOMDP();

	virtual ~DSPOMDP();

	/** Implement this to get speedup for LookaheadUpperBound.*/
	virtual bool Step(State& state, double random_num, int action, double& reward) const;
	virtual bool Step(State& state, int action, double& reward, uint64_t& obs) const;
  virtual bool Step(State& state, double random_num, int action,
			double& reward, uint64_t& obs) const = 0;
	static void StepMany(const DSPOMDP& model, const vector<State*>& particles, int first, int last, RandomStreams& streams,
		int action, vector<double>& rewards, vector<uint64_t>& obss, vector<short>& terminals);
	void Step(const vector<State*>& particles, RandomStreams& streams, int action,
			vector<double>& rewards, vector<uint64_t>& obss, vector<short>& terminals,
			int num_threads = 1) const;
  virtual int NumActions() const = 0;
  virtual double ObsProb(uint64_t obs, const State& state, int action) const = 0;

  virtual State* CreateStartState(string type = "DEFAULT") const = 0;
  virtual Belief* InitialBelief(const State* start, string type = "DEFAULT") const = 0;

	virtual double GetMaxReward() const = 0;
  virtual inline double UpperBound(const State& particle) const {
		return particle_upper_bound_->Value(particle);
	}
	virtual inline double UpperBound(const vector<State*>& particles, RandomStreams& streams,
			History& history) const {
		return scenario_upper_bound_->Value(particles, streams, history);
	}
	virtual inline double UpperBound(const Belief* belief) const {
		return belief_upper_bound_->Value(belief);
	}
	virtual void InitializeParticleUpperBound(string name);
	virtual void InitializeScenarioUpperBound(string name, RandomStreams& streams);
	inline ParticleUpperBound* particle_upper_bound() { return particle_upper_bound_; }
	inline ScenarioUpperBound* scenario_upper_bound() { return scenario_upper_bound_; }
	inline BeliefUpperBound* belief_upper_bound() { return belief_upper_bound_; }

	/**
	 * Return (a, v), where a is an action with largest minimum reward when it is
	 * executed, and v is its minimum reward.
	 */
	virtual ValuedAction GetMinRewardAction() const = 0;
  virtual inline ValuedAction LowerBound(const vector<State*>& particles) const {
		return particle_lower_bound_->Value(particles);
	}
	virtual inline ValuedAction LowerBound(const vector<State*>& particles, RandomStreams& streams,
			History& history) const {
		return scenario_lower_bound_->Value(particles, streams, history);
	}
	virtual inline ValuedAction LowerBound(const Belief* belief) const {
		return belief_lower_bound_->Value(belief);
	}
	virtual void InitializeParticleLowerBound(string name);
	virtual void InitializeScenarioLowerBound(string name, RandomStreams& streams);
	inline ParticleLowerBound* particle_lower_bound() { return particle_lower_bound_; }
	inline ScenarioLowerBound* scenario_lower_bound() { return scenario_lower_bound_; }
	inline BeliefLowerBound* belief_lower_bound() { return belief_lower_bound_; }

  virtual void PrintState(const State& state, ostream& out = cout) const = 0;
  virtual void PrintObs(const State& state, uint64_t obs, ostream& out = cout) const = 0;
	virtual void PrintAction(int action, ostream& out = cout) const = 0;
	virtual void PrintBelief(const Belief& belief, ostream& out = cout) const = 0;

	/* Memory management */
	virtual State* Allocate(int state_id, double weight) const = 0;
	virtual State* Copy(const State* particle) const = 0;
	virtual void Free(State* particle) const = 0;

	vector<State*> Copy(const vector<State*>& particles) const;

	mutable int num_active_particles;
};

class BeliefMDP : public DSPOMDP {
public:
	BeliefMDP();
	virtual ~BeliefMDP();

	virtual void InitializeBeliefLowerBound(string name);
	virtual void InitializeBeliefUpperBound(string name);

	virtual Belief* Tau(const Belief* belief, int action, uint64_t obs) const = 0;
	virtual void Observe(const Belief* belief, int action, map<uint64_t, double>& obss) const = 0;
	virtual double StepReward(const Belief* belief, int action) const = 0;
	// virtual vector<State*, double> double TransitonProbability(State* cur, int action, State* next);
};

#endif
