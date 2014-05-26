#ifndef BELIEF_H
#define BELIEF_H

#include <vector>

#include "util/random.h"
#include "util/logging.h"
#include "history.h"

using namespace logging;

class State;
class StateIndexer;
class DSPOMDP;

struct BeliefUpdateConfig {
	bool use_whole_history;
};

class Belief {
//protected:
public:
	const DSPOMDP* model_;
	History history_;
	BeliefUpdateConfig update_config_;

public:
	Belief(const DSPOMDP* model);
	virtual ~Belief() {}

	void belief_update_config(BeliefUpdateConfig config);

	virtual vector<State*> Sample(int num) const = 0;
	virtual void Update(int action, uint64_t obs) = 0;

	virtual string text() const;
	friend ostream& operator<<(ostream& os, const Belief& belief);
	virtual Belief* MakeCopy() const = 0; // For custom memory management

	static vector<State*> Sample(int num, vector<State*> belief, const DSPOMDP* model);
	static vector<State*> Resample(int num, const vector<State*>& belief, const DSPOMDP* model, History history, int hstart = 0);
	static vector<State*> Resample(int num, const Belief& belief, History history, int hstart = 0);
	static vector<State*> Resample(int num, const DSPOMDP* model, const StateIndexer* indexer, int action, uint64_t obs);
};

class ParticleBelief : public Belief {
protected:
	vector<State*> particles_;
	int num_particles_;
	Belief* prior_;
	vector<State*> initial_particles_;
	const StateIndexer* state_indexer_;

public:
	ParticleBelief(vector<State*> particles, const DSPOMDP* model, Belief* prior = NULL);

	virtual ~ParticleBelief();
	void state_indexer(const StateIndexer* indexer);

	virtual const vector<State*>& particles() const;
	virtual vector<State*> Sample(int num) const;

	virtual void Update(int action, uint64_t obs);

	virtual Belief* MakeCopy() const;

	virtual string text() const;
};

#endif
