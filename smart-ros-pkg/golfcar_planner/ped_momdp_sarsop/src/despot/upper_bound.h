#ifndef UPPER_BOUND_H
#define UPPER_BOUND_H

#include <vector>
#include <cassert>

#include "random_streams.h"
#include "history.h"

class State;
class StateIndexer;
class DSPOMDP;
class Belief;
class MDP;
class ValuedAction;

/*---------------------------------------------------------------------------*/

class ParticleUpperBound {
protected:
	const DSPOMDP* model_;

public:
	ParticleUpperBound(const DSPOMDP* model);

	/**
	 * Returns an upper bound to the maximum total discounted reward over an
	 * infinite horizon for the (unweighted) particle.
	 */
	virtual double Value(const State& state) const = 0;
};

/*---------------------------------------------------------------------------*/

class TrivialParticleUpperBound : public ParticleUpperBound {
public:
	TrivialParticleUpperBound(const DSPOMDP* model);

	double Value(const State& state) const;
};

/*---------------------------------------------------------------------------*/

class MDPParticleUpperBound : public ParticleUpperBound {
protected:
	const MDP* model_;
	const StateIndexer& indexer_;
	vector<ValuedAction> policy_;

public:
	MDPParticleUpperBound(const MDP* model, const StateIndexer& indexer);

	double Value(const State& state) const;
};

/*---------------------------------------------------------------------------*/

class ScenarioUpperBound {
protected:
	const DSPOMDP* model_;

public:
	ScenarioUpperBound(const DSPOMDP* model);

	virtual double Value(const vector<State*>& particles,
			RandomStreams& streams, History& history) const = 0;
};

class TrivialScenarioUpperBound : public ScenarioUpperBound {
public:
	TrivialScenarioUpperBound(const DSPOMDP* model);

	double Value(const vector<State*>& particles,
			RandomStreams& streams, History& history) const;
};

/*---------------------------------------------------------------------------*/

class BeliefUpperBound {
protected:
	const DSPOMDP* model_;

public:
	BeliefUpperBound(const DSPOMDP* model);

	virtual double Value(const Belief* belief) const = 0;
};

class TrivialBeliefUpperBound : public BeliefUpperBound {
public:
	TrivialBeliefUpperBound(const DSPOMDP* model);

	double Value(const Belief* belief) const;
};

class LookaheadUpperBound : public ScenarioUpperBound {
private:
	const StateIndexer& indexer_;
	RandomStreams& streams_;
	vector<vector<vector<double>>> bounds_;

public:
	LookaheadUpperBound(const DSPOMDP* model, const StateIndexer& indexer, RandomStreams& streams);

	double Value(const vector<State*>& particles,
			RandomStreams& streams, History& history) const;
};
#endif
