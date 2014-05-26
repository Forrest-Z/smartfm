#ifndef LOWER_BOUND_H
#define LOWER_BOUND_H

#include <vector>
#include "random_streams.h"
#include "history.h"
#include "solver.h"

class State;
class DSPOMDP;
class VNode;

struct ValuedAction {
	int action;
	double value;

	ValuedAction();
	ValuedAction(int _action, double _value);

	friend ostream& operator<<(ostream& os, const ValuedAction& va);
};

/** Interface for an algorithm used to compute a lower bound for the infinite
 * horizon reward that can be obtained by the optimal policy on any set of
 * weighted particles..
 */
class ParticleLowerBound {
protected:
	const DSPOMDP* model_;

	ParticleLowerBound(const DSPOMDP* model);

public:
	/** Returns a lower bound to the maximum total discounted reward over an
	 * infinite horizon for the particles.
	 */
	virtual ValuedAction Value(const vector<State*>& particles) const = 0;
};

class TrivialParticleLowerBound : public ParticleLowerBound {
public:
	TrivialParticleLowerBound(const DSPOMDP* model);

public:
	virtual ValuedAction Value(const vector<State*>& particles) const;
};

/** Interface for an algorithm used to compute a lower bound for the infinite
 * horizon reward that can be obtained by the optimal policy on any set of
 * weighted scenarios.
 */
class ScenarioLowerBound : public Solver {
public:
	ScenarioLowerBound(const DSPOMDP* model, Belief* belief = NULL);

	virtual int Search();
	virtual void Learn(VNode* tree);
	virtual void Reset();

	virtual ValuedAction Value(const vector<State*>& particles,
			RandomStreams& streams, History& history) const = 0;
};

class TrivialScenarioLowerBound : public ScenarioLowerBound {
public:
	TrivialScenarioLowerBound(const DSPOMDP* model, Belief* belief = NULL);

	ValuedAction Value(const vector<State*>& particles,
			RandomStreams& streams, History& history) const;
};

/** Interface for an algorithm used to compute a lower bound for the infinite
 * horizon reward that can be obtained by the optimal policy on a belief.
 */
class BeliefLowerBound : public Solver {
public:
	BeliefLowerBound(const DSPOMDP* model, Belief* belief = NULL);

	virtual int Search();
	virtual void Learn(VNode* tree);

	virtual ValuedAction Value(const Belief* belief) const = 0;
};

class TrivialBeliefLowerBound : public BeliefLowerBound {
public:
	TrivialBeliefLowerBound(const DSPOMDP* model, Belief* belief = NULL);

	virtual ValuedAction Value(const Belief* belief) const;
};

#endif
