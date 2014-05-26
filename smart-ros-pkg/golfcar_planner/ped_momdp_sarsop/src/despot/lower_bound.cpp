#include "lower_bound.h"
#include "pomdp.h"

ValuedAction::ValuedAction() : action(-1), value(0) {}

ValuedAction::ValuedAction(int _action, double _value) :
	action(_action), value(_value) {}

ostream& operator<<(ostream& os, const ValuedAction& va) {
	os << "(" << va.action << ", " << va.value << ")";
	return os;
}

/*---------------------------------------------------------------------------*/

ParticleLowerBound::ParticleLowerBound(const DSPOMDP* model) :
	model_(model) {
}

/*---------------------------------------------------------------------------*/

TrivialParticleLowerBound::TrivialParticleLowerBound(const DSPOMDP* model) :
	ParticleLowerBound(model) {
}

ValuedAction TrivialParticleLowerBound::Value(const vector<State*>& particles) const {
	ValuedAction va = model_->GetMinRewardAction();
	va.value *= State::Weight(particles) / (1 - Discount());
	return va;
}

/*---------------------------------------------------------------------------*/

ScenarioLowerBound::ScenarioLowerBound(const DSPOMDP* model, Belief* belief) :
	Solver(model, belief)
{
}

void ScenarioLowerBound::Reset() {
}

int ScenarioLowerBound::Search() {
	// cout << *belief_ << endl;
	RandomStreams streams(Seeds::Next(Globals::config.n_particles), Globals::config.search_depth); // TODO: use fixed streams?
	vector<State*> particles = belief_->Sample(Globals::config.n_particles);

	ValuedAction va = Value(particles, streams, history_);

	for (State* particle : particles)
		model_->Free(particle);

	return va.action;
}

void ScenarioLowerBound::Learn(VNode* tree) {
}

/*---------------------------------------------------------------------------*/

TrivialScenarioLowerBound::TrivialScenarioLowerBound(const DSPOMDP* model, Belief* belief) :
	ScenarioLowerBound(model, belief)
{
}

ValuedAction TrivialScenarioLowerBound::Value(const vector<State*>& particles,
		RandomStreams& streams, History& history) const  {
	return model_->LowerBound(particles);
}

/*---------------------------------------------------------------------------*/

BeliefLowerBound::BeliefLowerBound(const DSPOMDP* model, Belief* belief) :
	Solver(model, belief)
{
}

int BeliefLowerBound::Search() {
	// cout << *belief_ << endl;
	ValuedAction va = Value(belief_);
	return va.action;
}

void BeliefLowerBound::Learn(VNode* tree) {
}

/*---------------------------------------------------------------------------*/

TrivialBeliefLowerBound::TrivialBeliefLowerBound(const DSPOMDP* model, Belief* belief) :
	BeliefLowerBound(model, belief)
{
}

ValuedAction TrivialBeliefLowerBound::Value(const Belief* belief) const {
	ValuedAction va = model_->GetMinRewardAction();
	va.value *= 1.0 / (1 - Discount());
	return va;
}
