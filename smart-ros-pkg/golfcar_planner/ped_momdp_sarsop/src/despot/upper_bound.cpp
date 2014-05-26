#include "upper_bound.h"
#include "pomdp.h"
#include "mdp.h"

ParticleUpperBound::ParticleUpperBound(const DSPOMDP* model) 
	: model_(model) {
}

/*---------------------------------------------------------------------------*/

TrivialParticleUpperBound::TrivialParticleUpperBound(const DSPOMDP* model)
	: ParticleUpperBound(model) {
}

double TrivialParticleUpperBound::Value(const State& state) const {
	return model_->GetMaxReward() / (1 - Discount());
}

/*---------------------------------------------------------------------------*/
MDPParticleUpperBound::MDPParticleUpperBound(const MDP* model, const StateIndexer& indexer) 
	: ParticleUpperBound(NULL),
	model_(model),
	indexer_(indexer) {
		const_cast<MDP*>(model_)->ComputeOptimalPolicyUsingVI();
		policy_ = model_->policy();
}

double MDPParticleUpperBound::Value(const State& state) const {
	return policy_[indexer_.GetIndex(&state)].value;
}

/*---------------------------------------------------------------------------*/

ScenarioUpperBound::ScenarioUpperBound(const DSPOMDP* model) :
	model_(model)
{
}

TrivialScenarioUpperBound::TrivialScenarioUpperBound(const DSPOMDP* model) : 
	ScenarioUpperBound(model)
{
}

double TrivialScenarioUpperBound::Value(const vector<State*>& particles,
			RandomStreams& streams, History& history) const {
	double value = 0;
	for(State* particle : particles) {
		value += particle->weight * model_->UpperBound(*particle);
	}
	return value;
}

/*---------------------------------------------------------------------------*/
BeliefUpperBound::BeliefUpperBound(const DSPOMDP* model) :
	model_(model)
{
}

TrivialBeliefUpperBound::TrivialBeliefUpperBound(const DSPOMDP* model) : 
	BeliefUpperBound(model)
{
}

double TrivialBeliefUpperBound::Value(const Belief* belief) const {
	return model_->GetMaxReward() / (1 - Discount());
}

LookaheadUpperBound::LookaheadUpperBound(const DSPOMDP* model, const StateIndexer& indexer, RandomStreams& streams) :
	ScenarioUpperBound(model),
	indexer_(indexer),
	streams_(streams)
{
	int num_states = indexer.NumStates();
	int length = streams.Length();
	int num_particles = streams.NumStreams();

	SetSize(bounds_, num_particles, length+1, num_states);

	clock_t start = clock();
	for(int p=0; p<num_particles; p++) {
		if (p % 10 == 0)
			cerr << p << " scenarios done! [" << (double (clock() - start) / CLOCKS_PER_SEC) << "s]" << endl;
		for(int t=length; t>=0; t--) {
			if(t==length) { // base case
				for(int s=0; s<num_states; s++) {
					bounds_[p][t][s] = model->UpperBound(*indexer_.GetState(s));
				}
			} else { // lookahead
				for(int s=0; s<num_states; s++) {
					double best = Globals::NEG_INFTY;

					for(int a=0; a<model->NumActions(); a++) {
						double reward = 0;
						State* copy = model->Copy(indexer_.GetState(s));
						bool terminal = model->Step(*copy, streams.Entry(p, t), a, reward);
						model->Free(copy);
						reward += (!terminal) * Discount() * bounds_[p][t+1][indexer_.GetIndex(copy)];
						//reward += (!terminal) * Discount() * bounds_[p][t+1][copy->state_id];

						if(reward > best)
							best = reward;
					}

					bounds_[p][t][s] = best;
				}
			}
		}
	}
}

double LookaheadUpperBound::Value(const vector<State*>& particles,
			RandomStreams& streams, History& history) const {
	double bound = 0;
	for(auto& particle : particles) {
		bound += particle->weight * bounds_[particle->scenario_id][streams.position()][indexer_.GetIndex(particle)];
		// bound += particle->weight * bounds_[particle->scenario_id][streams.position()][particle->state_id];
	}
	return bound;
}
