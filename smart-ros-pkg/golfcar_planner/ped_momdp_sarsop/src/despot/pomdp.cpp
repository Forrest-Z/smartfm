#include "pomdp.h"
#include "policy.h"
#include "lower_bound.h"
#include "upper_bound.h"
#include <thread>
#include <mutex>

ostream& operator<<(ostream& os, const State& state) {
	os << "(state_id = " << state.state_id << ", weight = " << state.weight << ", text = " << (&state)->text() << ")";
	return os;
}

State::State() : state_id(-1) {}

State::State(int _state_id, double _weight) :
	state_id(_state_id),
	weight(_weight) {
}

State::~State() {
}

string State::text() const {
	return "AbstractState";
}

double State::Weight(const vector<State*>& particles) {
	double weight = 0;
	for(auto& particle : particles)
		weight += particle->weight;
	return weight;
}

/*---------------------------------------------------------------------------*/
mutex DSPOMDP::mem_lock;

DSPOMDP::DSPOMDP()
		: particle_lower_bound_(new TrivialParticleLowerBound(this)),
		scenario_lower_bound_(new TrivialScenarioLowerBound(this)),
		belief_lower_bound_(new TrivialBeliefLowerBound(this)),
		particle_upper_bound_(new TrivialParticleUpperBound(this)),
		scenario_upper_bound_(new TrivialScenarioUpperBound(this)),
		belief_upper_bound_(new TrivialBeliefUpperBound(this))
{}

DSPOMDP::~DSPOMDP() {}

bool DSPOMDP::Step(State& state, int action, double& reward, uint64_t& obs) const {
	return Step(state, Random::RANDOM.NextDouble(), action, reward, obs);
}

bool DSPOMDP::Step(State& state, double random_num, int action, double& reward) const {
	uint64_t obs;
	return Step(state, random_num, action, reward, obs);
}

void DSPOMDP::StepMany(const DSPOMDP& model, const vector<State*>& particles, int first, int last, RandomStreams& streams,
		int action, vector<double>& rewards, vector<uint64_t>& obss, vector<short>& terminals) {
	bool terminal;
	double reward;
	uint64_t obs;
	for (int i=first; i<=last; i++) {
		terminal = model.Step(*(particles[i]), streams.Entry(particles[i]->scenario_id), action, reward, obs);
		rewards[i] = reward;
		obss[i] = obs;
		terminals[i] = terminal;
	}
}

void DSPOMDP::Step(const vector<State*>& particles, RandomStreams& streams,
		int action, vector<double>& rewards, vector<uint64_t>& obss, vector<short>& terminals, int num_threads) const {
	if (rewards.size() < particles.size())
		rewards.resize(particles.size());
	if (obss.size() < particles.size())
		obss.resize(particles.size());
	if (terminals.size() < particles.size())
		terminals.resize(particles.size());

	vector<thread> threads;

	for (int t=0; t<num_threads; t++) {
		int first = particles.size() * 1.0 * t / num_threads,
				last = particles.size() * 1.0 * (t + 1) /num_threads - 1;
		threads.push_back(thread(DSPOMDP::StepMany, ref(*this), ref(particles), first, last, ref(streams), action, ref(rewards), ref(obss), ref(terminals)));
	}

	for (int t=0; t<num_threads; t++)
		threads[t].join();
}

void DSPOMDP::InitializeParticleUpperBound(string name) {
	if (name == "TRIVIAL") {
	} else {
		cerr << "----- Unsupported particle upper bound: " << name << " -----" << endl
			<< "Supported type: TRIVIAL" << endl
			<< "Default to TRIVIAL" << endl;
	}
}

void DSPOMDP::InitializeScenarioUpperBound(string name, RandomStreams& streams) {
	if(name == "TRIVIAL") {
		scenario_upper_bound_ = new TrivialScenarioUpperBound(this);
		logi("Upper bound algorithm initialized to TRIVIAL");
	} else {
		cerr << "----- Unsupported upper bound algorithm: " << name << " -----" << endl
			<< "Supported type: TRIVIAL" << endl
			<< "Default to TRIVIAL" << endl;
	}
}

void DSPOMDP::InitializeParticleLowerBound(string name) {
	if (name == "TRIVIAL") {
	} else {
		cerr << "----- Unsupported particle lower bound: " << name << " -----" << endl
			<< "Supported type: TRIVIAL" << endl
			<< "Default to TRIVIAL" << endl;
	}
}

void DSPOMDP::InitializeScenarioLowerBound(string name, RandomStreams& streams) {
	if(name == "TRIVIAL") {
		scenario_lower_bound_ = new TrivialScenarioLowerBound(this);
	} else if(name == "RANDOM") {
		scenario_lower_bound_ = new RandomPolicy(this);
	} else {
		cerr << "----- Unsupported lower bound algorithm: " << name << " -----" << endl
			<< "Supported types: TRIVIAL, RANDOM" << endl
			<< "Default to TRIVIAL" << endl;
	}
}


vector<State*> DSPOMDP::Copy(const vector<State*>& particles) const {
	vector<State*> copy;
	for(auto& particle : particles)
		copy.push_back(Copy(particle));
	return copy;
}

BeliefMDP::BeliefMDP() {}
BeliefMDP::~BeliefMDP() {}

void BeliefMDP::InitializeBeliefLowerBound(string name) {}

void BeliefMDP::InitializeBeliefUpperBound(string name) {
	if (name == "TRIVIAL") {
	} else {
		cerr << "----- Unsupported belief upper bound: " << name << " -----" << endl
			<< "Supported type: TRIVIAL" << endl
			<< "Default to TRIVIAL" << endl;
	}
}


