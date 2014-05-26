#include "belief.h"
#include "pomdp.h"

Belief::Belief(const DSPOMDP* model) : model_(model) {}

string Belief::text() const { return "AbstractBelief"; }

ostream& operator<<(ostream& os, const Belief& belief) {
	os << (&belief)->text();
	return os;
}

void Belief::belief_update_config(BeliefUpdateConfig config) {
	update_config_ = config;
}

vector<State*> Belief::Sample(int num, vector<State*> particles,
		const DSPOMDP* model) {
	double unit = 1.0/num;
	double mass = Random::RANDOM.NextDouble(0, unit);
	int pos = 0;
	double cur = particles[0]->weight;

	vector<State*> sample;
	for (int i = 0; i < num; i ++) {
		while(mass > cur) {
			pos ++;
			if (pos == particles.size())
				pos = 0;

			cur += particles[pos]->weight;
		}

		mass += unit;

		State* particle = model->Copy(particles[pos]);
		particle->weight = unit;
		sample.push_back(particle);
	}

	logi("Sampled ", sample.size(), " particles");
	for (int i = 0; i < sample.size(); i ++) {
		sample[i]->scenario_id = i;
		logi(" ", i, " = ", *sample[i]);
	}

	return sample;
}

vector<State*> Belief::Resample(int num, const vector<State*>& belief, const DSPOMDP* model, History history, int hstart) {
	double unit = 1.0/num;
	double mass = Random::RANDOM.NextDouble(0, unit);
	int pos = 0;
	double cur = belief[0]->weight;

	double reward;
	uint64_t obs;

	vector<State*> sample;
	int count = 0;
	double max_wgt = Globals::NEG_INFTY;
	int trial = 0;
	while (count < num && trial < 200 * num) {
		// Pick next particle
		while(mass > cur) {
			pos ++;
			if (pos == belief.size())
				pos = 0;

			cur += belief[pos]->weight;
		}
		trial ++;

		mass += unit;

		State* particle = model->Copy(belief[pos]);

		// Step through history
		double log_wgt = 0;
		for (int i = hstart; i < history.Size(); i ++) {
			model->Step(*particle, Random::RANDOM.NextDouble(), history.Action(i), reward, obs);

			double prob = model->ObsProb(history.Observation(i), *particle, history.Action(i));
			if (prob > 0) {
				log_wgt += log(prob);
			} else {
				model->Free(particle);
				break;
			}
		}

		// Add to sample if survived
		if (particle->IsAllocated()) {
			count ++;

			particle->weight = log_wgt;
			sample.push_back(particle);

			// cout << "Particle " << count << " " << trial << endl;
			// model->PrintState(*particle);

			max_wgt = max(log_wgt, max_wgt);
		}

		// Remove particles with very small weights
		if (count == num) {
			for (int i = sample.size()-1; i >= 0; i --)
				if (sample[i]->weight - max_wgt < log(1.0/num)) {
					model->Free(sample[i]);
					sample.erase(sample.begin() + i);
					count --;
				}
		}
	}

	double total_weight = 0;
	for (int i = 0; i < sample.size(); i ++) {
		sample[i]->weight = exp(sample[i]->weight - max_wgt);
		total_weight += sample[i]->weight;
	}
	for (int i = 0; i < sample.size(); i ++) {
		sample[i]->weight = sample[i]->weight / total_weight;
	}

	logi("Resampled ", sample.size(), " particles");
	for (int i = 0; i < sample.size(); i ++) {
		logi(" ", i, " = ", *sample[i]);
	}

	return sample;
}

vector<State*> Belief::Resample(int num, const DSPOMDP* model, const StateIndexer* indexer, int action, uint64_t obs) {
	assert(indexer != NULL);

	vector<State*> sample;

	for (int s = 0; s < indexer->NumStates(); s ++) {
		const State* state = indexer->GetState(s);
		double prob = model->ObsProb(obs, *state, action);
		if (prob > 0) {
			State* particle = model->Copy(state);
			particle->weight = prob;
			sample.push_back(particle);
		}
	}

	return sample;
}

vector<State*> Belief::Resample(int num, const Belief& belief, History history, int hstart) {
	double reward;
	uint64_t obs;

	vector<State*> sample;
	int count = 0;
	int pos = 0;
	double max_wgt = Globals::NEG_INFTY;
	vector<State*> particles;
	int trial = 0;
	while (count < num || trial < 200 * num) {
		// Pick next particle
		if (pos == particles.size()) {
			particles = belief.Sample(num);
			pos = 0;
		}
		State* particle = particles[pos];

		trial ++;

		// Step through history
		double log_wgt = 0;
		for (int i = hstart; i < history.Size(); i ++) {
			belief.model_->Step(*particle, Random::RANDOM.NextDouble(), history.Action(i), reward, obs);

			double prob = belief.model_->ObsProb(history.Observation(i), *particle, history.Action(i));
			if (prob > 0) {
				log_wgt += log(prob);
			} else {
				belief.model_->Free(particle);
				break;
			}
		}

		// Add to sample if survived
		if (particle->IsAllocated()) {
			particle->weight = log_wgt;
			sample.push_back(particle);

			max_wgt = max(log_wgt, max_wgt);
			count ++;
		}

		// Remove particles with very small weights
		if (count == num) {
			for (int i = sample.size()-1; i >= 0; i --) {
				if (sample[i]->weight - max_wgt < log(1.0/num)) {
					belief.model_->Free(sample[i]);
					sample.erase(sample.begin() + i);
					count --;
				}
			}
		}

		pos ++;
	}

	// Free unused particles
	for (int i = pos; i < particles.size(); i ++)
		belief.model_->Free(particles[i]);

	double total_weight = 0;
	for (int i = 0; i < sample.size(); i ++) {
		sample[i]->weight = exp(sample[i]->weight - max_wgt);
		total_weight += sample[i]->weight;
	}
	for (int i = 0; i < sample.size(); i ++) {
		sample[i]->weight = sample[i]->weight / total_weight;
	}

	logi("Resampled ", sample.size(), " particles");
	for (int i = 0; i < sample.size(); i ++) {
		logi(" ", i, " = ", *sample[i]);
	}

	return sample;
}

ParticleBelief::ParticleBelief(vector<State*> particles, const DSPOMDP* model, Belief* prior) :
	Belief(model),
	particles_(particles),
	num_particles_(particles.size()),
	prior_(prior),
	state_indexer_(NULL) {
	if (2 * num_particles_ < 1000) {
		num_particles_ = 1000;
		vector<State*> new_belief = Belief::Sample(num_particles_, particles_, model_);
		for (State* particle : particles_)
			model_->Free(particle);

		particles_ = new_belief;
	}

	assert(fabs(State::Weight(particles) - 1.0) < 1e-6);

	random_shuffle(particles_.begin(), particles_.end());
	cerr << "Number of particles in initial belief: " << particles_.size() << endl;

	if (prior_ == NULL) {
		for (State* particle : particles)
			initial_particles_.push_back(model_->Copy(particle));
	}
}

ParticleBelief::~ParticleBelief() {
	for (State* particle : particles_) {
		model_->Free(particle);
	}

	for (State* particle: initial_particles_) {
		model_->Free(particle);
	}
}

void ParticleBelief::state_indexer(const StateIndexer* indexer) {
	state_indexer_ = indexer;
}

const vector<State*>& ParticleBelief::particles() const {
	return particles_;
}

vector<State*> ParticleBelief::Sample(int num) const {
	return Belief::Sample(num, particles_, model_);
}

void ParticleBelief::Update(int action, uint64_t obs) {
	history_.Add(action, obs);

	vector<State*> updated;
	double total_weight = 0;
	double reward;
	uint64_t o;
	// Update particles
	for (State* particle : particles_) {
		bool terminal = model_->Step(*particle, Random::RANDOM.NextDouble(), action, reward, o);
		double prob = model_->ObsProb(obs, *particle, action);

		if (!terminal && prob) { // Terminal state is not required to be explicitly represented and may not have any observation
			particle->weight *= prob;
			total_weight += particle->weight;
			updated.push_back(particle);
		} else {
			model_->Free(particle);
		}
	}

	logi("", updated.size(), " particles survived among ", particles_.size());
	particles_ = updated;

	// Resample if the particle set is empty
	if (particles_.size() == 0) {
		cerr << "Particle set is empty!" << endl;	
		if (prior_ != NULL) {
			cerr << "Resampling by drawing random particles from prior which are consistent with history" << endl;
			particles_ = Resample(num_particles_, *prior_, history_);
		} else {
			cerr << "Resampling by searching initial particles which are consistent with history" << endl;
			particles_ = Resample(num_particles_, initial_particles_, model_, history_);
		}

		if (particles_.size() == 0 && state_indexer_ != NULL) {
			cerr << "Resampling by searching states consistent with last (action, observation) pair" << endl;
			particles_ = Resample(num_particles_, model_, state_indexer_, action ,obs);
		}

		if (particles_.size() == 0) {
			cerr << "Resampling failed - Using initial particles" << endl;
			for (auto& particle : initial_particles_)
				particles_.push_back(model_->Copy(particle));
		}
		cerr << *this << endl;
	}

	double weight_square_sum = 0;
	for (State* particle : particles_) {
		particle->weight /= total_weight;
		weight_square_sum += particle->weight * particle->weight;
	}

	// Resample if the effective number of particles is "small"
	double num_effective_particles = 1.0 / weight_square_sum;
	if (num_effective_particles < num_particles_ / 2.0) {
		vector<State*> new_belief = Belief::Sample(num_particles_, particles_, model_);
		for (State* particle : particles_)
			model_->Free(particle);

		particles_ = new_belief;
	}
}

Belief* ParticleBelief::MakeCopy() const {
	vector<State*> copy;
	for (State* particle : particles_) {
		copy.push_back(model_->Copy(particle));
	}

	return new ParticleBelief(copy, model_, prior_);
}

string ParticleBelief::text() const {
	ostringstream oss;
	map<string, double> pdf;
	oss << "Particles:" << endl;
	for (int i = 0; i < particles_.size(); i ++) {
		oss << " " << i << " = " << *particles_[i] << endl;;
		pdf[particles_[i]->text()] += particles_[i]->weight;
	}

	oss << "pdf:" << endl;
	for (auto& it : pdf) {
		oss << " " << it.first << " = " << it.second << endl;
	}
	return oss.str();
}
