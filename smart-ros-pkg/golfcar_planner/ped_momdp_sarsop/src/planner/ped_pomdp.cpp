#include "ped_pomdp.h"

class PedPomdpParticleLowerBound : public ParticleLowerBound {
private:
	const PedPomdp* ped_pomdp_;
public:
	PedPomdpParticleLowerBound(const DSPOMDP* model) :
		ParticleLowerBound(model),
		ped_pomdp_(static_cast<const PedPomdp*>(model))
	{
	}

    // IMPORTANT: Check after changing reward function.
	virtual ValuedAction Value(const vector<State*>& particles) const {
		PomdpState* state = static_cast<PomdpState*>(particles[0]);
		int min_step = numeric_limits<int>::max();
		auto& carpos = ped_pomdp_->world.path[state->car.pos];
		double carvel = state->car.vel;
	
		// Find mininum num of steps for car-pedestrian collision
		for (int i=0; i<state->num; i++) {
			auto& p = state->peds[i];
            // 3.25 is maximum distance to collision boundary from front laser (see collsion.cpp)
			int step = int(ceil(ModelParams::control_freq
						* max(COORD::EuclideanDistance(carpos, p.pos) - 3.25, 0.0)
						/ ((p.vel + carvel))));
			min_step = min(step, min_step);
		}

        double move_penalty = ped_pomdp_->MovementPenalty(*state);

        // Case 1, no pedestrian: Constant car speed
		double value = move_penalty / (1 - Discount());
        // Case 2, with pedestrians: Constant car speed, head-on collision with nearest neighbor
		if (min_step != numeric_limits<int>::max()) {
            double crash_penalty = ped_pomdp_->CrashPenalty(*state);
			value = (move_penalty) * (1 - Discount(min_step)) / (1 - Discount()) 
				+ crash_penalty * Discount(min_step);
		}

		return ValuedAction(ped_pomdp_->ACT_CUR, State::Weight(particles) * value);
	}
};

PedPomdp::PedPomdp(WorldModel &model_) :
	world(model_),
	random_(Random((unsigned) Seeds::Next()))
{
	particle_lower_bound_ = new PedPomdpParticleLowerBound(this);
}

const vector<int>& PedPomdp::ObserveVector(const State& state_) const {
	const PomdpState &state=static_cast<const PomdpState&>(state_);
	static vector<int> obs_vec;
	obs_vec.resize(state.num * 2 + 2);

	int i=0;
    obs_vec[i++] = state.car.pos;
	obs_vec[i++] = int(state.car.vel / ModelParams::vel_rln);

	for(int j = 0; j < state.num; j ++) {
		obs_vec[i++] = int(state.peds[j].pos.x / ModelParams::pos_rln); 
		obs_vec[i++] = int(state.peds[j].pos.y / ModelParams::pos_rln);
	}

	return obs_vec;
}

uint64_t PedPomdp::Observe(const State& state) const {
	hash<vector<int>> myhash;
	return myhash(ObserveVector(state));
}

vector<State*> PedPomdp::ConstructParticles(vector<PomdpState> & samples) {
	int num_particles=samples.size();
	vector<State*> particles;
	for(int i=0;i<samples.size();i++) {
		PomdpState* particle = static_cast<PomdpState*>(Allocate(-1, 1.0/num_particles));
		(*particle) = samples[i];
		particle->SetAllocated();
		particle->weight = 1.0/num_particles;
		particles.push_back(particle);
	}
	return particles;
}

// Very high cost for collision
double PedPomdp::CrashPenalty(const PomdpState& state) const { // , int closest_ped, double closest_dist) const {
	// double ped_vel = state.ped[closest_ped].vel;
    return ModelParams::CRASH_PENALTY * (state.car.vel * state.car.vel + ModelParams::REWARD_BASE_CRASH_VEL); 
}

// Avoid frequent dec or acc
double PedPomdp::ActionPenalty(int action) const {
    return (action == ACT_DEC || action == ACT_ACC) ? -0.1 : 0.0;
}

// Less penalty for longer distance travelled
double PedPomdp::MovementPenalty(const PomdpState& state) const {
    return ModelParams::REWARD_FACTOR_VEL * (state.car.vel - ModelParams::VEL_MAX) / ModelParams::VEL_MAX;
    /*
    // with no pedestrians nearby, but lower speed with pedestrians nearby
    retrun //(min_dist > 3.0 || (min_dist < 3.0 && action != ACT_ACC)) ?
		(1.0 * (state.car.vel - ModelParams::VEL_MAX) / ModelParams::VEL_MAX);
		//: -1.5;
    */
}

bool PedPomdp::Step(State& state_, double rNum, int action, double& reward, uint64_t& obs) const {
	PomdpState& state = static_cast<PomdpState&>(state_);
	reward = 0.0;

	// CHECK: relative weights of each reward component
	// Terminate upon reaching goal
	if (world.isLocalGoal(state)) {
        reward = ModelParams::GOAL_REWARD;
		return true;
	}

	//int closest_front_ped;
	//double closest_front_dist;
	//int closest_side_ped;
	//double closest_side_dist;
	//world.getClosestPed(state, closest_front_ped, closest_front_dist,
			//closest_side_ped, closest_side_dist);

 	// Safety control: collision; Terminate upon collision
	//if (closest_front_dist < ModelParams::COLLISION_DISTANCE) {
    if(world.inCollision(state)) {
		reward = CrashPenalty(state); //, closest_ped, closest_dist);
		return true;
	}

	// Forbidden actions
    double carvel = state.car.vel;
	if (action == ACT_CUR && 0.1 < carvel && carvel < 0.6) {
		reward = CrashPenalty(state);
		return true;
	}
    if (action == ACT_ACC && carvel >= ModelParams::VEL_MAX) {
		reward = CrashPenalty(state);
		return true;
    }
    if (action == ACT_DEC && carvel <= 0.01) {
		reward = CrashPenalty(state);
		return true;
    }

    // encourage speed up when there's no pedestrian
    //if (carvel < 0.1 && closest_side_dist > 1.5 && closest_front_dist > 3.5 &&
            //(closest_front_ped < 0 || world.isMovingAway(state, closest_front_ped))) {
        //reward += -10;
    //}


    // encourage slowdown when pedestrian is close
    //double min_dist_all_dirs = min(closest_front_dist, closest_side_dist);
	//if (min_dist_all_dirs<6.0 && state.car.vel>1.0) {
		//reward+=-4;
	//}

	/*
	if ((min_dist < 2.5 && (state.car.vel > 1.0 || action == ACT_ACC)) || (min_dist > 4.0 && state.car.vel < 0.5)) {
		reward += -1000;
	}
	*/

	// Smoothness control
	reward += ActionPenalty(action);

	// Speed control: Encourage higher speed
	reward += MovementPenalty(state);

	// State transition
	Random random(rNum);
	double acc = (action == ACT_ACC) ? ModelParams::AccSpeed :
		((action == ACT_CUR) ?  0 : (-ModelParams::AccSpeed));
	world.RobStep(state.car, random);
	world.RobVelStep(state.car, acc, random);
	for(int i=0;i<state.num;i++)
		world.PedStep(state.peds[i], random);

	// Observation
	obs = Observe(state);

	return false;
}

double PedPomdp::ObsProb(uint64_t obs, const State& s, int action) const {
	return obs == Observe(s);
}

vector<vector<double>> PedPomdp::GetBeliefVector(const vector<State*> particles) const {
	vector<vector<double>> belief_vec;
	return belief_vec;
}

Belief* PedPomdp::InitialBelief(const State* start, string type) const {
	assert(false);
	return NULL;
}

void PedPomdp::Statistics(const vector<PomdpState*> particles) const {
	double goal_count[10][10]={{0}};
	cout << "Current Belief" << endl;
	if(particles.size() == 0)
		return;

	PrintState(*particles[0]);
	PomdpState* state_0 = particles[0];
	for(int i = 0; i < particles.size(); i ++) {
		PomdpState* state = particles[i];
		for(int j = 0; j < state->num; j ++) {
			goal_count[j][state->peds[j].goal] += particles[i]->weight;
		}
	}

	for(int j = 0; j < state_0->num; j ++) {
		cout << "Ped " << j << " Belief is ";
		for(int i = 0; i < world.goals.size(); i ++) {
			cout << (goal_count[j][i] + 0.0) <<" ";
		}
		cout << endl;
	}
}


ValuedAction PedPomdp::GetMinRewardAction() const {
	return ValuedAction(0, 
			ModelParams::CRASH_PENALTY * (ModelParams::VEL_MAX*ModelParams::VEL_MAX + ModelParams::REWARD_BASE_CRASH_VEL));
}

class PedPomdpSmartScenarioLowerBound : public Policy {
protected:
	const PedPomdp* ped_pomdp_;

public:
	PedPomdpSmartScenarioLowerBound(const DSPOMDP* model) :
		Policy(model),
		ped_pomdp_(static_cast<const PedPomdp*>(model))
	{
	}

	int Action(const vector<State*>& particles,
			RandomStreams& streams, History& history) const {
		return ped_pomdp_->world.defaultPolicy(particles);
	}
};

void PedPomdp::InitializeScenarioLowerBound(string name, RandomStreams& streams) {
	// name = "TRIVIAL";
	name="SMART";
	if(name == "TRIVIAL") {
		scenario_lower_bound_ = new TrivialScenarioLowerBound(this);
	} else if(name == "RANDOM") {
		scenario_lower_bound_ = new RandomPolicy(this);
	} else if (name == "SMART") {
		scenario_lower_bound_ = new PedPomdpSmartScenarioLowerBound(this);
	} else {
		cerr << "Unsupported scenario lower bound: " << name << endl;
		exit(0);
	}
}

double PedPomdp::GetMaxReward() const {
	return 0;
}

class PedPomdpSmartParticleUpperBound : public ParticleUpperBound {
protected:
	const PedPomdp* ped_pomdp_;
public:
	PedPomdpSmartParticleUpperBound(const DSPOMDP* model) :
		ParticleUpperBound(model),
		ped_pomdp_(static_cast<const PedPomdp*>(model))
	{
	}

    // IMPORTANT: Check after changing reward function.
	double Value(const State& s) const {
		const PomdpState& state = static_cast<const PomdpState&>(s);
        /*
        double min_dist = ped_pomdp_->world.getMinCarPedDist(state);
        if (min_dist < ModelParams::COLLISION_DISTANCE) {
            return ped_pomdp_->CrashPenalty(state);
        }
        */
        if (ped_pomdp_->world.inCollision(state))
            return ped_pomdp_->CrashPenalty(state);

        int min_step = ped_pomdp_->world.minStepToGoal(state);
		return ModelParams::GOAL_REWARD * Discount(min_step);
	}
};

void PedPomdp::InitializeParticleUpperBound(string name, RandomStreams& streams) {
	name = "SMART";
	if (name == "TRIVIAL") {
		particle_upper_bound_ = new TrivialParticleUpperBound(this);
	} else if (name == "SMART") {
		particle_upper_bound_ = new PedPomdpSmartParticleUpperBound(this);
	} else {
		cerr << "Unsupported particle upper bound: " << name << endl;
		exit(0);
	}
}

void PedPomdp::InitializeScenarioUpperBound(string name, RandomStreams& streams) {
	//name = "SMART";
	name = "TRIVIAL";
	if (name == "TRIVIAL") {
		scenario_upper_bound_ = new TrivialScenarioUpperBound(this);
	} else {
		cerr << "Unsupported scenario upper bound: " << name << endl;
		exit(0);
	}
}

void PedPomdp::PrintState(const State& s, ostream& out) const {
	const PomdpState & state=static_cast<const PomdpState&> (s);
    COORD& carpos = world.path[state.car.pos];

	out << "car pos / dist_trav / vel = " << "(" << carpos.x<< ", " <<carpos.y << ") / " 
        << state.car.dist_travelled << " / "
        << state.car.vel << endl;
	out<< state.num << " pedestrians " << endl;
	for(int i = 0; i < state.num; i ++) {
		out << "ped " << i << ": id / pos / vel / goal / dist2car / infront =  " << state.peds[i].id << " / "
            << "(" << state.peds[i].pos.x << ", " << state.peds[i].pos.y << ") / "
            << state.peds[i].vel << " / "
            << state.peds[i].goal << " / "
            << COORD::EuclideanDistance(state.peds[i].pos, carpos) << "/"
			<< world.inFront(state.peds[i].pos, state.car.pos) << endl;
	}
    double min_dist = -1;
    if (state.num > 0)
        min_dist = COORD::EuclideanDistance(carpos, state.peds[0].pos);
	out << "MinDist: " << min_dist << endl;
}

void PedPomdp::PrintObs(const State&state, uint64_t obs, ostream& out) const {
	out << obs << endl;
}

void PedPomdp::PrintAction(int action, ostream& out) const {
	out << action << endl;
}

void PedPomdp::PrintBelief(const Belief& belief, ostream& out ) const {
	
}

State* PedPomdp::Allocate(int state_id, double weight) const {
	num_active_particles ++;
	PomdpState* particle = memory_pool_.Allocate();
	particle->state_id = state_id;
	particle->weight = weight;
	return particle;
}

State* PedPomdp::Copy(const State* particle) const {
	num_active_particles ++;
	PomdpState* new_particle = memory_pool_.Allocate();
	*new_particle = *static_cast<const PomdpState*>(particle);
	new_particle->SetAllocated();
	return new_particle;
}

void PedPomdp::Free(State* particle) const {
	num_active_particles --;
	memory_pool_.Free(static_cast<PomdpState*>(particle));
}
