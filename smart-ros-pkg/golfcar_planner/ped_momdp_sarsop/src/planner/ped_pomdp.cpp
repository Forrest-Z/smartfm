#include "ped_pomdp.h"

int PedPomdp::action_vel[3] = {0, 1, -1};

class PedPomdpParticleLowerBound : public ParticleLowerBound {
public:
	PedPomdpParticleLowerBound(const DSPOMDP* model) :
		ParticleLowerBound(model) {
	}

	virtual ValuedAction Value(const vector<State*>& particles) const {
		PomdpState* state = static_cast<PomdpState*>(particles[0]);
		//return ValuedAction(0, State::Weight(particles) * ModelParams::CRASH_PENALTY * ModelParams::VEL_MAX / (1 - Discount()));
		return ValuedAction(0, State::Weight(particles) * ModelParams::CRASH_PENALTY * state->car.vel / (1 - Discount()));
	}
};


PedPomdp::PedPomdp(WorldModel &model_) :
	world(model_),
	random_(Random((unsigned) Seeds::Next()))
{
	particle_lower_bound_ = new PedPomdpParticleLowerBound(this);
}

vector<int> PedPomdp::ObserveVector(const State& state_) const {
	const PomdpState &state=static_cast<const PomdpState&>(state_);
	static vector<int> obs_vec;
	obs_vec.resize(state.num*2+2);
	//int rx=int(state.car.pos.x/ModelParams::pos_rln);
	//obs_vec.push_back(rx);
	//int ry=int(state.car.pos.y/ModelParams::pos_rln);
	//obs_vec.push_back(ry);
	int i=0;
    obs_vec[i++]=(state.car.pos);
	int rvel=int(state.car.vel/ModelParams::vel_rln);
	obs_vec[i++]=(rvel);

	for(int i = 0; i < state.num; i ++) {
		int px = int(state.peds[i].pos.x/ModelParams::pos_rln);
		int py = int(state.peds[i].pos.y/ModelParams::pos_rln);
		obs_vec[i++]=(px);
		obs_vec[i++]=(py);
	}
	return obs_vec;
}

//PomdpState PedPomdp::Discretize(const PomdpState& state) const {
//	PomdpState 	
//}

uint64_t PedPomdp::Observe(const State& state) const {
	hash<vector<int>> myhash;
	return myhash(ObserveVector(state));
}


vector<State*> PedPomdp::ConstructParticles(vector<PomdpState> & samples) {
	int num_particles=samples.size();
	vector<State*> particles;
	for(int i=0;i<samples.size();i++) {
		PomdpState* particle = static_cast<PomdpState*>(Allocate(-1, 1.0/num_particles));
		(*particle)=samples[i];
		particle->weight=1.0/num_particles;
		particles.push_back(particle);
	}
	return particles;
}

bool PedPomdp::Step(State& state_, double rNum, int action, double& reward, uint64_t& obs) const {
	const bool REWARD_DIST = false;

	PomdpState& state = static_cast<PomdpState&>(state_);
	reward = 0;
	if(world.isLocalGoal(state)) {
		reward = (REWARD_DIST ? 0 : ModelParams::GOAL_REWARD)
			+ (state.car.dist_travelled - ModelParams::GOAL_TRAVELLED)
			- (ModelParams::VEL_MAX/ModelParams::control_freq);
		//cout << "goal reached!" << endl;
		//PrintState(state, cout);
		return true;
	}
	double collision_penalty = world.inCollision(state,action);
	if(collision_penalty < 0) {
		reward += collision_penalty;
	}

	int rob_vel = state.car.vel;

	Random random(rNum);
	double p = random.NextDouble();
	//reward += (action == ACT_DEC) ? 10 : -1;

	double acc;
	if (action == ACT_ACC) {
		acc = ModelParams::AccSpeed;
	} else if (action == ACT_CUR) {
		acc = 0;
	} else {
		acc = -ModelParams::AccSpeed;
	}

	
	world.RobStep(state.car, random);

	if (REWARD_DIST)
		reward+=state.car.vel/ModelParams::control_freq*(ModelParams::GOAL_REWARD/ModelParams::GOAL_TRAVELLED);

	world.RobVelStep(state.car, acc, random);
	for(int i=0;i<state.num;i++)
		world.PedStep(state.peds[i], random);

	obs = Observe(state);

	//assert(reward>=0);
	return false;
}
	
double PedPomdp::ObsProb(uint64_t obs, const State& s, int action) const {
	return obs == Observe(s);
}

// TODO
vector<vector<double>> PedPomdp::GetBeliefVector(const vector<State*> particles) const {
	/*
	double goal_count[10][10] = {0};

	PomdpState state_0 = particles[0]->state;
	for(int i=0;i<particles.size();i++) {
		PomdpState state=particles[i]->state;
		for(int j=0;j<state.num;j++) {
			goal_count[j][state.peds[j].goal]+=particles[i]->weight;
		}
	}
	*/

	vector<vector<double>> belief_vec;
	/*
	for(int j=0;j<state_0.num;j++) {
		vector<double> belief;
		for(int i=0;i<ModelParams::NGOAL;i++) {
			belief.push_back((goal_count[j][i]+0.0));
		}
		belief_vec.push_back(belief);
	}
	*/
	return belief_vec;
}
/*
class PedPomdpBelief : public Belief {
private:
	const PedPomdp* model_;

public:
	PedPomdpBelief(const PedPomdp* model) :
		Belief(model),
		model_(model)
	{
	}

	void Update(int action, uint64_t obs) { // TODO
	}

	vector<State*> Sample(int num_particles) const { // TODO
		//vector<State*> sample;
		//model_->world.getPomdpState();
		//model_->world.sample_goal();
		return sample;
	}

	Belief* MakeCopy() const { // Not really needed
		return NULL;
	}

	string text() const { // TODO
		return "TODO";
	}
};
*/
Belief* PedPomdp::InitialBelief(const State* start, string type) const {
	assert(false);
	return NULL;
}

void PedPomdp::Statistics(const vector<PomdpState*> particles) const {
	double goal_count[10][10]={0};
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
	return ValuedAction(0, ModelParams::CRASH_PENALTY * ModelParams::VEL_MAX);
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
			RandomStreams& streams, History& history) const { // TODO default policy
		return ped_pomdp_->world.defaultPolicy(particles);
		//const PomdpState* state = static_cast<const PomdpState*>(particles[0]);
				

		/*
		PomdpState state=particles[0]->state;
		int robY=state.car.pos.y
		int rx=rob_map[robY].first;
		int ry=rob_map[robY].second;
		double rate=ModelParams::map_rln/ModelParams::rln;

		int rangeX=(ModelParams::map_rln/ModelParams::rln)*1;
		int rangeY=(ModelParams::map_rln/ModelParams::rln)*3;
		for(int i=0;i<state.num;i++)
		{
			int px=state.peds[i].pos.x;
			int py=state.peds[i].pos.y;
			int crash_point=sfm->crash_model[px][py];
			int crashx=rob_map[crash_point].first;
			int crashy=rob_map[crash_point].second;
			if(abs(px-crashx)<=rangeX&&crashy-ry>=-rate&&crashy-ry<=rangeY)
			{
				return 2;
			}
			//if(abs(px-crashx)<=rangeX*2+2&&crashy-ry>=-4&&crashy-ry<=rangeY*2+2)
			if(abs(px-crashx)<=rangeX*2&&crashy-ry>=-rate&&crashy-ry<=rangeY*2)
			{

				double unit=ModelParams::VEL_MAX/(ModelParams::VEL_N-1);
				if(state.car.vel>1.0/unit) return 2;
				else if(state.car.vel<0.5/unit) return 1;
				else return 0;
			}
		}
		*/
		//return 1;
	}
};

void PedPomdp::InitializeScenarioLowerBound(string name, RandomStreams& streams) {
	//name = "TRIVIAL";
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
	return ModelParams::GOAL_REWARD;
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

	double Value(const State& s) const {
		const PomdpState& state = static_cast<const PomdpState&>(s);

		//TODO noise
		int d = ped_pomdp_->world.minStepToGoal(state);

		return ModelParams::GOAL_REWARD * pow(Discount(), d);
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
	
	out << "Rob Pos: " << carpos.x<< " " <<carpos.y << endl;
	out << "Rob travelled: " << state.car.dist_travelled << endl;
	for(int i = 0; i < state.num; i ++) {
		out << "Ped Pos: " << state.peds[i].pos.x << " " << state.peds[i].pos.y << endl;
		out << "Goal: " << state.peds[i].goal << endl;
		out << "id: " << state.peds[i].id << endl;
	}
	out << "Vel: " << state.car.vel << endl;
	out<<  "num  " << state.num << endl;
	
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
