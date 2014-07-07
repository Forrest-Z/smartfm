#include<limits>
#include<cmath>
#include<cstdlib>
#include"WorldModel.h"
#include"math_utils.h"
#include"coord.h"
using namespace std;

WorldModel::WorldModel(): freq(ModelParams::control_freq) {
    goals = {
        COORD(54, 4),
        COORD(31, 4),
        COORD(5,  5),
        COORD(44,49),
        COORD(18,62),
		COORD(-1,-1)
    };
	/*
    goals = {
        COORD(107, 167),
        COORD(121, 169),
        COORD(125,  143),
        COORD(109,109),
        COORD(122,114)
    };
	*/
}

bool WorldModel::isLocalGoal(const PomdpState& state) {
    return state.car.dist_travelled > ModelParams::GOAL_TRAVELLED;
}

bool WorldModel::isGlobalGoal(const CarStruct& car) {
    double d = COORD::EuclideanDistance(path[car.pos], path[path.size()-1]);
    return (d<ModelParams::GOAL_TOLERANCE);
}

int WorldModel::defaultPolicy(const vector<State*>& particles)  {
	const PomdpState *state=static_cast<const PomdpState*>(particles[0]);
    double mindist = numeric_limits<double>::infinity();
    auto& carpos = path[state->car.pos];
    double carvel = state->car.vel;
    for (int i=0; i<state->num; i++) {
		auto& p = state->peds[i];
        double d = COORD::EuclideanDistance(carpos, p.pos) *  inFront(p.pos, state->car.pos);
		// cout << d << " " << carpos.x << " " << carpos.y << " "<< p.pos.x << " " << p.pos.y << endl;
        if (d > 0 && d < mindist) 
			mindist = d;
    }

	// cout << "min_dist = " << mindist << endl;

    // TODO set as a param
    if (mindist < 4) {
		return (carvel == 0.0) ? 0 : 2;
    }

    if (mindist < 6) {
		if (carvel > 1.0) return 2;	
		else if (carvel < 0.5) return 1;
		else return 0;
    }
    return 1;
}

bool WorldModel::inFront(COORD ped_pos, int car) const {
	COORD car_pos = path[car];
	COORD forward_pos = path[path.forward(car, 1.0)];

	/*NOTE: To increase the region checked, compute the cosine value and set a negative threshold.*/
	return DotProduct(forward_pos.x - car_pos.x, forward_pos.y - car_pos.y,
			ped_pos.x - car_pos.x, ped_pos.y - ped_pos.y) > 0;
}

bool WorldModel::inCollision(const PomdpState& state) {
    double mindist = numeric_limits<double>::infinity();
    auto& carpos = path[state.car.pos];
    double carvel = state.car.vel;

	// Find the closest pedestrian in front
    for(int i=0; i<state.num; i++) {
		auto& p = state.peds[i];
        double d = COORD::EuclideanDistance(carpos, p.pos) * inFront(p.pos, state.car.pos);
        if (d > 0 && d < mindist) mindist = d;
    }

	return mindist < 2;
}

int WorldModel::minStepToGoal(const PomdpState& state) {
    double d = ModelParams::GOAL_TRAVELLED - state.car.dist_travelled;
    if (d < 0) d = 0;
    return ceil(d / (ModelParams::VEL_MAX/freq));
}

void WorldModel::PedStep(PedStruct &ped, Random& random) {
    COORD& goal = goals[ped.goal];
	if (goal.x == -1 && goal.y == -1) {  //stop intention 
		return;
	}
	
	MyVector goal_vec(goal.x - ped.pos.x, goal.y - ped.pos.y);
    double a = goal_vec.GetAngle();
	double noise = random.NextGaussian() * ModelParams::NOISE_GOAL_ANGLE;
    a += noise;
    
	//TODO noisy speed
    MyVector move(a, ModelParams::PED_SPEED/freq, 0);
    ped.pos.x += move.dw;
    ped.pos.y += move.dh;
    return;
}

double gaussian_prob(double x, double stddev) {
    double a = 1.0 / stddev / sqrt(2 * M_PI);
    double b = - x * x / 2.0 / (stddev * stddev);
    return a * exp(b);
}

double WorldModel::pedMoveProb(COORD prev, COORD curr, int goal_id) {
	const double K = 0.001;
    const COORD& goal = goals[goal_id];
	double move_dist = Norm(curr.x-prev.x, curr.y-prev.y),
		   goal_dist = Norm(goal.x-prev.x, goal.y-prev.y);
	double sensor_noise = 0.1;

	// CHECK: beneficial to add back noise?
	cout<<"goal id "<<goal_id<<endl;
	if (goal.x == -1 && goal.y == -1) {  //stop intention 
		return (move_dist < sensor_noise);
	} else {
		if (move_dist < sensor_noise) return 0;

		double cosa = DotProduct(curr.x-prev.x, curr.y-prev.y, goal.x-prev.x, goal.y-prev.y) / (move_dist * goal_dist);
		double angle = acos(cosa);
		return gaussian_prob(angle, ModelParams::NOISE_GOAL_ANGLE) + K;
	}
}

void WorldModel::RobStep(CarStruct &car, Random& random) {
    double dist = car.vel / freq;
    int nxt = path.forward(car.pos, dist);
    car.pos = nxt;
    car.dist_travelled += dist;
}

void WorldModel::RobVelStep(CarStruct &car, double acc, Random& random) {
    double prob = random.NextDouble();
    if (prob > 0.2) {
        car.vel += acc / freq;
    }

	car.vel = max(min(car.vel, ModelParams::VEL_MAX), 0.0);
    
	return;
}

void WorldModel::setPath(Path path) {
    this->path = path;
	/*
	for(int i=0; i<this->path.size(); i++) {
		const auto& p = this->path[i];
		cout << p.x << " " << p.y << endl;
	}*/
}

void WorldModel::updatePedBelief(PedBelief& b, const PedStruct& curr_ped) {
	for(double w: b.prob_goals) {
		cout << w << " ";
	}
	cout << endl;
    for(int i=0; i<goals.size(); i++) {
		double prob = pedMoveProb(b.pos, curr_ped.pos, i);
		cout << "likelihood " << i << ": " << prob << endl;
        b.prob_goals[i] *=  prob;

		// Important: Keep the belief noisy to avoid aggressive policies
		b.prob_goals[i] += 0.1 / goals.size(); // CHECK: decrease or increase noise
	}
	for(double w: b.prob_goals) {
		cout << w << " ";
	}
	cout << endl;

    // normalize
    double total_weight = accumulate(b.prob_goals.begin(), b.prob_goals.end(), double(0.0));
	cout << "total_weight = " << total_weight << endl;
    for(double& w : b.prob_goals) {
        w /= total_weight;
    }

	for(double w: b.prob_goals) {
		cout << w << " ";
	}
	cout << endl;

	b.pos = curr_ped.pos;
}

PedBelief WorldModel::initPedBelief(const PedStruct& ped) {
    PedBelief b = {ped.id, ped.pos, vector<double>(goals.size(), 1.0/goals.size())};
    return b;
}

double timestamp() {
    return ((double) clock()) / CLOCKS_PER_SEC;
}

void WorldStateTracker::cleanPed() {
    vector<Pedestrian> ped_list_new;
    //for(vector<Pedestrian>::iterator it=ped_list.begin();it!=ped_list.end();++it)
    for(int i=0;i<ped_list.size();i++)
    {
        bool insert=true;
        //for(vector<Pedestrian>::iterator it2=ped_list.begin();it2!=it;++it2)
        int w1,h1;
        w1=ped_list[i].w;
        h1=ped_list[i].h;
        for(int j=0;j<i;j++)
        {
            int w2,h2;
            w2=ped_list[j].w;
            h2=ped_list[j].h;
            //if (abs(it->w-it2->w)<=1&&abs(it->h-it2->h)<=1)
            if (abs(w1-w2)<=0.1&&abs(h1-h2)<=0.1)
            {
                insert=false;
                break;
            }
        }
        if (timestamp() - ped_list[i].last_update > 1.0) insert=false;
        if (insert)
            ped_list_new.push_back(ped_list[i]);
    }
    ped_list=ped_list_new;
}


void WorldStateTracker::updatePed(const Pedestrian& ped){
    int i=0;
    for(;i<ped_list.size();i++)
    {
        if (ped_list[i].id==ped.id)
        {
            //found the corresponding ped,update the pose
            ped_list[i].w=ped.w;
            ped_list[i].h=ped.h;
            ped_list[i].last_update = timestamp();
            break;
        }
        if (abs(ped_list[i].w-ped.w)<=0.1&&abs(ped_list[i].h-ped.h)<=0.1)   //overlap 
            return;
    }
    if (i==ped_list.size())   //not found, new ped
    {
        ped_list.push_back(ped);
        ped_list.back().last_update = timestamp();
    }
}

void WorldStateTracker::updateCar(const COORD& car) {
    carpos=car;
}

bool WorldStateTracker::emergency() {
    //TODO improve emergency stop to prevent the false detection of leg
    double mindist = numeric_limits<double>::infinity();
    for(auto& ped : ped_list) {
		COORD p(ped.w, ped.h);
        double d = COORD::EuclideanDistance(carpos, p);
        if (d < mindist) mindist = d;
    }
	cout << "emergency mindist = " << mindist << endl;
	return (mindist < 0.5);
}

void WorldStateTracker::updateVel(double vel) {
	/*
	if (vel>ModelParams::VEL_MAX)
		carvel=ModelParams::VEL_MAX;
	else carvel = vel;
	*/
	carvel = vel;
}

vector<WorldStateTracker::PedDistPair> WorldStateTracker::getSortedPeds() {
    // sort peds
    vector<PedDistPair> sorted_peds;
    for(const auto& p: ped_list) {
        COORD cp(p.w, p.h);
        float dist = COORD::EuclideanDistance(cp, carpos);
        sorted_peds.push_back(PedDistPair(dist, p));
    }
    sort(sorted_peds.begin(), sorted_peds.end(),
            [](const PedDistPair& a, const PedDistPair& b) -> bool {
                return a.first < b.first;
            });
    return sorted_peds;
}

PomdpState WorldStateTracker::getPomdpState() {
    auto sorted_peds = getSortedPeds();

    // construct PomdpState
    PomdpState pomdpState;
    pomdpState.car.pos = model.path.nearest(carpos);
    pomdpState.car.vel = carvel;
	pomdpState.car.dist_travelled = 0;
    pomdpState.num = sorted_peds.size();

	if (pomdpState.num > ModelParams::N_PED_IN) {
		pomdpState.num = ModelParams::N_PED_IN;
	}

    for(int i=0;i<pomdpState.num;i++) {
        const auto& ped = sorted_peds[i].second;
        pomdpState.peds[i].pos.x=ped.w;
        pomdpState.peds[i].pos.y=ped.h;
		pomdpState.peds[i].id = ped.id;
		pomdpState.peds[i].goal = -1;
    }
	return pomdpState;
}

void WorldBeliefTracker::update() {
    // update car
    car.pos = model.path.nearest(stateTracker.carpos);
    car.vel = stateTracker.carvel;
	car.dist_travelled = 0;

    auto sorted_peds = stateTracker.getSortedPeds();
    map<int, PedStruct> newpeds;
    for(const auto& dp: sorted_peds) {
        auto& p = dp.second;
        PedStruct ped(COORD(p.w, p.h), -1, p.id);
        newpeds[p.id] = ped;
    }

    // remove disappeared peds
    auto peds_disappeared = find_if (peds.begin(), peds.end(),
                [&](decltype(*peds.begin()) p) -> bool {
                return newpeds.find(p.first) == newpeds.end(); });
    peds.erase(peds_disappeared, peds.end());

    // update existing peds
    for(auto& kv : peds) {
        model.updatePedBelief(kv.second, newpeds[kv.first]);
    }

    // add new peds
    for(const auto& kv: newpeds) {
		auto& p = kv.second;
        if (peds.find(p.id) == peds.end()) {
            peds[p.id] = model.initPedBelief(p);
        }
    }

	sorted_beliefs.clear();
	for(const auto& dp: sorted_peds) {
		auto& p = dp.second;
		sorted_beliefs.push_back(peds[p.id]);
	}

    return;
}

int PedBelief::sample_goal() {
    double r = double(rand()) / RAND_MAX;
    int i = 0;
    r -= prob_goals[i];
    while(r > 0) {
        i++;
        r -= prob_goals[i];
    }
    return i;
}

PomdpState WorldBeliefTracker::sample() {
    PomdpState s;
    s.car = car;
    s.num = min(int(sorted_beliefs.size()), ModelParams::N_PED_IN);

    for(int i=0; i<s.num; i++) {
		auto& p = sorted_beliefs[i];
        s.peds[i].pos = p.pos;
        s.peds[i].goal = p.sample_goal();
        s.peds[i].id = p.id;
    }
    return s;
}

vector<PomdpState> WorldBeliefTracker::sample(int num) {
    vector<PomdpState> particles;
    for(int i=0; i<num; i++) {
        particles.push_back(sample());
    }
    return particles;
}
