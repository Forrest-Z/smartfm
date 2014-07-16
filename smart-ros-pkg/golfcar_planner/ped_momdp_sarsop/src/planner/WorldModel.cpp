#include<limits>
#include<cmath>
#include<cstdlib>
#include"WorldModel.h"
#include"math_utils.h"
#include"coord.h"
using namespace std;

WorldModel::WorldModel(): freq(ModelParams::control_freq),
    in_front_angle_cos(cos(ModelParams::IN_FRONT_ANGLE_DEG / 180.0 * M_PI)) {
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
    return state.car.dist_travelled > ModelParams::GOAL_TRAVELLED || state.car.pos >= path.size()-1;
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
		if(!inFront(p.pos, state->car.pos)) continue;
        double d = COORD::EuclideanDistance(carpos, p.pos);
		// cout << d << " " << carpos.x << " " << carpos.y << " "<< p.pos.x << " " << p.pos.y << endl;
        if (d >= 0 && d < mindist) 
			mindist = d;
    }

	// cout << "min_dist = " << mindist << endl;

    // TODO set as a param
    if (mindist < 3) {
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
	const COORD& car_pos = path[car];
	const COORD& forward_pos = path[path.forward(car, 1.0)];
	double d0 = COORD::EuclideanDistance(car_pos, ped_pos);
	if(d0<=0) return true;
	double d1 = COORD::EuclideanDistance(car_pos, forward_pos);
	if(d1<=0) return false;
	double dot = DotProduct(forward_pos.x - car_pos.x, forward_pos.y - car_pos.y,
			ped_pos.x - car_pos.x, ped_pos.y - car_pos.y);
	double cosa = dot / (d0 * d1);
	cosa = min(cosa, 1.0);
	cosa = max(cosa, -1.0);
    return cosa > in_front_angle_cos;
	//double angle = acos(cosa);
	//return (fabs(angle) < M_PI / 180 * 60);
}

double WorldModel::getMinCarPedDist(const PomdpState& state) {
    double mindist = numeric_limits<double>::infinity();
    const auto& carpos = path[state.car.pos];

	// Find the closest pedestrian in front
    for(int i=0; i<state.num; i++) {
		const auto& p = state.peds[i];
		if(!inFront(p.pos, state.car.pos)) continue;
        double d = COORD::EuclideanDistance(carpos, p.pos);
        if (d >= 0 && d < mindist) mindist = d;
    }

	return mindist;
}

double WorldModel::getMinCarPedDistAllDirs(const PomdpState& state) {
    double mindist = numeric_limits<double>::infinity();
    const auto& carpos = path[state.car.pos];

	// Find the closest pedestrian in front
    for(int i=0; i<state.num; i++) {
		const auto& p = state.peds[i];
        double d = COORD::EuclideanDistance(carpos, p.pos);
        if (d >= 0 && d < mindist) mindist = d;
    }

	return mindist;
}

bool WorldModel::inCollision(const PomdpState& state) {
    double mindist = numeric_limits<double>::infinity();
    const auto& carpos = path[state.car.pos];
    //double carvel = state.car.vel;

	// Find the closest pedestrian in front
    for(int i=0; i<state.num; i++) {
		const auto& p = state.peds[i];
		if(!inFront(p.pos, state.car.pos)) continue;
        double d = COORD::EuclideanDistance(carpos, p.pos);
        if (d >= 0 && d < mindist) mindist = d;
    }

	return mindist < 1.0;
}

int WorldModel::minStepToGoal(const PomdpState& state) {
    double d = ModelParams::GOAL_TRAVELLED - state.car.dist_travelled;
    if (d < 0) d = 0;
    return int(ceil(d / (ModelParams::VEL_MAX/freq)));
}

void WorldModel::PedStep(PedStruct &ped, Random& random) {
    const COORD& goal = goals[ped.goal];
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

void WorldModel::PedStepDeterministic(PedStruct& ped, int step) {
    const COORD& goal = goals[ped.goal];
	if (goal.x == -1 && goal.y == -1) {  //stop intention
		return;
	}

	MyVector goal_vec(goal.x - ped.pos.x, goal.y - ped.pos.y);
    goal_vec.AdjustLength(step * ModelParams::PED_SPEED / freq);
    ped.pos.x += goal_vec.dw;
    ped.pos.y += goal_vec.dh;
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
		return (move_dist < sensor_noise) ? 0.4 : 0;
	} else {
		if (move_dist < sensor_noise) return 0;

		double cosa = DotProduct(curr.x-prev.x, curr.y-prev.y, goal.x-prev.x, goal.y-prev.y) / (move_dist * goal_dist);
		double angle = acos(cosa);
		return gaussian_prob(angle, ModelParams::NOISE_GOAL_ANGLE) + K;
	}
}

void WorldModel::RobStep(CarStruct &car, Random& random) {
    double dist = car.vel / freq;
    //double dist_l=max(0.0,dist-ModelParams::AccSpeed/freq);
    //double dist_r=min(ModelParams::VEL_MAX,dist+ModelParams::AccSpeed/freq);
    //double sample_dist=random.NextDouble(dist_l,dist_r);
    //int nxt = path.forward(car.pos, sample_dist);
    int nxt = path.forward(car.pos, dist);
    car.pos = nxt;
    car.dist_travelled += dist;
}

void WorldModel::RobVelStep(CarStruct &car, double acc, Random& random) {
    const double N = ModelParams::NOISE_ROBVEL;
    if (N>0) {
        double prob = random.NextDouble();
        if (prob > N) {
            car.vel += acc / freq;
        }
    } else {
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
	const double SMOOTHING=ModelParams::BELIEF_SMOOTHING;
	for(double w: b.prob_goals) {
		cout << w << " ";
	}
	cout << endl;
    for(int i=0; i<goals.size(); i++) {
		double prob = pedMoveProb(b.pos, curr_ped.pos, i);
		cout << "likelihood " << i << ": " << prob << endl;
        b.prob_goals[i] *=  prob;

		// Important: Keep the belief noisy to avoid aggressive policies
		b.prob_goals[i] += SMOOTHING / goals.size(); // CHECK: decrease or increase noise
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
    //return ((double) clock()) / CLOCKS_PER_SEC;
    static double starttime=get_time_second();
    return get_time_second()-starttime;
}

void WorldStateTracker::cleanPed() {
    vector<Pedestrian> ped_list_new;
    for(int i=0;i<ped_list.size();i++)
    {
        bool insert=true;
        double w1,h1;
        w1=ped_list[i].w;
        h1=ped_list[i].h;
        for(const auto& p: ped_list_new) {
            double w2,h2;
            w2=p.w;
            h2=p.h;
            if (abs(w1-w2)<=0.1&&abs(h1-h2)<=0.1) {
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
    for(;i<ped_list.size();i++) {
        if (ped_list[i].id==ped.id) {
            //found the corresponding ped,update the pose
            ped_list[i].w=ped.w;
            ped_list[i].h=ped.h;
            ped_list[i].last_update = timestamp();
            break;
        }
        if (abs(ped_list[i].w-ped.w)<=0.1 && abs(ped_list[i].h-ped.h)<=0.1)   //overlap
            return;
    }
    if (i==ped_list.size()) {
        //not found, new ped
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
    cout << "before sorting:" << endl;
    // sort peds
    vector<PedDistPair> sorted_peds;
    for(const auto& p: ped_list) {
        COORD cp(p.w, p.h);
        float dist = COORD::EuclideanDistance(cp, carpos);
        sorted_peds.push_back(PedDistPair(dist, p));
        cout << " " << dist;
    }
    cout << endl;
    sort(sorted_peds.begin(), sorted_peds.end(),
            [](const PedDistPair& a, const PedDistPair& b) -> bool {
                return a.first < b.first;
            });
    cout << "after sorting:" << endl;
    for(const auto& p : sorted_peds) {
        cout << " " << p.first;
    }
    cout << endl;

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

    cout<<"pedestrian time stamps"<<endl;
    for(int i=0;i<pomdpState.num;i++) {
        const auto& ped = sorted_peds[i].second;
        pomdpState.peds[i].pos.x=ped.w;
        pomdpState.peds[i].pos.y=ped.h;
		pomdpState.peds[i].id = ped.id;
		pomdpState.peds[i].goal = -1;
        cout<<"ped "<<i<<" "<<ped.last_update<<endl;
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
    vector<int> peds_to_remove;
    for(const auto& p: peds) {
        if (newpeds.find(p.first) == newpeds.end()) {
            peds_to_remove.push_back(p.first);
        }
    }
    for(const auto& i: peds_to_remove) {
        peds.erase(i);
    }

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

int PedBelief::sample_goal() const {
    double r = double(rand()) / RAND_MAX;
    int i = 0;
    r -= prob_goals[i];
    while(r > 0) {
        i++;
        r -= prob_goals[i];
    }
    return i;
}

int PedBelief::maxlikely_goal() const {
    double ml = 0;
    int mi = prob_goals.size()-1; // stop intention
    for(int i=0; i<prob_goals.size(); i++) {
        if (prob_goals[i] > ml && prob_goals[i] > 0.5) {
            ml = prob_goals[i];
            mi = i;
        }
    }
    return mi;
}

void WorldBeliefTracker::printBelief() const {
	int num = 0;
    for(int i=0; i < sorted_beliefs.size() && i < ModelParams::N_PED_IN; i++) {
		auto& p = sorted_beliefs[i];
		if (COORD::EuclideanDistance(p.pos, model.path[car.pos]) < 5) {
            cout << "ped belief " << num << ": ";
            for (int g = 0; g < p.prob_goals.size(); g ++)
                cout << " " << p.prob_goals[g];
            cout << endl;
		}
    }
}

PomdpState WorldBeliefTracker::sample() {
    PomdpState s;
    s.car = car;

	s.num = 0;
    for(int i=0; i < sorted_beliefs.size() && i < ModelParams::N_PED_IN; i++) {
		auto& p = sorted_beliefs[i];
		if (COORD::EuclideanDistance(p.pos, model.path[car.pos]) < 10) {
			s.peds[s.num].pos = p.pos;
			s.peds[s.num].goal = p.sample_goal();
			s.peds[s.num].id = p.id;
			s.num ++;
		}
    }
    //cout<<"print state from planner "<<endl;
    //PrintState(s,cout);
    return s;
}
vector<PomdpState> WorldBeliefTracker::sample(int num) {
    vector<PomdpState> particles;
    for(int i=0; i<num; i++) {
        particles.push_back(sample());
    }

    cout << "Num peds for planning: " << particles[0].num << endl;

    return particles;
}

vector<PedStruct> WorldBeliefTracker::predictPeds() {
    vector<PedStruct> prediction;

    for(const auto& p: sorted_beliefs) {
        double dist = COORD::EuclideanDistance(p.pos, model.path[car.pos]);
        int step = int(dist / (ModelParams::PED_SPEED + car.vel) * ModelParams::control_freq);
        //for(int j=0; j<10; j++) {
            //int goal = p.sample_goal();
        for(int j=0; j<1; j++) {
            int goal = p.maxlikely_goal();
            PedStruct ped0(p.pos, goal, p.id);
            for(int i=0; i<3; i++) {
                PedStruct ped = ped0;
                model.PedStepDeterministic(ped, step+i*6);
                prediction.push_back(ped);
            }
        }
    }
    return prediction;
}

void WorldBeliefTracker::PrintState(const State& s, ostream& out) const {
	const PomdpState & state=static_cast<const PomdpState&> (s);
    COORD& carpos = model.path[state.car.pos];

	out << "Rob Pos: " << carpos.x<< " " <<carpos.y << endl;
	out << "Rob travelled: " << state.car.dist_travelled << endl;
	for(int i = 0; i < state.num; i ++) {
		out << "Ped Pos: " << state.peds[i].pos.x << " " << state.peds[i].pos.y << endl;
		out << "Goal: " << state.peds[i].goal << endl;
		out << "id: " << state.peds[i].id << endl;
	}
	out << "Vel: " << state.car.vel << endl;
	out<<  "num  " << state.num << endl;
	double min_dist = COORD::EuclideanDistance(carpos, state.peds[0].pos);
	out << "MinDist: " << min_dist << endl;
}

