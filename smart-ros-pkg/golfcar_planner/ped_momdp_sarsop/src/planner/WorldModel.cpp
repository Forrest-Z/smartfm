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
		//COORD(-1,-1)
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
    for(int i=0; i<state->num; i++) {
		auto& p = state->peds[i];
        double d = COORD::EuclideanDistance(carpos, p.pos);
        if(d < mindist) mindist = d;
    }

    // TODO set as a param
    if(mindist < 3) {
		//assert(false);
		return 2;
    }

    if(mindist < 6) {
		//assert(false);
		if(carvel>1.0) return 2;	
		else if(carvel<0.5) return 1;
		else return 0;
    }
    return 1;
}
double WorldModel::inCollision(const PomdpState& state, int action) {
    double penalty = 0;
    double mindist = numeric_limits<double>::infinity();
    auto& carpos = path[state.car.pos];
    double carvel = state.car.vel;
    for(int i=0; i<state.num; i++) {
		auto& p = state.peds[i];
        double d = COORD::EuclideanDistance(carpos, p.pos);
        if(d < mindist) mindist = d;
    }

    // TODO set as a param
    if(mindist < 2 && carvel > 0) {
        penalty += ModelParams::CRASH_PENALTY * (carvel + 1);
    }

    if(carvel > 1.0 && mindist < 4) {
        penalty += ModelParams::CRASH_PENALTY / 2;
    }
	if (penalty != 0) {
		//cout << "penalty =" << penalty << endl;
	}
    return penalty;
}

double WorldModel::minStepToGoal(const PomdpState& state) {
    double d = ModelParams::GOAL_TRAVELLED - state.car.dist_travelled;
    if (d < 0) d = 0;
    return d / (ModelParams::VEL_MAX/freq);
}


void WorldModel::PedStep(PedStruct &ped, Random& random) {
    COORD& goal = goals[ped.goal];
	if(goal.x==-1&&goal.y==-1) {  //stop intention 
		if(random.NextDouble()<0.5)	 { //move	
			double a = random.NextDouble()*2*3.14;
			MyVector move(a, ModelParams::PED_SPEED/freq, 0);
			ped.pos.x += move.dw;
			ped.pos.y += move.dh;
		}
		return;
	}
//	if(random.NextDouble()<0.1) return;   //not move
	MyVector goal_vec(goal.x - ped.pos.x, goal.y - ped.pos.y);
    double a = goal_vec.GetAngle();
    a += random.NextGaussian() * ModelParams::NOISE_GOAL_ANGLE;
    //TODO noisy speed
    MyVector move(a, ModelParams::PED_SPEED/freq, 0);
    ped.pos.x += move.dw;
    ped.pos.y += move.dh;
    return;
}

inline double sqr(double a) {
    return a*a;
}

double gaussian_prob(double x, double stddev) {
    double a = 1.0 / stddev / sqrt(2 * M_PI);
    double b = -sqr(x) / 2 / sqr(stddev);
    return a * exp(b);
}

double WorldModel::pedMoveProb(COORD p0, COORD p1, int goal_id) {
	const double K = 0.001;
    const COORD& goal = goals[goal_id];
	cout<<"goal id "<<goal_id<<endl;
	if(goal.x==-1&&goal.y==-1) {  //stop intention 
		double a = ModelParams::PED_SPEED*0.5/freq;
		double b = 6*a;
		double p;
		if(Norm(p1.x-p0.x, p1.y-p0.y)<a) {
			p = 0.9/a;
		}
		else p = 0.1/(b-a);
		return p * 0.2;
	}

	double norm = Norm(p1.x-p0.x, p1.y-p0.y) * Norm(goal.x-p0.x, goal.y-p0.y);
	if(norm <= 0.0)
		return 0.1;
    double cosa = DotProduct(p1.x-p0.x, p1.y-p0.y, goal.x-p0.x, goal.y-p0.y) / norm;
    double a = acos(cosa);
    double p = gaussian_prob(a, ModelParams::NOISE_GOAL_ANGLE) + K;

    return p;
}

void WorldModel::RobStep(CarStruct &car, Random& random) {
    //TODO noise
    double dist = car.vel / freq;
    //int curr = path.nearest(car.pos);
    int nxt = path.forward(car.pos, dist);
    car.pos = nxt;
    car.dist_travelled += dist;
}

void WorldModel::RobVelStep(CarStruct &car, double acc, Random& random) {
    double prob = random.NextDouble();
    if (prob > 0.2) {
        car.vel += acc / freq;
    }
    if (car.vel < 0) car.vel = 0;
    if (car.vel > ModelParams::VEL_MAX) car.vel = ModelParams::VEL_MAX;
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
		cout << prob << endl;
        b.prob_goals[i] *=  prob;
		b.prob_goals[i] += 0.01;
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
    PedBelief b = {ped.pos, vector<double>(goals.size(), 1.0/goals.size())};
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
            //if(abs(it->w-it2->w)<=1&&abs(it->h-it2->h)<=1)
            if(abs(w1-w2)<=0.1&&abs(h1-h2)<=0.1)
            {
                insert=false;
                break;
            }
        }
        if(timestamp() - ped_list[i].last_update > 1.0) insert=false;
        if(insert)
            ped_list_new.push_back(ped_list[i]);
    }
    ped_list=ped_list_new;
}


void WorldStateTracker::updatePed(const Pedestrian& ped){
    int i=0;
    for(;i<ped_list.size();i++)
    {
        if(ped_list[i].id==ped.id)
        {
            //found the corresponding ped,update the pose
            ped_list[i].w=ped.w;
            ped_list[i].h=ped.h;
            ped_list[i].last_update = timestamp();
            break;
        }
        if(abs(ped_list[i].w-ped.w)<=0.1&&abs(ped_list[i].h-ped.h)<=0.1)   //overlap 
            return;
    }
    if(i==ped_list.size())   //not found, new ped
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
        if(d < mindist) mindist = d;
    }
	cout << "emergency mindist = " << mindist << endl;
	return (mindist < 0.5);
}

void WorldStateTracker::updateVel(double vel) {
    carvel = vel;
}

PomdpState WorldStateTracker::getPomdpState() {
    PomdpState pomdpState;
    pomdpState.car.pos = model.path.nearest(carpos);
    pomdpState.car.vel = carvel;
	pomdpState.car.dist_travelled = 0;
    pomdpState.num = ped_list.size();

    //assert(pomdpState.num <= ModelParams::N_PED_IN);
	if(pomdpState.num > ModelParams::N_PED_IN) {
		pomdpState.num = ModelParams::N_PED_IN;
	}

    for(int i=0;i<pomdpState.num;i++) {
        pomdpState.peds[i].pos.x=ped_list[i].w;
        pomdpState.peds[i].pos.y=ped_list[i].h;
		pomdpState.peds[i].id = ped_list[i].id;
		pomdpState.peds[i].goal = -1;
    }
	return pomdpState;
}

void WorldBeliefTracker::update(const PomdpState& s) {
    car = s.car;

    map<int, PedStruct> newpeds;
    for(int i=0; i<s.num; i++) {
		auto& p = s.peds[i];
        newpeds[p.id] = p;
    }

    // remove disappeared peds
    auto peds_disappeared = find_if(peds.begin(), peds.end(),
                [&](decltype(*peds.begin()) p) -> bool {
                return newpeds.find(p.first) == newpeds.end(); });
    peds.erase(peds_disappeared, peds.end());

    // update existing peds
    for(auto& kv : peds) {
        model.updatePedBelief(kv.second, newpeds[kv.first]);
    }

    // add new peds
    for(int i=0; i<s.num; i++) {
		auto& p = s.peds[i];
        if (peds.find(p.id) == peds.end()) {
            peds[p.id] = model.initPedBelief(p);
        }
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
    s.num = peds.size();

    int i=0;
    for(auto& kv : peds) {
        s.peds[i].pos = kv.second.pos;
        s.peds[i].goal = kv.second.sample_goal();
        s.peds[i].id = kv.first;
        i++;
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


