#include<numeric_limits>
#include"WorldModel.h"
using namespace std;

WorldModel(): freq(ModelParams::control_freq) {
    goals = {
        COORD(54, 4),
        COORD(31, 4),
        COORD(5,  5),
        COORD(44,49),
        COORD(18,62)
    };
}

bool WorldModel::isLocalGoal(PomdpState state) {
    return state.car.dist_travelled > ModelParams::GOAL_TRAVELLED;
}

bool WorldModel::isGlobalGoal(CarStruct car) {
    double d = EuclideanDistance(car.pos, path[path.size()-1]);  
    return (d<ModelParams::tolerance);
}

double WorldModel::inCollision(PomdpState state, int action) {
    double penalty = 0;
    double mindist = numeric_limits<double>::infinity();
    auto& carpos = state.car.pos;
    double carvel = state.car.vel;
    for(auto& p: state.peds) {
        double d = EuclideanDistance(carpos, p.pos);
        if(d < mindist) mindist = d;
    }

    if(mindist < 1) {
        penalty += CRASH_PENALTY * (carvel + 1);
    }

    if(carvel > 1.0 and mindist < 2) {
        penalty += CRASH_PENALTY / 2;
    }
    return penalty;
}

void WorldModel::PedStep(PedStruct &ped, UtilUniform &unif) {
    COORD& goal = goals[ped.goal];
	MyVector goal_vec(goal.x - pos.x, goal.y - pos.y);
    double a = goal_vec.GetAngle();
    a += unif.NextGaussian() * ModelParams::NOISE_GOAL_ANGLE;
    //TODO noisy speed
    MyVector move(a, ModelParams::PED_SPEED, 0);
    goal.x += move.dw;
    goal.y += move.dh;
    return;
}

void WorldModel::RobStep(CarStruct &car, UtilUniform &unif) {
    //TODO noise
    double dist = car.vel / freq;
    int curr = path.nearest(car.pos);
    int nxt = path.forward(curr, dist);
    car.pos = path[nxt];
    car.dist_travelled += dist;
}

void WorldModel::RobVelStep(CarStruct &car, double acc, UtilUniform& unfi) {
    double prob = unif.next();
    if (prob > 0.2) {
        car.vel += acc / freq;
    }
    if (car.vel < 0) car.vel = 0;
    if (car.vel > ModelParams::MAX_VEL) car.vel = ModelParams::MAX_VEL;
    return;
}

void WorldModel::setPath(Path path) {
    this->path = path;
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


void WorldStateTracker::updatePed(Pedestrian& ped){
    for(int i=0;i<ped_list.size();i++)
    {
        if(ped_list[i].id==ped.id)
        {
            //found the corresponding ped,update the pose
            ped_list[i].w=ped.w;
            ped_list[i].h=ped.h;
            ped_list[i].last_update = timestamp();
            break;
        }
        if(abs(ped_list[i].w-ped.w)<=0.1&&abs(ped_list[i].h-ped.h)<=0.1)   //overladp 
            return;
    }
    if(i==ped_list.size())   //not found, new ped
    {
        ped.last_update = timestamp();
        ped_list.push_back(ped);
    }
}
void WorldStateTracker::updateCar(COORD& car) {
    this->car.pos=car;
}

void updateVel(double vel) {
    this->car.vel=vel;
}
PomdpState getPomdpState() {
    PomdpState pomdpState;
    pomdpState.car=car;
    pomdpState.num=ped_list.size();
    for(int i=0;i<pomdpState.num;i++) {
        pomdpState.peds[i].pos.x=ped_list[i].w;
        pomdpState.peds[i].pos.y=ped_list[i].h;
    }
}
