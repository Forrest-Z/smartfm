#include"ped_pomdp.h"
#include "despotstar.h"
#include "WorldModel.h"
#include"state.h"
#include"Path.h"

using namespace std;

const int n_sim = 5000;

const double PED_X0 = 35;
const double PED_Y0 = 35;
const double PED_X1 = 42;
const double PED_Y1 = 47;
const int n_peds = 6; // should be smaller than ModelParams::N_PED_IN

class Simulator {
public:
    Simulator(): start(40, 40), goal(40, 45)
    {
        Path p;
        p.push_back(start);
        p.push_back(goal);
        path = p.interpolate();
        worldModel.setPath(path);
    }

    void run() {
        cout << "====================" << endl;
        WorldStateTracker stateTracker(worldModel);
        WorldBeliefTracker beliefTracker(worldModel, stateTracker);
        PedPomdp pomdp(worldModel);

        RandomStreams streams(Seeds::Next(Globals::config.n_particles), Globals::config.search_depth);
        pomdp.InitializeParticleLowerBound("smart");
        pomdp.InitializeScenarioLowerBound("smart", streams);
        pomdp.InitializeParticleUpperBound("smart", streams);
        pomdp.InitializeScenarioUpperBound("smart", streams);
        DESPOTSTAR solver(&pomdp, NULL, streams);

        // init state
        PomdpState s;
        s.car.pos = 0;
        s.car.vel = 0;
        s.car.dist_travelled = 0;
        s.num = n_peds;
        for(int i=0; i<n_peds; i++) {
            s.peds[i] = randomPed();
            s.peds[i].id = i;
        }

        double total_reward = 0;
        int step = 0;

        for(int step=0; step < 50; step++) {
            cout << "step=" << step << endl;
            double reward;
            uint64_t obs;
            stateTracker.updateCar(path[s.car.pos]);
            stateTracker.updateVel(s.car.vel);
            for(int i=0; i<n_peds; i++) {
                Pedestrian p(s.peds[i].pos.x, s.peds[i].pos.y, s.peds[i].id);
                stateTracker.updatePed(p);
            }

            if(worldModel.isGlobalGoal(s.car)) {
                cout << "goal_reached=1" << endl;
                break;
            }

            cout << "state=[[" << endl;
            pomdp.PrintState(s);
            cout << "]]" << endl;

            if(worldModel.inCollision(s)) {
                cout << "collision=1" << endl;
            }

            beliefTracker.update();
            vector<PomdpState> samples = beliefTracker.sample(Globals::config.n_particles);
		    vector<State*> particles = pomdp.ConstructParticles(samples);
            ParticleBelief* pb = new ParticleBelief(particles, &pomdp);
            solver.belief(pb);
            int act = solver.Search();
            cout << "act=" << act << endl;
            bool terminate = pomdp.Step(s,
                    Random::RANDOM.NextDouble(),
                    act, reward, obs);
            cout << "obs=" << obs << endl;
            cout << "reward=" << reward << endl;
            total_reward += reward * Globals::Discount(step);
            if(terminate) {
                cout << "terminate=1" << endl;
                break;
            }
        }
        cout << "final_state=[[" << endl;
        pomdp.PrintState(s);
        cout << "]]" << endl;
        cout << "total_reward=" << total_reward << endl;
        cout << "====================" << endl;
    }

    PedStruct randomPed() {
        int n_goals = worldModel.goals.size();
        int goal = Random::RANDOM.NextInt(n_goals);
        double x = Random::RANDOM.NextDouble(PED_X0, PED_X1);
        double y = Random::RANDOM.NextDouble(PED_Y0, PED_Y1);
        if(goal == n_goals-1) {
            // stop intention
            while(path.mindist(COORD(x, y)) < 1.0) {
                // dont spawn on the path
                x = Random::RANDOM.NextDouble(PED_X0, PED_X1);
                y = Random::RANDOM.NextDouble(PED_Y0, PED_Y1);
            }
        }
        int id = 0;
        return PedStruct(COORD(x, y), goal, id);
    }

    COORD start, goal;

    Path path;
    WorldModel worldModel;

};

int main(int argc, char** argv) {
    ModelParams::CRASH_PENALTY = -atof(argv[1]);
    
    Globals::config.n_particles=300;
    Globals::config.time_per_move = (1.0/ModelParams::control_freq) * 0.9;
    // TODO the seed should be initialized properly so that
    // different process as well as process on different machines
    // all get different seeds
    Seeds::root_seed(get_time_second());
    // Global random generator
    double seed = Seeds::Next();
    Random::RANDOM = Random(seed);
    cerr << "Initialized global random generator with seed " << seed << endl;


    Simulator sim;
    for(int i=0; i<n_sim; i++)
        sim.run();
}
