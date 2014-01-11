#include "belief_update/belief_update_exact.h"
#include "belief_update/belief_update_particle.h"
#include "globals.h"
#include "lower_bound/lower_bound_policy_mode.h"
#include "lower_bound/lower_bound_policy_random.h"
#include "lower_bound/lower_bound_policy_suffix.h"
#include "model.h"
#include "optionparser.h"
#include "problems/pedestrian_changelane/pedestrian_changelane.h"
#include "problems/pedestrian_changelane/map.h"
#include "problems/pedestrian_changelane/window.h"
#include "problems/pedestrian_changelane/math_utils.h"
#include "problems/pedestrian_changelane/SFM.h"
#include "problems/pedestrian_changelane/param.h"
#include "solver.h"
#include "util_uniform.h"
#include "upper_bound/upper_bound_nonstochastic.h"
#include "upper_bound/upper_bound_stochastic.h"
#include "world.h"
#include <iomanip>
#include "problems/pedestrian_changelane/world_simulator.h"

using namespace std;

Model<PedestrianState>* Simulator;
ILowerBound<PedestrianState>* lb_global;
IUpperBound<PedestrianState>* ub_global;
BeliefUpdate<PedestrianState>* bu_global;
RandomStreams * streams_global;
option::Option* options_global;
/* The seeds used by different components of the system to generate random 
 * numbers are all derived from the root seed so that an experiment can be 
 * repeated deterministically. The seed for the random-number stream of 
 * particle i is given by config.n_particles ^ i (see RandomStreams). The
 * remaining seeds are generated as follows.
 */
int WorldSeed() {
  cout<<"root seed"<<Globals::config.root_seed<<endl;
  return Globals::config.root_seed ^ Globals::config.n_particles;
}

int BeliefUpdateSeed() {
  return Globals::config.root_seed ^ (Globals::config.n_particles + 1);
}

// Action selection for RandomPolicyLowerBound (if used):
int RandomActionSeed() {
  return Globals::config.root_seed ^ (Globals::config.n_particles + 2);
}

WorldSimulator world;

void TestSimulator()
{
	Simulator->sfm=&world.sfm;
	Simulator->rob_map=world.window.rob_map;
	cout<<"test simulator"<<endl;

    VNode<PedestrianState>::set_model(*Simulator);
    World<PedestrianState> test_world = World<PedestrianState>(Globals::config.root_seed ^ Globals::config.n_particles, *Simulator);
	//world.SetStartStates(states);
	PedestrianState state=Simulator->GetStartState();
	Simulator->PrintState(state,cout);
	Solver<PedestrianState>* solver = new Solver<PedestrianState>(*Simulator, Simulator->InitialBelief(), *lb_global, *ub_global, *bu_global, *streams_global);
	solver->Init();
	int action;
	uint64_t observation;
	double reward;
	bool crashed=false;
	int i;

	for(i=0;i<100&&!solver->Finished() ;
i++)  //10 steps
	{
		cout<<"curr sim "<<i<<endl;
	//	Simulator->PrintState(world,cout);
		int n_trials;
		action=2;
		action=solver->Search(Globals::config.time_per_move,n_trials);
		cout<<"action "<<action<<endl;
		Simulator->sfm->debug=true;
		test_world.Step(action, observation, reward); 
		Simulator->sfm->debug=false;
		if(reward<-2000) crashed=true;
		Simulator->rob_map=world.window.rob_map;

		//cout<<"solver finished "<<solver->Finished()<<endl;
		PedestrianState state=test_world.GetState();
		solver->UpdateBelief(action,observation,state);

	}
	if(crashed)
		cout << "\nTotal  fail reward after " << i << " steps = "<<endl; 
	else
		cout << "\nTotal success reward after " << i << " steps = "<<endl;
}
/*
void Plan()
{


	robPosGlobal=0;	
	windowOrigin=robPosGlobal;

	PedestrianState temp_state=Simulator->GetStartState();
	curr_state=&temp_state;
	UpdateSim();

	MyWorld.Init();

	world.Init();

	InitPedestrians();
	int safeAction=0;
	int i;
	int crush=0;
	velGlobal=1.0;

	//TestSimulator();
	//return;

	for(i=1;i<15;i++)
	{
		cout<<"!!!!!!!!!curr horizon "<<i<<endl;
		safeAction=0;


		//select action
		char buf[20];
		sprintf(buf,"tree %d",i);
		ofstream out(buf);

		int action= solver->Search(Globals::config.time_per_move,n_trials);
		//safeAction=1;
		cout<<"safeAction "<<safeAction<<endl;

		
		//update environment

			


		bool terminal=Update(safeAction);
		if(terminal)   {crush++;break;}
		if(GoalReached()) break;

		if(i%5==0)
		{
			//move the horizon forward
			windowOrigin=robPosGlobal;
			int end=(windowOrigin+200<my_map.pathLength)?(windowOrigin+200):(my_map.pathLength-1);
			double dh=my_map.global_plan[end][1]-my_map.global_plan[windowOrigin][1];
			double dw=my_map.global_plan[end][0]-my_map.global_plan[windowOrigin][0];
			double angle=atan2(dw,dh);
			cout<<"dw dh angle "<<dw<<" "<<dh<<" "<<angle<<endl;
			cout<<"path length "<<my_map.pathLength<<endl;
			double yaw=sin(-angle/2);
			cout<<"yaw "<<yaw<<endl;

			//mapWindowSplit(init_w*rln,init_h*rln,yaw);
			my_window.MapWindowSplit(my_map.global_plan[windowOrigin][0],my_map.global_plan[windowOrigin][1],yaw);
			TransitionModel();
			my_window.UpdateRobMap(windowOrigin);
			UpdateSim();
		}
	}
	
	cout<<"total number of runs "<<i<<endl;
	cout<<"total number of crush "<<crush<<endl;
}*/



void Plan()
{
	//TestSimulator();
	Solver<PedestrianState>* solver=0;// = new Solver<PedestrianState>(*Simulator, Simulator->InitialBelief(), *lb_global, *ub_global, *bu_global, *streams_global);

	bool crashed=false;
	Simulator->rob_map=world.window.rob_map;
	Simulator->sfm=&world.sfm;
	int action;
	int i;

	PedestrianState ped_state=world.GetCurrState();
	if(ModelParams::debug)
	{
		cout<<"Initial State"<<endl;
		Simulator->PrintState(world.GetCurrState());
	}



	Simulator->SetStartState(ped_state);
	solver=new Solver<PedestrianState>(*Simulator, Simulator->InitialBelief(), *lb_global, *ub_global, *bu_global, *streams_global);	
	solver->Init();


	for(i=0;i<1000;i++)  
	{
		cout<<"Run "<<i<<endl;
		int n_trials;
		action=solver->Search(Globals::config.time_per_move,n_trials);
		cout<<"action "<<action<<endl;
		if(world.OneStep(action)) break;	
		world.ShiftWindow();
		if(ModelParams::debug)
		{
			cout<<"after shift window"<<endl;
			Simulator->PrintState(world.GetCurrState(),cout);
			world.Display();
		}
		uint64_t obs=world.GetCurrObs();
		Simulator->rob_map=world.window.rob_map;
		/*
		if(world.NumPedInView()==0&&solver) 
		{
			cout<<"clean problem"<<endl;
			delete solver;
			solver=0;
		}*/
		if(ModelParams::debug)  cout<<"observation "<<obs<<endl;
		if(solver)   {
			PedestrianState ped_state=world.GetCurrState();
			solver->UpdateBelief(action,obs,ped_state);	
			//cout<<"solver finished "<<solver->Finished()<<endl;
			//if(solver->Finished()) break;
		}
	}
	if(world.InCollision(action))
	{
		if(world.InRealCollision(action))
		{
			cout << "\nTotal  fail reward after " << i << " steps = "<<endl; 
		}
		cout << "\nTotal  danger reward after " << i << " steps = "<<endl; 
	}
	else
		cout << "\nTotal success reward after " << i << " steps = "<<endl;
}


void disableBufferedIO(void)
{
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    setbuf(stderr, NULL);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}


/*
int main(int argc,char**argv)
{

	cout<<"start run loop"<<endl;
	       
	BuildPriorTable();
	
	LoadMap();
	for(int i=0;i<1;i++)
	{
		
		LoadPath(i);	
		Plan();
		break;
	}
	cout<<"exit program"<<endl;
	return 0;
}*/


enum optionIndex { 
  UNKNOWN, HELP, PROBLEM, PARAMS_FILE, DEPTH, DISCOUNT, SEED, TIMEOUT, 
  NPARTICLES, PRUNE, SIMLEN, LBTYPE, BELIEF, KNOWLEDGE, APPROX_BOUNDS, NUMBER
};

// option::Arg::Required is a misnomer. The program won't complain if these 
// are absent, and required flags must be checked manually.
const option::Descriptor usage[] = {
 {UNKNOWN,       0, "",  "",              option::Arg::None,     "USAGE: despot [options]\n\nOptions:"},
 {HELP,          0, "",  "help",          option::Arg::None,     "  --help   \tPrint usage and exit."},
 {PROBLEM,       0, "q", "problem",       option::Arg::Required, "  -q <arg> \t--problem=<arg>  \tProblem name."},
 {PARAMS_FILE,   0, "m", "model-params",  option::Arg::Required, "  -m <arg> \t--model-params=<arg>  \tPath to model-parameters file, if any."},
 {DEPTH,         0, "d", "depth",         option::Arg::Required, "  -d <arg> \t--depth=<arg>  \tMaximum depth of search tree (default 90)."},
 {DISCOUNT,      0, "g", "discount",      option::Arg::Required, "  -g <arg> \t--discount=<arg>  \tDiscount factor (default 0.95)."},
 {SEED,          0, "r", "seed",          option::Arg::Required, "  -r <arg> \t--seed=<arg>  \tRandom number seed (default 42)."},
 {TIMEOUT,       0, "t", "timeout",       option::Arg::Required, "  -t <arg> \t--timeout=<arg>  \tSearch time per move, in seconds (default 1)."},
 {NPARTICLES,    0, "n", "nparticles",    option::Arg::Required, "  -n <arg> \t--nparticles=<arg>  \tNumber of particles (default 500)."},
 {PRUNE,         0, "p", "prune",         option::Arg::Required, "  -p <arg> \t--prune=<arg>  \tPruning constant (default no pruning)."},
 {SIMLEN,        0, "s", "simlen",        option::Arg::Required, "  -s <arg> \t--simlen=<arg>  \tNumber of steps to simulate. (default 90; 0 = infinite)."},
 {LBTYPE,        0, "l", "lbtype",        option::Arg::Required, "  -l <arg> \t--lbtype=<arg>  \tLower bound strategy, if applicable."},
 {BELIEF,        0, "b", "belief",        option::Arg::Required, "  -b <arg> \t--belief=<arg>  \tBelief update strategy, if applicable."},
 {KNOWLEDGE,     0, "k", "knowledge",     option::Arg::Required, "  -k <arg> \t--knowledge=<arg>  \tKnowledge level for random lower bound policy, if applicable."},
 {NUMBER,     0, "", "number",     option::Arg::Required, "--number=<arg>  \tNumber of pedestrians."},
 {APPROX_BOUNDS, 0, "a", "approx-bounds", option::Arg::None,     "  -a \t--approx-bounds  \tWhether initial lower/upper bounds are approximate or true (default false)."},
 {0,0,0,0,0,0}
};


template<typename T>
int RunMultiple(Model<T>* model, ILowerBound<T>* lb, IUpperBound<T>* ub, 
        BeliefUpdate<T>* bu, const RandomStreams& streams) {
	int num_ped = Globals::config.number;
  	cout<<"start run loop"<<endl;
	       
	
  	VNode<T>::set_model(*Simulator);
	//TestSimulator();
	Plan();


  /*
  VNode<T>::set_model(*model);
  World<T> world = World<T>(Globals::config.root_seed ^ Globals::config.n_particles, *model);
	vector<T> states = model->GetStartStates(num_ped);
	world.SetStartStates(states);

 
  vector<Solver<T>*> solvers;
	for(int i=0; i<num_ped; i++) {
		model->SetStartState(states[i]);
		Solver<T>* solver = new Solver<T>(*model, model->InitialBelief(), *lb, *ub, *bu, streams);
		solver->Init();
		solvers.push_back(solver);
	}

	cout << "Number of particles = " << Globals::config.n_particles << endl;
  cout << "\nSTARTING STATE:\n";
	for(int i=0; i<num_ped; i++) {
		cout << "Pedestrian " << i << endl;
		model->PrintState(states[i]);
	}

  int total_trials = 0, step;
	int order[] = {1, 0, 2};
	bool crashed = false;

  for (step = 0; (Globals::config.sim_len == 0 || step < Globals::config.sim_len); step++) {
		cout << "\nSTEP " << step + 1 << endl;
		int optimal = 1;
		for(int i=0; i<num_ped; i++) {
			Solver<T>* solver = solvers[i];
			int n_trials;
			int act = solver->Search(Globals::config.time_per_move, n_trials);

			if(order[act] > order[optimal])
				optimal = act;
		}

		vector<double> rewards; 
		vector<uint64_t> obss;
		world.StepMultiple(optimal, obss, rewards);

		bool finished = false;
		for(int i=0;i <num_ped; i++) {
			solvers[i]->UpdateBelief(optimal, obss[i]);

			if(solvers[i]->Finished())
				finished = true;

			if(rewards[i] == -50000)
				crashed = true;

			if(finished==true) break;
		}

		if(finished)
			break;
  }
  if(crashed)
	  cout << "\nTotal  fail reward after " << step+1 << " steps = " << world.TotalReward()
       << endl;
  else
  	  cout << "\nTotal success reward after " << step+1 << " steps = " << world.TotalReward()
       << endl;
  cout << "Undiscounted reward after " << step << " steps = " << world.TotalUndiscountedReward()
       << endl;
  cerr << "Average # of trials per move = "
       << (step == 0 ? 0 : (double)total_trials / step) << endl;
 
	for(int i=0; i<Globals::config.number; i++)
		delete solvers[i];

  return 0;*/
}


template<typename T>
int Run(Model<T>* model, ILowerBound<T>* lb, IUpperBound<T>* ub, 
        BeliefUpdate<T>* bu, const RandomStreams& streams) {
  VNode<T>::set_model(*model);
  World<T> world = World<T>(Globals::config.root_seed ^ Globals::config.n_particles, *model);

  Solver<T>* solver = 
      new Solver<T>(*model, model->InitialBelief(), *lb, *ub, *bu, streams);
  solver->Init();

  cout << "\nSTARTING STATE:\n";
  model->PrintState(model->GetStartState());

  int total_trials = 0, step;
  double reward; uint64_t obs;
  for (step = 0; 
       !solver->Finished() && (Globals::config.sim_len == 0 || step < Globals::config.sim_len);
       step++) {
		cout << "\nSTEP " << step + 1 << endl;
    int n_trials = 0;
    int act = solver->Search(Globals::config.time_per_move, n_trials); // each solver returns an action
    total_trials += n_trials;

    world.Step(act, obs, reward); 
    solver->UpdateBelief(act, obs);
  }
  if(reward==-50000)
	  cout << "\nTotal  fail reward after " << step << " steps = " << world.TotalReward()
       << endl;
  else
  	  cout << "\nTotal success reward after " << step << " steps = " << world.TotalReward()
       << endl;
  cout << "Undiscounted reward after " << step << " steps = " << world.TotalUndiscountedReward()
       << endl;
  
  delete solver;

  return 0;
}

int RunPedestrian(option::Option* options, const RandomStreams& streams) {
  if (!options[PARAMS_FILE]) {
    cerr << "pedestrian requires a params file\n";
    return 1;
  }
  auto model = new Model<PedestrianState>(streams, options[PARAMS_FILE].arg);
  Simulator  = new Model<PedestrianState>(streams, options[PARAMS_FILE].arg);

  ILowerBound<PedestrianState>* lb;
  string lb_type = options[LBTYPE] ? options[LBTYPE].arg : "random";
  if (lb_type == "random") {
    int knowledge = options[KNOWLEDGE] ? atoi(options[KNOWLEDGE].arg) : 2;
    lb = new RandomPolicyLowerBound<PedestrianState>(
        streams, knowledge, RandomActionSeed());
  }
  else {
    cerr << "pedestrian requires lower bound of type 'random'\n";
    return 1;
  }

  //IUpperBound<PedestrianState>* ub =
      //new UpperBoundStochastic<PedestrianState>(streams, *model);

  string bu_type = options[BELIEF] ? options[BELIEF].arg : "particle";
  BeliefUpdate<PedestrianState>* bu;
  if (bu_type == "particle")
    bu = new ParticleFilterUpdate<PedestrianState>(BeliefUpdateSeed(), *Simulator);
  else {
    cerr << "pedestrian requires belief update strategy of type 'particle'\n";
    return 1;
  }

  // int ret = Run(model, lb, ub, bu, streams);
  lb_global=lb;
  ub_global=Simulator;
  bu_global=bu;
  
  int ret = RunMultiple(model, lb, model, bu, streams);

  delete model;
  delete lb;
  // delete ub;
  delete bu;

  return ret;
}

int main(int argc, char* argv[]) {
  argc-=(argc>0); argv+=(argc>0); // skip program name argv[0] if present
  option::Stats stats(usage, argc, argv);
  option::Option* options = new option::Option[stats.options_max];
  option::Option* buffer = new option::Option[stats.buffer_max];
  option::Parser parse(usage, argc, argv, options, buffer);

  // Required parameters
  string problem;
  if (!options[PROBLEM]) {
    option::printUsage(std::cout, usage);
    return 0;
  }
  problem = options[PROBLEM].arg;

  // Optional parameters
  if (options[DEPTH]) Globals::config.search_depth = atoi(options[DEPTH].arg);
  if (options[DISCOUNT]) Globals::config.discount = atof(options[DISCOUNT].arg);
  if (options[SEED]) Globals::config.root_seed = atoi(options[SEED].arg);
  if (options[TIMEOUT]) Globals::config.time_per_move = atof(options[TIMEOUT].arg);
  if (options[NPARTICLES]) Globals::config.n_particles = atoi(options[NPARTICLES].arg);
  if (options[PRUNE]) Globals::config.pruning_constant = atof(options[PRUNE].arg);
  if (options[SIMLEN]) Globals::config.sim_len = atoi(options[SIMLEN].arg);
  if (options[APPROX_BOUNDS]) Globals::config.approximate_bounds = true;
	if (options[NUMBER]) Globals::config.number = atoi(options[NUMBER].arg);

  RandomStreams streams(Globals::config.n_particles, Globals::config.search_depth, 
                        Globals::config.root_seed);

  cout<<"option seed "<<Globals::config.root_seed<<endl;

  streams_global=&streams;
  world.SetSeed(WorldSeed());
  world.NumPedTotal=Globals::config.number;
  world.Init();

  if (problem == "pedestrian")
    return RunPedestrian(options, streams);
  else
	  cout << "Problem must be one of tag, lasertag, rocksample, tiger, bridge "
          "and pocman.\n";

  return 1;
}
