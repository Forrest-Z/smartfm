#include <typeinfo>
#include "despot.h"
#include "despotstar.h"
#include "aems.h"
#include "pomdp.h"
#include "optionparser.h"
#include "util/seeds.h"
#include "problems/bridge.h"
#include "problems/chain.h"
#include "problems/lasertag.h"
#include "problems/navigation.h"
// #include "problems/pedestrian.h"
#include "problems/pocman.h"
#include "problems/pomdpx.h"
#include "problems/rock_sample.h"
#include "problems/fvrs.h"
#include "problems/tag.h"
#include "problems/tiger.h"
#include "test.h"
#include "Client.h"
#include <chrono>
#include "util/pomdpx_parser/parser.h"

using namespace chrono;
using namespace std;

void disableBufferedIO(void) {
	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
	setbuf(stderr, NULL);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);
}

enum OptionIndex {
	E_UNKNOWN,
	E_HELP,
	E_PROBLEM,
	E_PARAMS_FILE,
	E_DEPTH,
	E_DISCOUNT,
	E_SIZE,
	E_NUMBER,
	E_SEED,
	E_TIMEOUT,
	E_NPARTICLES,
	E_PRUNE,
	E_GAP,
	E_SIM_LEN,
	E_SIMULATOR,
	E_MAX_POLICY_SIM_LEN,
	E_DEFAULT_ACTION,
	E_RUNS,
	E_BLBTYPE,
	E_LBTYPE,
	E_BUBTYPE,
	E_UBTYPE,
	E_BELIEF,
	E_KNOWLEDGE,
	E_VERBOSITY,
	E_SOLVER,
	E_MDP_BOUND_CREATION,
	E_BOUND_LINK,
	E_TIME_LIMIT,
	E_NOISE,
};

// option::Arg::Required is a misnomer. The program won't complain if these
// are absent, and required flags must be checked manually.
const option::Descriptor usage[] = {
	{E_UNKNOWN,       0, "",  "",              option::Arg::None,     "USAGE: despot [options]\n\nOptions:"},
	{E_HELP,          0, "",  "help",          option::Arg::None,     "  --help   \tPrint usage and exit."},
	{E_PROBLEM,       0, "q", "problem",       option::Arg::Required, "  -q <arg> \t--problem=<arg>  \tProblem name."},
	{E_PARAMS_FILE,   0, "m", "model-params",  option::Arg::Required, "  -m <arg> \t--model-params=<arg>  \tPath to model-parameters file, if any."},
	{E_DEPTH,         0, "d", "depth",         option::Arg::Required, "  -d <arg> \t--depth=<arg>  \tMaximum depth of search tree (default 90)."},
	{E_DISCOUNT,      0, "g", "discount",      option::Arg::Required, "  -g <arg> \t--discount=<arg>  \tDiscount factor (default 0.95)."},
	{E_SIZE,      0, "", "size",      option::Arg::Required, "  --size <arg>\t."},
	{E_NUMBER,      0, "", "number",      option::Arg::Required, "  --number <arg> \t."},
	{E_SEED,          0, "r", "seed",          option::Arg::Required, "  -r <arg> \t--seed=<arg>  \tRandom number seed (default 42)."},
	{E_TIMEOUT,       0, "t", "timeout",       option::Arg::Required, "  -t <arg> \t--timeout=<arg>  \tSearch time per move, in seconds (default 1)."},
	{E_NPARTICLES,    0, "n", "nparticles",    option::Arg::Required, "  -n <arg> \t--nparticles=<arg>  \tNumber of particles (default 500)."},
	{E_PRUNE,         0, "p", "prune",         option::Arg::Required, "  -p <arg> \t--prune=<arg>  \tPruning constant (default no pruning)."},
	{E_GAP,         0, "", "xi",         option::Arg::Required, "  --xi=<arg>  \tGap constant (default to 0.95)."},
	{E_SIM_LEN,        0, "s", "simlen",        option::Arg::Required, "  -s <arg> \t--simlen=<arg>  \tNumber of steps to simulate. (default 90; 0 = infinite)."},
	{E_SIMULATOR,        0, "", "simulator",        option::Arg::Required, "  \t--simulator=<arg>  \tUse IPPC server or a POMDP model as the simulator."},
	{E_MAX_POLICY_SIM_LEN,        0, "", "max-policy-simlen",        option::Arg::Required, " \t--max-policy-simlen=<arg>  \tNumber of steps to simulate the default policy. (default 90)."},
	{E_DEFAULT_ACTION,0, "", "default-action",   option::Arg::Required, "  --default-action <arg>  \tType of default action to use. (default none)."},
	{E_RUNS,        0, "", "runs",        option::Arg::Required, "  --runs=<arg>  \tNumber of runs. (default 1)."},
	{E_BELIEF,        0, "", "belief",        option::Arg::Required, "  --belief=<arg>  \tBelief type."},
	{E_BLBTYPE,        0, "", "blbtype",        option::Arg::Required, "  -l <arg> \t--blbtype=<arg>  \tBase lower bound, if applicable."},
	{E_LBTYPE,        0, "l", "lbtype",        option::Arg::Required, "  -l <arg> \t--lbtype=<arg>  \tLower bound strategy, if applicable."},
	{E_BUBTYPE,        0, "", "bubtype",        option::Arg::Required, "  -u <arg> \t--bubtype=<arg>  \tBase upper bound, if applicable."},
	{E_UBTYPE,        0, "u", "ubtype",        option::Arg::Required, "  -u <arg> \t--ubtype=<arg>  \tUpper bound strategy, if applicable."},
	{E_BELIEF,        0, "b", "belief",        option::Arg::Required, "  -b <arg> \t--belief=<arg>  \tBelief update strategy, if applicable."},
	{E_VERBOSITY,        0, "v", "verbosity",        option::Arg::Required, "  -v <arg> \t--verbosity=<arg>  \tVerbosity level."},
	{E_SOLVER, 0, "", "solver", option::Arg::Required, " --solver \t"},
	{E_MDP_BOUND_CREATION,        0, "", "MDP-bound-creation",        option::Arg::None, " --MDP-bound-creation \tCompute the MDP default policy and the MDP upper bound (default false)"},
	{E_BOUND_LINK, 0, "", "bound-link", option::Arg::Required, " - <arg>\t--bound-link \tread upper and lower bounds from bound link (default false)."},
	{E_TIME_LIMIT, 0, "", "time-limit", option::Arg::Required, " - <arg>\t--time-limit \ttotal amount of time allowed for the program."},
	{E_NOISE, 0, "", "noise", option::Arg::Required, " - <arg>\t--noise \tnoise level for transition."},
	{0,0,0,0,0,0}
};

DSPOMDP* InitializeModel(string problem, option::Option* options) {
	DSPOMDP* model = NULL;
	if (problem == "bridge") {
		model = new Bridge();
	} else if (problem == "chain") {
		model = !options[E_PARAMS_FILE] ? new Chain() :
			new Chain(options[E_PARAMS_FILE].arg);
	} else if (problem == "lasertag") {
		model = !options[E_PARAMS_FILE] ? new LaserTag() :
			new LaserTag(options[E_PARAMS_FILE].arg);
	} else if (problem == "navigation") {
		model = !options[E_PARAMS_FILE] ? new Navigation() :
			new Navigation(options[E_PARAMS_FILE].arg);
	} else if (problem == "rocksample") {
		if (options[E_PARAMS_FILE]){
			model = new RockSample(options[E_PARAMS_FILE].arg);
		} else {
			int size = 7, number = 8;
			if (options[E_SIZE])
				size = atoi(options[E_SIZE].arg);
			else {
				cerr << "Specify map size using --size option" << endl;
				exit(0);
			}
			if (options[E_NUMBER]) {
				number = atoi(options[E_NUMBER].arg);
			} else {
				cerr << "Specify number of rocks using --number option" << endl;
				exit(0);
			}
			model = new RockSample(size, number);
		}
	} else if (problem == "fvrs") {
		if (options[E_PARAMS_FILE]){
			model = new FVRS(options[E_PARAMS_FILE].arg);
		} else {
			int size = 7, number = 8;
			if (options[E_SIZE])
				size = atoi(options[E_SIZE].arg);
			else {
				cerr << "Specify map size using --size option" << endl;
				exit(0);
			}
			if (options[E_NUMBER]) {
				number = atoi(options[E_NUMBER].arg);
			} else {
				cerr << "Specify number of rocks using --number option" << endl;
				exit(0);
			}
			model = new FVRS(size, number);
		}
	} else if (problem == "pedestrian") {
	} else if (problem == "pocman") {
		model = new FullPocman();
	} else if (problem == "pomdpx") {
		model = new POMDPX(options[E_PARAMS_FILE].arg);
	} else if (problem == "tag") {
		model = !options[E_PARAMS_FILE] ? new Tag() :
			new Tag(options[E_PARAMS_FILE].arg);
	} else if (problem == "tiger") {
		model = new Tiger();
	} else {
		cout << "No model for the problem " << problem << endl;
		cout << "Did you mean one of these: bridge, chain, lasertag, rocksample, pedestrian, pocman, tag, tiger?\n";
		exit(0);
	}
	return model;
}

Solver* InitializeSolver(DSPOMDP* model, string solver_type, option::Option* options) {
	Solver* solver = NULL;
	RandomStreams* streams = NULL;
	if (solver_type == "DESPOT" || solver_type == "DESPOTSTAR" || solver_type == "PLB") {
		streams = new RandomStreams(Seeds::Next(Globals::config.n_particles), Globals::config.search_depth);
		cerr << "Initialized " << streams->NumStreams() << " random streams." << endl;

		if (options[E_BLBTYPE])
			model->InitializeParticleLowerBound(options[E_BLBTYPE].arg);
		cerr << "Base lower bound: " << typeid(*model->particle_lower_bound()).name() << endl;

		if (options[E_LBTYPE])
			model->InitializeScenarioLowerBound(options[E_LBTYPE].arg, *streams);
		cerr << "Lower bound algorithm: " << typeid(*model->scenario_lower_bound()).name() << endl;

		if (options[E_BUBTYPE])
			model->InitializeParticleUpperBound(options[E_BUBTYPE].arg);
		cerr << "Base upper bound: " << typeid(*model->particle_upper_bound()).name() << endl;

		if (options[E_UBTYPE])
			model->InitializeScenarioUpperBound(options[E_UBTYPE].arg, *streams);
		cerr << "Upper bound algorithm: " << typeid(*model->scenario_upper_bound()).name() << endl;
		cerr << endl;

		if (solver_type == "DESPOT")
			solver = new DESPOT(model, NULL, *streams);
		else if (solver_type == "DESPOTSTAR")
			solver = new DESPOTSTAR(model, NULL, *streams);
		else
			solver = model->scenario_lower_bound();
	} else if (solver_type == "AEMS" || solver_type == "BLB") {
		if (options[E_LBTYPE])
			static_cast<BeliefMDP*>(model)->InitializeBeliefLowerBound(options[E_LBTYPE].arg);
		cerr << "Lower bound algorithm: " << typeid(*model->belief_lower_bound()).name() << endl;

		if (options[E_UBTYPE])
			static_cast<BeliefMDP*>(model)->InitializeBeliefUpperBound(options[E_UBTYPE].arg);
		cerr << "Upper bound algorithm: " << typeid(*model->belief_upper_bound()).name() << endl;
		cerr << endl;

		if (solver_type == "AEMS")
			solver = new AEMS(model, NULL);
		else
			solver = model->belief_lower_bound();
	}
	return solver;
}

class Simulator {
public:
	Simulator() {}

	virtual ~Simulator() {}

	virtual int Handshake(string instance) = 0; // Initialize simulator and return number of runs
	virtual void StartRound(State* state) = 0;
	virtual void EndRound() = 0;
	virtual bool ExecuteAction(int action, double& reward, uint64_t& obs, int step) = 0;
	virtual void ReportStepReward() = 0;
	virtual double End() = 0; // Free resources and return total reward collected

	virtual void UpdateTimePerMove(double step_time) = 0;
};

// TODO: Check out whether each instance indeed need 30 runs
class IPPCLog {
private:
	vector<string> runned_instances;
	vector<int> num_of_completed_runs;

public:
	static time_t start_time;
	static const string log_file_;

	static double curr_inst_start_time;
	static double curr_inst_target_time; // Targetted amount of time used for each step
	static double curr_inst_budget; // Total time in seconds given for current instance
	static double curr_inst_remaining_budget; // Remaining time in seconds for current instance
	static int curr_inst_steps;
	static int curr_inst_remaining_steps;
	static double allocated_time;
	static double plan_time_ratio;

	IPPCLog() {
		ifstream fin(log_file_);
		if (!fin.good() || fin.peek() == ifstream::traits_type::eof()) {
			time(&start_time);
		} else {
			fin >> start_time;

			int num_instances;
			fin >> num_instances;
			for (int i = 0; i < num_instances; i ++) {
				string name;
				int num_runs;
				fin >> name >> num_runs;
				runned_instances.push_back(name);
				num_of_completed_runs.push_back(num_runs);
			}
		}
		fin.close();
	}

	void Save() {
		ofstream fout(log_file_);
		fout << start_time << endl;
		fout << runned_instances.size() << endl;
		for (int i = 0; i < runned_instances.size(); i ++)
			fout << runned_instances[i] << " " << num_of_completed_runs[i] << endl;
		fout.close();
	}

	void IncNumOfCompletedRuns(string problem) {
		bool seen = false;
		for (int i = 0; i < runned_instances.size(); i ++) {
			if (runned_instances[i] == problem) {
				num_of_completed_runs[i] ++;
				seen = true;
			}
		}

		if (!seen) {
			runned_instances.push_back(problem);
			num_of_completed_runs.push_back(1);
		}
	}

	int GetNumCompletedRuns() {
		int num = 0;
		for (int i = 0; i < num_of_completed_runs.size(); i ++)
			num += num_of_completed_runs[i];
		return num;
	}

	int GetNumRemainingRuns() {
		return 80 * 30 - GetNumCompletedRuns();
	}

	int GetNumCompletedRuns(string instance) {
		for (int i = 0; i < runned_instances.size(); i ++) {
			if (runned_instances[i] == instance)
				return num_of_completed_runs[i];
		}
		return 0;
	}

	int GetNumRemainingRuns(string instance) {
		return 30 - GetNumCompletedRuns(instance);
	}

	double GetUsedTimeInSeconds() {
		time_t curr;
		time(&curr);
		return (double) (curr - start_time);
	}

	double GetRemainingTimeInSeconds() {
		return 24 * 3600 - GetUsedTimeInSeconds();
	}

	// Pre-condition: curr_inst_start_time is initialized
	void SetInitialBudget(string instance) {
		curr_inst_budget = 0;
		if (GetNumRemainingRuns() != 0 && GetNumRemainingRuns(instance) != 0) {
			cout << "Num of remaining runs: curr / total = " << GetNumRemainingRuns(instance) << " / " << GetNumRemainingRuns() << endl;
			curr_inst_budget = (24 * 3600 - (curr_inst_start_time - start_time)) / GetNumRemainingRuns() * GetNumRemainingRuns(instance);
			if (curr_inst_budget < 0)
				curr_inst_budget = 0;
			if (curr_inst_budget > 18 * 60)
				curr_inst_budget = 18 * 60;
		}
	}

	double GetRemainingBudget(string instance) {
		return curr_inst_budget - (get_time_second() - IPPCLog::curr_inst_start_time);
	}
};
const string IPPCLog::log_file_ = "ippc.log";

time_t IPPCLog::start_time = 0;
double IPPCLog::curr_inst_start_time = 0;
double IPPCLog::curr_inst_target_time = 0;
double IPPCLog::curr_inst_budget = 0;
double IPPCLog::curr_inst_remaining_budget = 0;
int IPPCLog::curr_inst_steps = 0;
int IPPCLog::curr_inst_remaining_steps = 0;
double IPPCLog::allocated_time = 1.0;
double IPPCLog::plan_time_ratio = 1.0;

class IPPCSimulator : public Simulator {
private:
	POMDPX* pomdpx_;
	Client* client_;
	IPPCLog log_;
	string instance_;

public:
	IPPCSimulator(DSPOMDP* model) :
		pomdpx_(static_cast<POMDPX*>(model))
	{
	}

	int Handshake(string instance) {
		int num_remaining_runs = log_.GetNumRemainingRuns(instance);
		if (num_remaining_runs == 0) {
			return 0; 
		}

		double start_t = get_time_second();
		instance_ = instance;

		client_ = new Client();
		ifstream fin("ippc.server_setting");
		int which;
		fin >> which;
		string hostname, port;
		for (int i = 0; i < which; i ++)
			fin >> hostname >> port;
		client_->setHostName(hostname);
		client_->setPort(port);

		client_->initializeSocket();
		client_->connectToServer();

		client_->sendMessage(client_->createSessionRequestMes(instance));

		string sessionInitMes = client_->recvMessage();
		cout << sessionInitMes <<endl;
		client_->processSessionInitMes(sessionInitMes);
		double end_t = get_time_second();
		cout << "Time for handsake " << (end_t - start_t) << endl;

		log_.SetInitialBudget(instance);
		IPPCLog::curr_inst_steps = num_remaining_runs * 40;
		IPPCLog::curr_inst_remaining_steps = num_remaining_runs * 40;
		IPPCLog::curr_inst_target_time = IPPCLog::curr_inst_budget / IPPCLog::curr_inst_steps;
		UpdateTimeInfo(instance);
		IPPCLog::plan_time_ratio = 1.0;
		Globals::config.time_per_move = IPPCLog::plan_time_ratio * IPPCLog::allocated_time;

		return num_remaining_runs;
	}

	void StartRound(State* state) {
		double start_t = get_time_second();

		client_->sendMessage(client_->createRoundRequestMes());
		string roundMes = client_->recvMessageTwice();

		double end_t = get_time_second();
		cout << "Time for startround msg " << (end_t - start_t) << endl;
	}

	void EndRound() {
		double start_t = get_time_second();

		string roundEndMes = client_->recvMessage();
		double roundReward = client_->processRoundEndMes(roundEndMes);
		cout << "Total undiscounted reward = " << roundReward << endl;

		log_.IncNumOfCompletedRuns(instance_);
		log_.Save();

		double end_t = get_time_second();
		cout << "Time for endround msg (save log) " << (end_t - start_t) << endl;
	}

	bool ExecuteAction(int action, double& reward, uint64_t& obs, int step) {
		double start_t = get_time_second();

		client_->sendMessage(client_->createActionMes(pomdpx_->GetActionName(), pomdpx_->GetEnumedAction(action)));

		if (step == Globals::config.sim_len-1) {
			return true;
		}

		string turnMes = client_->recvMessage();
		map<string, string> observs = client_->processTurnMes(turnMes);
		obs = pomdpx_->GetPOMDPXObservation(observs);

		double end_t = get_time_second();
		cout << "Time for executing action " << (end_t - start_t) << endl;

		return false;
	}

	void ReportStepReward() {
	}

	double End() {
		double start_t = get_time_second();

		string sessionEndMes = client_->recvMessage();
		double totalReward = client_->processSessionEndMes(sessionEndMes);
		client_->closeConnection();
		delete client_;

		double end_t = get_time_second();
		cout << "Time for endsession " << (end_t - start_t) << endl;

		return totalReward;
	}

	void UpdateTimeInfo(string instance) {
		IPPCLog::curr_inst_remaining_budget = log_.GetRemainingBudget(instance);
		if (IPPCLog::curr_inst_remaining_budget <= 0) {
			IPPCLog::curr_inst_remaining_budget = 0;
		}

		if (IPPCLog::curr_inst_remaining_steps <= 0) {
			IPPCLog::allocated_time = 0;
		} else {
			IPPCLog::allocated_time = (IPPCLog::curr_inst_remaining_budget - 2.0) / IPPCLog::curr_inst_remaining_steps;

			if (IPPCLog::allocated_time > 5.0)
				IPPCLog::allocated_time = 5.0;
		}
	}

	void UpdateTimePerMove(double step_time) {
		if (step_time < 0.99 * IPPCLog::allocated_time) {
			if (IPPCLog::plan_time_ratio < 1.0)
				IPPCLog::plan_time_ratio += 0.01;
			if (IPPCLog::plan_time_ratio > 1.0)
				IPPCLog::plan_time_ratio = 1.0;
		} else if (step_time > IPPCLog::allocated_time) {
			double delta = (step_time - IPPCLog::allocated_time) / (IPPCLog::allocated_time + 1E-6);
			if (delta < 0.02) delta = 0.02; // Minimum reduction per step
			if (delta > 0.05) delta = 0.05; // Maximum reduction per step
			IPPCLog::plan_time_ratio -= delta;
			if (IPPCLog::plan_time_ratio < 0)
				IPPCLog::plan_time_ratio = 0;
		}

		IPPCLog::curr_inst_remaining_budget = log_.GetRemainingBudget(instance_);
		IPPCLog::curr_inst_remaining_steps --;

		UpdateTimeInfo(instance_);
		Globals::config.time_per_move = IPPCLog::plan_time_ratio * IPPCLog::allocated_time;

		cout << "Time per move set to " << Globals::config.time_per_move << endl;
		cout << "Plan time ratio set to " << IPPCLog::plan_time_ratio << endl;

		cout << "Total time: curr_inst / inst_target / remaining / since_start = "
			<< (get_time_second() - IPPCLog::curr_inst_start_time)
			<< " / " << (IPPCLog::curr_inst_target_time * (IPPCLog::curr_inst_steps - IPPCLog::curr_inst_remaining_steps))
			<< " / " << IPPCLog::curr_inst_remaining_budget
			<< " / " << (get_time_second() - IPPCLog::start_time) << endl;
	}
};

class POMDPSimulator : public Simulator {
private:
	DSPOMDP* model_;
	Random random_;

	double target_finish_time_;

	State* state_;
	double reward_;
	double total_discounted_reward_;
	double total_undiscounted_reward_;

public:
	POMDPSimulator(DSPOMDP* model, Random random, double target_finish_time = -1, int num_steps = -1) :
		model_(model),
		random_(random),
		target_finish_time_(target_finish_time) {
		if (target_finish_time_ != -1) {
			IPPCLog::allocated_time = (target_finish_time_ - get_time_second()) / num_steps;
			Globals::config.time_per_move = IPPCLog::allocated_time;
			IPPCLog::curr_inst_remaining_steps = num_steps;
		}
	}

	int Handshake(string instance) {
		return -1; // Not to be used
	}

	void StartRound(State* state) {
		state_ = state;
		total_discounted_reward_ = 0;
		total_undiscounted_reward_ = 0;
	}

	void EndRound() {
		cout << "Total discounted reward = " << total_discounted_reward_ << endl;
		cout << "Total undiscounted reward = " << total_undiscounted_reward_ << endl;
	}

	bool ExecuteAction(int action, double& reward, uint64_t& obs, int step) {
		double random_num = random_.NextDouble();
		bool terminal = model_->Step(*state_, random_num, action, reward, obs);

		reward_ = reward;
		total_discounted_reward_ += Discount(step) * reward;
		total_undiscounted_reward_ += reward;

		return terminal;
	}

	void ReportStepReward() {
		cout << "- Reward = " << reward_ << endl;
		cout << "- Current rewards:" << endl
			<< "  discounted / undiscounted = "
			<< total_discounted_reward_ << " / " << total_undiscounted_reward_ << endl;
	}

	double End() {
		return 0; // Not to be used
	}

	void UpdateTimePerMove(double step_time) {
		if (target_finish_time_ != -1) {
			if (step_time < 0.99 * IPPCLog::allocated_time) {
				if (IPPCLog::plan_time_ratio < 1.0)
					IPPCLog::plan_time_ratio += 0.01;
				if (IPPCLog::plan_time_ratio > 1.0)
					IPPCLog::plan_time_ratio = 1.0;
			} else if (step_time > IPPCLog::allocated_time) {
				double delta = (step_time - IPPCLog::allocated_time) / (IPPCLog::allocated_time + 1E-6);
				if (delta < 0.02) delta = 0.02; // Minimum reduction per step
				if (delta > 0.05) delta = 0.05; // Maximum reduction per step
				IPPCLog::plan_time_ratio -= delta;
				if (IPPCLog::plan_time_ratio < 0)
					IPPCLog::plan_time_ratio = 0;
			}

			IPPCLog::curr_inst_remaining_budget = target_finish_time_ - get_time_second();
			IPPCLog::curr_inst_remaining_steps --;

			if (IPPCLog::curr_inst_remaining_steps <= 0) {
				IPPCLog::allocated_time = 0;
			} else {
				IPPCLog::allocated_time = (IPPCLog::curr_inst_remaining_budget - 2.0) / IPPCLog::curr_inst_remaining_steps;

				if (IPPCLog::allocated_time > 5.0)
					IPPCLog::allocated_time = 5.0;
			}

			Globals::config.time_per_move = IPPCLog::plan_time_ratio * IPPCLog::allocated_time;

			cout << "Time per move set to " << Globals::config.time_per_move << endl;
			cout << "Plan time ratio set to " << IPPCLog::plan_time_ratio << endl;
		}
	}
};

int main(int argc, char* argv[]) {
	//Parser parser(argv[1]); exit(0);
	// int* ar = 0; ar[0] = 0;
	// TestRandom();
	// TestModel();
	// GenerateRandomInitialConfigs(argc, argv);
	// CompareTagImplementations(argc, argv);

	clock_t main_clock_start = clock();
	IPPCLog::curr_inst_start_time = get_time_second();

	argc -= (argc>0); argv += (argc>0); // skip program name argv[0] if present
	option::Stats stats(usage, argc, argv);
	option::Option* options = new option::Option[stats.options_max];
	option::Option* buffer = new option::Option[stats.buffer_max];
	option::Parser parse(usage, argc, argv, options, buffer);

	// Required parameters
	string problem;
	if (!options[E_PROBLEM]) {
		option::printUsage(std::cout, usage);
		return 0;
	}
	problem = options[E_PROBLEM].arg;

	int num_runs = 1;
	string simulator_type = "pomdp";
	string belief_type = "default";
	int time_limit = -1;

	// Optional parameters
	if (options[E_DEPTH]) Globals::config.search_depth = atoi(options[E_DEPTH].arg);
	if (options[E_DISCOUNT]) Globals::config.discount = atof(options[E_DISCOUNT].arg);
	if (options[E_SEED]) Globals::config.root_seed = atoi(options[E_SEED].arg);
	if (options[E_TIMEOUT]) Globals::config.time_per_move = atof(options[E_TIMEOUT].arg);
	if (options[E_NPARTICLES]) Globals::config.n_particles = atoi(options[E_NPARTICLES].arg);
	if (options[E_PRUNE]) Globals::config.pruning_constant = atof(options[E_PRUNE].arg);
	if (options[E_GAP]) Globals::config.xi = atof(options[E_GAP].arg);
	if (options[E_SIM_LEN]) Globals::config.sim_len = atoi(options[E_SIM_LEN].arg);
	if (options[E_SIMULATOR]) simulator_type = options[E_SIMULATOR].arg;
	if (options[E_MAX_POLICY_SIM_LEN]) Globals::config.max_policy_sim_len = atoi(options[E_MAX_POLICY_SIM_LEN].arg);
	if (options[E_DEFAULT_ACTION]) {
		Globals::config.default_action = options[E_DEFAULT_ACTION].arg;
		cerr << "Default action = " << Globals::config.default_action << endl;
	}
	if (options[E_RUNS]) num_runs = atoi(options[E_RUNS].arg);
	if (options[E_BELIEF]) belief_type = options[E_BELIEF].arg;

	if (options[E_MDP_BOUND_CREATION]) Globals::config.create_mdp_bound = true;
	if (options[E_BOUND_LINK]) {
		Globals::config.bound_file = options[E_BOUND_LINK].arg;
	}
	if (options[E_TIME_LIMIT]) {
		time_limit = atoi(options[E_TIME_LIMIT].arg);
		cout << "Time limit = " << time_limit << "s" << endl;
	}
	if (options[E_NOISE])
		Globals::config.noise = atof(options[E_NOISE].arg);

	string solver_type = "DESPOTSTAR";
	if (options[E_SOLVER]) solver_type = options[E_SOLVER].arg;

	int verbosity = 0;
	if (options[E_VERBOSITY]) verbosity = atoi(options[E_VERBOSITY].arg);
	logging::level(verbosity);

	cerr << "Regularization constant: " << Globals::config.pruning_constant << endl;

	Seeds::root_seed(Globals::config.root_seed);
	cerr << "Random root seed set to " << Globals::config.root_seed << endl;

	// World simulator random seed
	unsigned seed = Seeds::Next();
	Random random(seed);
	cerr << "Initialized world random generator with seed " << seed << endl;

	// Global random generator
	seed = Seeds::Next();
	Random::RANDOM = Random(seed);
	cerr << "Initialized global random generator with seed " << seed << endl;

	// Model
	DSPOMDP* model = InitializeModel(problem, options);
	cerr << "Initialized " << problem << " model." << endl;
	model->num_active_particles = 0;

	// Solver
	Solver* solver = InitializeSolver(model, solver_type, options);
	assert(solver != NULL);
	cerr << "Solver: " << typeid(*solver).name() << endl << endl;

	// Simulator
	Simulator* simulator = NULL;
	if (simulator_type == "ippc") {
		simulator = new IPPCSimulator(model);
	} else {
		if (time_limit != -1) {
			simulator = new POMDPSimulator(model, random, IPPCLog::curr_inst_start_time + time_limit, num_runs * Globals::config.sim_len);
		} else {
			simulator = new POMDPSimulator(model, random);
		}
	}
	cerr << "Simulator: " << typeid(*simulator).name() << endl;

	if (simulator_type == "ippc") {
		IPPCLog log;
		log.Save(); // Initialize log file
		cout << "Time: elapsed_since_start / total_remaining = " << (IPPCLog::curr_inst_start_time - IPPCLog::start_time)
			<< " / " << (24 * 3600 - (IPPCLog::curr_inst_start_time - IPPCLog::start_time)) << endl;

		string input_file(options[E_PARAMS_FILE].arg);
		int seconddot = input_file.find_last_of('.');
		int firstdot = input_file.substr(0, seconddot).find_last_of('/');
		string instance = input_file.substr(firstdot + 1, seconddot - firstdot - 1);

		num_runs = simulator->Handshake(instance);
		if (num_runs == 0) {
			cout << "Finished running instance. Exit." << endl;
			exit(0);
		}
		cout << num_runs << " runs to be completed in " << IPPCLog::curr_inst_budget << " seconds" << endl;
	}

	// Run num_runs simulations
	double start_t, end_t;
	for (int r = 0; r < num_runs; r ++) {
		// Initial state
		State* state = model->CreateStartState();
		if (simulator_type != "ippc") {
			cout << "Initial state:" << endl;
			model->PrintState(*state);
			cout << endl;
		}

		// Initial belief
		start_t = get_time_second();
		delete solver->belief();
		end_t = get_time_second();
		cout << "Time for deleting belief " << (end_t - start_t) << endl;

		start_t = get_time_second();
		Belief* belief = model->InitialBelief(state, belief_type);
		cerr << "Initialized initial belief: " << typeid(*belief).name() << endl;
		end_t = get_time_second();
		cout << "Time for init prior " << (end_t - start_t) << endl;

		solver->belief(belief);

		uint64_t obs;
		double reward;
		int step = 0;

		simulator->StartRound(state);
		cout << "Time per move set to " << Globals::config.time_per_move << endl;

		// Run sim_len steps 
		double step_start_t, step_end_t;
		for(int i = 0; i < Globals::config.sim_len; i ++) {
			if (time_limit != -1 && get_time_second() - IPPCLog::curr_inst_start_time > time_limit) {
				cerr << "Exit. (Total time " << (get_time_second() - IPPCLog::curr_inst_start_time) << "s exceeded time limit of " << time_limit << "s)" << endl;
				cout << "Total time: Real / CPU = " << (get_time_second() - IPPCLog::curr_inst_start_time) << " / " << (double (clock() - main_clock_start) / CLOCKS_PER_SEC) << "s" << endl;
				exit(1);
			}

			step_start_t = get_time_second();
			cout << "Step " << i << endl;

			start_t = get_time_second();
			int action = solver->Search();
			end_t = get_time_second();

			bool terminal = simulator->ExecuteAction(action, reward, obs, i);

			step ++;

			start_t = get_time_second();
			cout << "- Action = "; model->PrintAction(action);

			if (simulator_type != "ippc") {
				cout << "- State:\n"; model->PrintState(*state);
			}

			cout << "- Observation = "; model->PrintObs(*state, obs);
			cout << "- ObsProb = " << model->ObsProb(obs, *state, action) << endl;

			simulator->ReportStepReward();
			end_t = get_time_second();

			if(terminal) {
				step_end_t = get_time_second();
				cout << "Time for step: actual / allocated = " << (step_end_t - step_start_t)
					<< " / " << (IPPCLog::curr_inst_remaining_budget / IPPCLog::curr_inst_remaining_steps) << endl;
				simulator->UpdateTimePerMove(step_end_t - step_start_t);
				cout << endl;
				break;
			}

			start_t = get_time_second();
			solver->Update(action, obs);
			end_t = get_time_second();
			cout << "Time for belief update " << (end_t - start_t) << endl;

			step_end_t = get_time_second();
			cout << "Time for step: actual / allocated = " << (step_end_t - step_start_t)
				<< " / " << IPPCLog::allocated_time << endl;
			simulator->UpdateTimePerMove(step_end_t - step_start_t);
			cout << endl;
		}

		cout << "Simulation terminated in " << step << " steps" << endl;
		simulator->EndRound();
		cout << endl;
	}

	if (simulator_type == "ippc" && num_runs != 30) {
		cout << "Exit without receiving reward." <<endl;
		cout << "Total time: Real / CPU = " << (get_time_second() - IPPCLog::curr_inst_start_time) << " / " << (double (clock() - main_clock_start) / CLOCKS_PER_SEC) << "s" << endl;
		return 0;
	}

	double total_reward = simulator->End();
	if (simulator_type == "ippc") {
		cout << "Total reward for all runs = " << total_reward << endl;

		cout << "Total time: Real / CPU = " << (get_time_second() - IPPCLog::curr_inst_start_time) << " / " << (double (clock() - main_clock_start) / CLOCKS_PER_SEC) << "s" << endl;
	}

	cout << "Exit. Total time: Real / CPU = " << (get_time_second() - IPPCLog::curr_inst_start_time) << " / " << (double (clock() - main_clock_start) / CLOCKS_PER_SEC) << "s" << endl;
	return 0;
}
