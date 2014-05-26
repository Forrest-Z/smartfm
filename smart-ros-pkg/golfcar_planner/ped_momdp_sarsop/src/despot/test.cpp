#include "test.h"

void TestRandom() {
	Random random((unsigned)1);
	cout << "10 RVs from U[0, 1]:" << endl;
	for(int i=0; i<10; i++)
		cout << " " << random.NextDouble() << endl;

	cout << "10 RVs from U[10, 12]:" << endl;
	for(int i=0; i<10; i++)
		cout << " " << random.NextDouble(10, 12) << endl;

	vector<double> category_probs = {0.1, 0.4, 0.5};
	const int N = 1000;
	cout << "Distribution of " << N << " categories from (0.1, 0.4, 0.5): " << endl;
	vector<double> counts(category_probs.size());
	for(int i=0; i<N; i++)
		counts[random.NextCategory(category_probs)] ++;
	for(int i=0; i<counts.size(); i++)
		cout << " " << counts[i] << endl;

	vector<int> fre(1000);
	for(int i=0; i<10000000; i++) {
		Random r(random.NextDouble());
		fre[(int) (r.NextDouble() / 0.001)] ++;
	}
	double l1 = 0;
	for (int i=0; i<1000; i++) {
		l1 += fabs(0.001 - fre[i] / 10000000.0);
		cout << fre[i] << endl;
	}
	cout << l1 << endl;
}

void TestTree() {
	DSPOMDP* model = new Bridge();
	logi("Initialized bridge model.");

	vector<State*> particles = {BridgeState(0)(0, 0.2),
		BridgeState(0)(1, 0.2),
		BridgeState(1)(2, 0.2),
		BridgeState(1)(3, 0.2),
		BridgeState(1)(4, 0.2)
	};

	BridgeState* state = new BridgeState(0);
	Belief* belief = model->InitialBelief(state);
	particles = belief->Sample(10);

	logi("Constructed ", particles.size(), " particles:");
	for(int i=0; i<particles.size(); i++) {
		logi(" ", i, " = ", *particles[i]);
	}

	RandomStreams streams(Seeds::Next(particles.size()), 90);
	logi("Initialized random streams.");

	//Globals::config.discount = 0.8;

	VNode* root = new VNode(move(particles), 0);
	History history;
	DESPOT::InitLowerBound(root, model, streams, history);
	DESPOT::InitUpperBound(root, model, streams, history);
	root->PrintTree();
	cout << endl;

	for (int i=0; i<1000; i++) {
		VNode* promising_node = root->vstar;
		if(promising_node != NULL && root->upper_bound() - root->lower_bound() > 10) {
			DESPOT::Expand(promising_node, model, streams, history);
			DESPOT::Backup(promising_node);
			// root->PrintTree();
			// cout << endl;
		}
	}
	root->PrintTree();
}

void SimulateModel(DSPOMDP * model, ostream& os) {
	Seeds::root_seed(0);
	RandomStreams streams(Seeds::Next(Globals::config.n_particles), 500);
	Random random(Seeds::Next());
	Random::RANDOM = Random(Seeds::Next());
	os << streams << endl;

	State* start = model->CreateStartState();
	Belief* belief = model->InitialBelief(start);
	History history;
	vector<State*> particles = belief->Sample(500);
	cout << model->LowerBound(particles, streams, history) << endl;
	cout << model->UpperBound(particles, streams, history) << endl;

	particles.resize(0);
	int count = 0;
	for(State* state : particles) {
		while(true) {
			double reward;
			uint64_t obs;
			int action = random.NextInt(model->NumActions());
			double rand_num = random.NextDouble();
			bool terminal = model->Step(*state, rand_num, action , reward, obs);
			os << "- Random number = " << rand_num << endl;
			os << "- Action = " << action << endl;
			os << "- State:\n";
			model->PrintState(*state, os);
			os << "- Reward = " << reward << endl;
			os << "- Observation = ";
			model->PrintObs(*state, obs, os);
			os << endl;
			os << "- ObsProb = " << model->ObsProb(obs, *state, action) << endl;

			if(terminal) break;
		}
		os << "OK " << (count ++) << endl;
	}
}

void CompareAglos() {
}

/*
void CompareTagImplementations(int argc, char* argv[]) {
	string param_file = argv[1];
	ofstream fout1(argv[2]);
	SimulateModel(new OldTag(param_file), fout1);
	fout1.close();
	cout << "separator" << endl;
	ofstream fout2(argv[3]);
	SimulateModel(new Tag(param_file), fout2);
	fout2.close();
	exit(0);
}
*/

void GenerateRandomInitialConfigs(int argc, char* argv[]) {
	Tag* model = new Tag("tag.params");
	for(int i=1; i<=32000; i++) {
		string fn = concat("map", i);
		ofstream fout(fn);
		fout << "mapSize = 5 10" << endl; 
		State* start = model->CreateStartState();
		model->PrintState(*start, fout);
		fout.close();
	}
}

void TestModel() {
	RandomStreams streams(Seeds::Next(1), 90);
	DSPOMDP* model = new LaserTag("lasertag.param");
	State* start = model->CreateStartState();
	model->InitializeScenarioUpperBound("MANHATTAN", streams);
	model->InitializeScenarioLowerBound("SMART", streams);
	ScenarioLowerBound* policy = model->scenario_lower_bound();
	History history;

	model->PrintState(*start);
	cout << endl;

	vector<State*> particles;
	particles.push_back(model->Copy(start));
	particles[0]->weight = 1.0;
	//particles[0].
	cout << "OK " << particles[0]->IsAllocated() << endl;

	double reward;
	uint64_t obs;

	for (int i=0; i<90; i++) {
		ValuedAction va = policy->Value(particles, streams, history);
		bool terminal = model->Step(*particles[0], 0.4, va.action, reward, obs);
		model->PrintState(*particles[0]);
		cout << va.action << endl;
		cout << endl;

		if (terminal) break;

		history.Add(va.action, obs);
	}

	/*
	vector<int> actions = {0, 1, 2, 3, 4};
	vector<double> nums = {0.1, 0.1, 0.2, 0.1, 0.1};

	for (int i=0; i<actions.size(); i++) {
		int action = actions[i];
		double rand_num = nums[i];
		model->Step(*start, rand_num, action, reward, obs);
		model->PrintState(*start);
		cout << reward << endl;
		model->PrintObs(*start, obs);
		cout << model->ObsProb(obs, *start, action) << endl;
		cout << endl;
	}
	*/
}

void TestDespot() {
	// DSPOMDP* model = new Bridge("bridge.txt");
	// logi("Initialized bridge model.");
	// BridgeState state(0);

	DSPOMDP* model = new Tiger();
	logi("Initialized tiger model.");
	TigerState* state = new TigerState(Random::RANDOM.NextInt(2));

	Belief* belief = model->InitialBelief(state);

	RandomStreams streams(Seeds::Next(Globals::config.n_particles), Globals::config.sim_len);
	logi("Initialized ", streams.NumStreams(), " random streams.");

	Random random(Seeds::Next());

	model->InitializeScenarioLowerBound("RANDOM", streams);
	// Policy* policy = new BlindPolicy(*model, 0);
	// ScenarioLowerBound* lower_bound = policy;

	Solver* solver = new DESPOT(model, belief, streams);
	uint64_t obs;
	double reward;
	double total_discounted_reward = 0;
	double total_undiscounted_reward = 0;
	double discount = 1.0;
	int step = 0;
	for(int i=0; i<Globals::config.sim_len; i++) {
		int action = solver->Search();
		int terminal = model->Step(*state, random.NextDouble(), action, reward, obs);

		total_discounted_reward += discount * reward;
		discount *= Globals::config.discount;

		total_undiscounted_reward += reward;
		step ++;

		cout << "Step " << i << endl;
		cout << "- Action = " << action << endl;
		cout << "- State:\n";
		model->PrintState(*state);
		cout << "- Reward = " << reward << endl;
		cout << "- Observation = ";
		model->PrintObs(*state, obs);
		cout << endl;

		if(terminal)
			break;

		solver->Update(action, obs);
	}

	cout << "Simulation terminated in " << step << " steps" << endl;
	cout << "Total discounted reward =" << total_discounted_reward << endl;
	cout << "Total undiscounted reward = " << total_undiscounted_reward << endl;
}


