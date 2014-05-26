#include "policy.h"
#include "pomdp.h"
#include <unistd.h>
#include <thread>

Policy::Policy(const DSPOMDP* model, Belief* belief)
	: ScenarioLowerBound(model, belief) {
}

Policy::~Policy() {}

ValuedAction Policy::Value(const vector<State*>& particles) const {
	return model_->LowerBound(particles);
}

ValuedAction Policy::Value(const vector<State*>& particles,
		RandomStreams& streams, History& history) const {

	// if (seen.find(history) != seen.end()) {
		// return seen[history];
	// }

	vector<State*> copy;
	for (auto& particle : particles)
		copy.push_back(model_->Copy(particle));

	initial_depth_ = history.Size();
	ValuedAction va = RecursiveValue(copy, streams, history);

	for (auto& particle : copy)
		model_->Free(particle);

	return va;
}

ValuedAction Policy::RecursiveValue(const vector<State*>& particles,
		RandomStreams& streams, History& history) const {
	if (streams.Exhausted() || (history.Size() - initial_depth_ >= Globals::config.max_policy_sim_len)) {
		// cout << "simulated until depth " << (history.Size() - initial_depth_) << endl;
		return Value(particles);
	} else {
		int action = Action(particles, streams, history);

		double value = 0;

		map<uint64_t, vector<State*>> partitions;
		uint64_t obs;
		double reward;
		for (auto& particle : particles) {
			bool terminal = model_->Step(*particle,
					streams.Entry(particle->scenario_id), action, reward, obs);

			value += reward * particle->weight;

			if (!terminal) {
				partitions[obs].push_back(particle);
			}
		}

		if (partitions.size() == 0) {
			// cout << "simulated until depth " << (history.Size() - initial_depth_) << endl;
		}

		for (auto& it : partitions) {
			uint64_t obs = it.first;
			history.Add(action, obs);
			streams.Advance();
			ValuedAction va = RecursiveValue(it.second, streams, history);
			// seen[history] = va;
			value += Discount() * va.value;
			streams.Back();
			history.RemoveLast();
		}

		return ValuedAction(action, value);
	}
}

void Policy::Reset() {
	seen.clear();
}

int Policy::Search() {
	RandomStreams streams(Seeds::Next(Globals::config.n_particles), Globals::config.search_depth);
	vector<State*> particles = belief_->Sample(Globals::config.n_particles);

	int action = Action(particles, streams, history_);

	for (State* particle : particles)
		model_->Free(particle);

	return action;
}

/*---------------------------------------------------------------------------*/

BlindPolicy::BlindPolicy(const DSPOMDP* model, int action, Belief* belief)
	: Policy(model, belief), action_(action)
{
}

int BlindPolicy::Action(const vector<State*>& particles,
		RandomStreams& streams, History& history) const {
	return action_;
}

int BlindPolicy::Search() {
	return action_;
}

void BlindPolicy::Update(int action, uint64_t obs) {
}

/*---------------------------------------------------------------------------*/

RandomPolicy::RandomPolicy(const DSPOMDP* model, Belief* belief) : Policy(model, belief) {
}

RandomPolicy::RandomPolicy(const DSPOMDP* model, const vector<double>& action_probs, Belief* belief)
	: Policy(model, belief), action_probs_(action_probs) {
	double sum = 0;
	for (int i=0; i<action_probs.size(); i++)
		sum += action_probs[i];
	assert(fabs(sum - 1.0) < 1.0e-8);
}

int RandomPolicy::Action(const vector<State*>& particles,
		RandomStreams& streams, History& history) const {
	if (action_probs_.size() > 0) {
		return Random::GetCategory(action_probs_, Random::RANDOM.NextDouble());
	} else {
		return Random::RANDOM.NextInt(model_->NumActions());
	}
}

int RandomPolicy::Search() {
	if (action_probs_.size() > 0) {
		return Random::GetCategory(action_probs_, Random::RANDOM.NextDouble());
	} else {
		return Random::RANDOM.NextInt(model_->NumActions());
	}
}

void RandomPolicy::Update(int action, uint64_t obs) {
}

/*---------------------------------------------------------------------------*/
MajorityActionPolicy::MajorityActionPolicy(const DSPOMDP* model, const StatePolicy& policy, Belief* belief)
	: Policy(model, belief),
	policy_(policy) {
}

int MajorityActionPolicy::Action(const vector<State*>& particles,
		RandomStreams& streams, History& history) const {
	vector<double> frequencies(model_->NumActions());

	for (auto& particle : particles) {
		int action = policy_.GetAction(*particle);
		frequencies[action] += particle->weight;
	}

	int bestAction = 0;
	double bestWeight = frequencies[0];
	for (int a=1; a<frequencies.size(); a++) {
		if (bestWeight < frequencies[a]) {
			bestWeight = frequencies[a];
			bestAction = a;
		}
	}

	return bestAction;
}

/*---------------------------------------------------------------------------*/
ModeStatePolicy::ModeStatePolicy(const DSPOMDP* model, const StateIndexer& indexer, const StatePolicy& policy, Belief* belief)
	: Policy(model, belief),
	indexer_(indexer),
	policy_(policy) {
		state_probs_.resize(indexer_.NumStates());
}

int ModeStatePolicy::Action(const vector<State*>& particles,
		RandomStreams& streams, History& history) const {
	double maxWeight = 0;
	State* mode = NULL;
	for (auto& particle : particles) {
		int id = indexer_.GetIndex(particle);
		state_probs_[id] += particle->weight;

		if (state_probs_[id] > maxWeight) {
			maxWeight = state_probs_[id];
			mode = particle;
		}
	}

	for (auto& particle : particles) {
		state_probs_[indexer_.GetIndex(particle)] = 0;
	}

	assert(mode != NULL);
	return policy_.GetAction(*mode);
}

/*---------------------------------------------------------------------------*/

MMAPStatePolicy::MMAPStatePolicy(const DSPOMDP* model, const MMAPInferencer& inferencer,
		const StatePolicy& policy, Belief* belief) :
	Policy(model),
	inferencer_(inferencer), 
	policy_(policy) {
}

int MMAPStatePolicy::Action(const vector<State*>& particles,
		RandomStreams& streams, History& history) const {
	return policy_.GetAction(*inferencer_.GetMMAP(particles));
}

/*---------------------------------------------------------------------------*/

/*
void DESPOT::RetrieveHistoryActionMapping(SuffixPolicyLowerBound* suffix_policy) {
	History history = history_;
	RetrieveHistoryActionMapping(root_, history, suffix_policy);
}

void DESPOT::RetrieveHistoryActionMapping(VNode& node,
		History& history,
		SuffixPolicyLowerBound* suffix_policy) {
	if (!node->in_tree()) return;

	// The history should be long enough so that current belief does not depend
	// much on the initial belief.
	// TODO: may need to use a longer history
	if (history.Size() > 0 && node->best_ub_action() != -1)
		suffix_policy->Put(history, node->OptimalAction()); 

  vector<QNode>& qnodes = node->Children();

  int n_actions = model_.NumActions();

  for (int action = 0; action < n_actions; action++) {
		QNode& qnode = qnodes[action];
		vector<uint64_t> labels = qnode.BranchLabels();

		for (int i=0; i<labels.size(); i++) {
			history.Add(action, labels[i]);
			RetrieveHistoryActionMapping(qnode.Belief(labels[i]), history,
					suffix_policy);
			history.RemoveLast();
		}
  }
}
*/

SuffixPolicy::SuffixPolicy(const DSPOMDP* model, Belief* belief) : Policy(model, belief) {
	//default_policy = new ModePolicyLowerBound<T>(streams, num_states);
	defaultCount = 0;
	suffixCount = 0;
}

int SuffixPolicy::Action(const vector<State*>& particles,
		RandomStreams& streams, History& history) const {
	vector<int> actions = SearchForLongestSuffixMatchActions(history);
	if (actions.size() == 0) {
		defaultCount ++;
		return 0; // TODO
		//return default_policy->Action(particles, model, history);
		//return rand_r(&action_root_seed_) % model.NumActions();
	}

	suffixCount ++;
	return actions[rand_r(&action_root_seed_) % actions.size()];
}

void SuffixPolicy::Learn(VNode* tree) {
	/*
	cout << "Clearing previously stored traces...";
	Clear();
	cout << "Done." << endl;

	cout << "DefaultCount = " << defaultCount << endl;
	cout << "SuffixCount = " << suffixCount << endl;

	cout << "Retrieving history-action mapping...";
	solver->RetrieveHistoryActionMapping(static_cast<SuffixPolicyLowerBound<T>*>(this));
	cout << Count() << " pairs are retrieved." << endl;

	cout << "Constructing suffix tree...";
	ConstructMapping();
	cout << "Done." << endl;
	*/
}

// SuffixPolicy* Instance(string name, RandomStreams& streams, int num_states, unsigned action_root_seed);

/*
class SuffixPolicySuffixTreeImpl : public SuffixPolicy {
private:
	SSTree* policy;
	vector<uchar*> runs;
	mutable unsigned action_root_seed_;

	const static int CODE_LENGTH = 16;
	const static char OBS_PREFIX = 'O';
	const static char ACTION_PREFIX = 'A';
	const static int BASE = 26;
	const static char FIRST = 'a';

	static void ObsToString(uint64_t obs, uchar* buf, int pos) {
		buf[pos] = OBS_PREFIX;
		for (int i=1; i<CODE_LENGTH; i++) {
			buf[pos+i] = FIRST + obs%BASE;
			obs /= BASE;
		}
	}
	static void ActionToString(int action, uchar* buf, int pos) {
		buf[pos] = ACTION_PREFIX;
		for (int i=1; i<CODE_LENGTH; i++) {
			buf[pos+i] = FIRST + action%BASE;
			action /= BASE;
		}
	}
	static int StringToAction(const uchar* buf) {
		int action = 0,
				length = strlen((char*) buf),
				pow = 1;
		for (int i=0; i<length; i++) {
			if (buf[i] == '$')
				break;

			action += (buf[i] - FIRST) * pow;
			pow *= BASE;
		}
		return action;
	}

	static uchar* toString(const History& history, int action) {
		return toString(history, action, 0);
	}
	static uchar* toString(const History& history, int action, int start) {
		uchar* buf = new uchar[CODE_LENGTH *
			(2 * (history.Size() - start)
			 + (action == -1 ? 0 : 1))
			+ (action == -1 ? 2 : 1)];
		int pos = 0;
		for (int i=start; i<history.Size(); i++) {
			ActionToString(history.Action(i), buf, pos);
			pos += CODE_LENGTH;
			ObsToString(history.Observation(i), buf, pos);
			pos += CODE_LENGTH;
		}
		if (action != -1) {
			ActionToString(action, buf, pos);
			buf[pos] = 'R';
			pos += CODE_LENGTH;
		} else {
			buf[pos] = 'R';
			pos ++;
		}
		buf[pos] = '\0';
		return buf;
	}

	static void Copy(uchar* from, uchar* to, int& pos) {
		int length = strlen((char*) from);
		for (int i=0; i<length; i++)
			to[pos++] = from[i];
	}

	static bool Matches(const uchar* text, const uchar* pattern, int& pos) {
		int length = strlen((const char*)pattern);

		if (pos + length > (int)strlen((const char*) text))
			return false;

		for (int i=0; i<length; i++)
			if (text[pos++] != pattern[i])
				return false;
		return true;
	}

	static ulong Search(SSTree* sst, uchar* str) {
		ulong node = sst->root();
		int pos = 0, length = strlen((char*) str);
		while(true) {
			node = sst->child(node, str[pos]);

			if (node == 0) // no match
				return 0;

			uchar* label = sst->edge(node);

			if (!Matches(str, label, pos))
				break;
			if (pos == length)
				return node;
		}
		return 0;
	}

public:
	SuffixPolicyLowerBoundSuffixTreeImpl(RandomStreams& streams, int num_states, unsigned action_root_seed) 
		: SuffixPolicyLowerBound<T>(streams, num_states),
		action_root_seed_(action_root_seed) {
			policy = NULL;
	}

  static string Name() { return "suffix"; }

public:
	void Clear() {
		runs.clear();
	}

	int Count() { return runs.size(); }

	void Put(const History& history, int action) {
		runs.push_back(toString(history, action));
	}

	void ConstructMapping() {
		if (runs.size() == 0) {
			policy = NULL;
			return;
		}

		int length = 0;
		for (unsigned int i=0; i<runs.size(); i++)
			length += strlen((char*)runs[i]) + 1;
		uchar* runsStr = new uchar[length];
		int pos = 0;
		for (unsigned int i=0; i<runs.size(); i++) {
			Copy(runs[i], runsStr, pos);
			runsStr[pos] = '$';
			pos++;
		}
		runsStr[pos-1] = '\0';

		// cout << "Length: " << length << " " << pos << endl;
		// cout << "RunsString: " << runsStr << endl;

		policy = new SSTree(runsStr, pos);

		// policy->PrintTree(policy->root(), 0);
	}

	vector<int> SearchForLongestSuffixMatchActions(const History& history) const {
		vector<int> actions;

		if (policy == NULL) return actions;

		for (int i=0; i<history.Size(); i++) {
			uchar* str = toString(history, -1, i);
			// cout << "Query: " << str << endl;
			ulong match = Search(static_cast<SSTree*>(policy), str);
			// cout << "Match: " << match << " " << policy->pathlabel(match) << endl;

			if (match != 0) {
				ulong child = policy->firstChild(match);

				while(child != 0) {
					actions.push_back(StringToAction(policy->edge(child)));

					child = policy->sibling(child);
				}

				break;
			}
		}

		return actions;
	}
};
*/


SuffixPolicyMapImpl::SuffixPolicyMapImpl(const DSPOMDP* model, Belief* belief) 
		: SuffixPolicy(model, belief),
		count(0) {
	}

string SuffixPolicyMapImpl::Name() { return "suffix"; }

void SuffixPolicyMapImpl::Clear()  {
	suffix_action_map.clear();
	count = 0;
}

int SuffixPolicyMapImpl::Count() { return count; }

void SuffixPolicyMapImpl::Put(const History& history, int action) {
	count ++;
	for (int i=0; i<history.Size()-1; i++)
		suffix_action_map[history.Suffix(i)].push_back(action);
}

void SuffixPolicyMapImpl::ConstructMapping() {
}

vector<int> SuffixPolicyMapImpl::SearchForLongestSuffixMatchActions(const History& history) const {
	if (suffix_action_map.size() == 0)
		return vector<int>();

	for (int i=0; i<history.Size()-10; i++) {
		map<History, vector<int>>::const_iterator iter = suffix_action_map.find(history.Suffix(i));
		if (iter != suffix_action_map.end())
			return iter->second;
	}

	return vector<int>();
}

SuffixPolicy* SuffixPolicy::Instance(string name, const DSPOMDP* model, Belief* belief) {
	//if (name == "suffix_tree") return new SuffixPolicyLowerBoundMapImpl<T>(streams, num_states, action_root_seed);

	return new SuffixPolicyMapImpl(model, belief);
}
