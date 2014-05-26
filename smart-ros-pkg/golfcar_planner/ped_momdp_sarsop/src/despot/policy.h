#ifndef POLICY_H
#define POLICY_H

#include <vector>

#include "random_streams.h"
#include "lower_bound.h"
#include "util/random.h"
#include "history.h"

//#include "SSTree.h"
//#include "Tools.h"
#include "string.h"
#include <queue>
#include <vector>
#include <stdlib.h>
#include "globals.h"

using namespace std;

class State;
class StateIndexer;
class StatePolicy;
class DSPOMDP;
class MMAPInferencer;

/*---------------------------------------------------------------------------*/

class Policy : public ScenarioLowerBound {
private:
	mutable map<History, ValuedAction> seen;
	mutable int initial_depth_;

	ValuedAction RecursiveValue(const vector<State*>& particles,
			RandomStreams& streams, History& history) const;

public:
	Policy(const DSPOMDP* model, Belief* belief = NULL);
	virtual ~Policy();

	void Reset();
	virtual int Action(const vector<State*>& particles,
			RandomStreams& streams, History& history) const = 0;

	virtual ValuedAction Value(const vector<State*>& particles) const;

	ValuedAction Value(const vector<State*>& particles,
			RandomStreams& streams, History& history) const;

	virtual int Search();
};

/*---------------------------------------------------------------------------*/
class BlindPolicy : public Policy {
private:
	int action_;

public:
	BlindPolicy(const DSPOMDP* model, int action, Belief* belief = NULL);

	int Action(const vector<State*>& particles,
			RandomStreams& streams, History& history) const;

	int Search();
	void Update(int action, uint64_t obs);
};

/*---------------------------------------------------------------------------*/
class RandomPolicy : public Policy {
private:
	vector<double> action_probs_;

public:
	RandomPolicy(const DSPOMDP* model, Belief* belief = NULL);
	RandomPolicy(const DSPOMDP* model, const vector<double>& action_probs, Belief* belief = NULL);

	int Action(const vector<State*>& particles,
			RandomStreams& streams, History& history) const;

	int Search();
	void Update(int action, uint64_t obs);
};

/*---------------------------------------------------------------------------*/
class ModeStatePolicy : public Policy {
private:
	const StateIndexer& indexer_;
	const StatePolicy& policy_;
	mutable vector<double> state_probs_;

public:
	ModeStatePolicy(const DSPOMDP* model, const StateIndexer& indexer, const StatePolicy& policy, Belief* belief = NULL);

	int Action(const vector<State*>& particles,
			RandomStreams& streams, History& history) const;
};

/*---------------------------------------------------------------------------*/
class MMAPStatePolicy : public Policy { // Marginal MAP state policy
private:
	const MMAPInferencer& inferencer_;
	const StatePolicy& policy_;

public:
	MMAPStatePolicy(const DSPOMDP* model, const MMAPInferencer& inferencer,
			const StatePolicy& policy, Belief* belief = NULL);

	int Action(const vector<State*>& particles, RandomStreams& streams,
			History& history) const;
};

/*---------------------------------------------------------------------------*/
class MajorityActionPolicy : public Policy {
private:
	const StatePolicy& policy_;

public:
	MajorityActionPolicy(const DSPOMDP* model, const StatePolicy& policy, Belief* belief = NULL);

	int Action(const vector<State*>& particles,
			RandomStreams& streams, History& history) const;
};

/*---------------------------------------------------------------------------*/

class SuffixPolicy : public Policy {
protected:
	mutable unsigned action_root_seed_;
	ScenarioLowerBound* default_policy;
	mutable int defaultCount;
	mutable int suffixCount;

public:
	SuffixPolicy(const DSPOMDP* model, Belief* belief = NULL);

  int Action(const vector<State*>& particles,
			RandomStreams& streams, History& history) const;

	void Learn(VNode* root);

	static SuffixPolicy* Instance(string name, const DSPOMDP* model, Belief* belief = NULL);

	virtual int Count() = 0;
	virtual void Clear() = 0;
	virtual void Put(const History& history, int action) = 0;
	virtual void ConstructMapping() = 0;
	virtual vector<int> SearchForLongestSuffixMatchActions(const History& history) const = 0;
};

/*---------------------------------------------------------------------------*/
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
		for(int i=1; i<CODE_LENGTH; i++) {
			buf[pos+i] = FIRST + obs%BASE;
			obs /= BASE;
		}
	}
	static void ActionToString(int action, uchar* buf, int pos) {
		buf[pos] = ACTION_PREFIX;
		for(int i=1; i<CODE_LENGTH; i++) {
			buf[pos+i] = FIRST + action%BASE;
			action /= BASE;
		}
	}
	static int StringToAction(const uchar* buf) {
		int action = 0,
				length = strlen((char*) buf),
				pow = 1;
		for(int i=0; i<length; i++) {
			if(buf[i] == '$')
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
		for(int i=start; i<history.Size(); i++) {
			ActionToString(history.Action(i), buf, pos);
			pos += CODE_LENGTH;
			ObsToString(history.Observation(i), buf, pos);
			pos += CODE_LENGTH;
		}
		if(action != -1) {
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
		for(int i=0; i<length; i++)
			to[pos++] = from[i];
	}

	static bool Matches(const uchar* text, const uchar* pattern, int& pos) {
		int length = strlen((const char*)pattern);

		if(pos + length > (int)strlen((const char*) text))
			return false;

		for(int i=0; i<length; i++)
			if(text[pos++] != pattern[i])
				return false;
		return true;
	}

	static ulong Search(SSTree* sst, uchar* str) {
		ulong node = sst->root();
		int pos = 0, length = strlen((char*) str);
		while(true) {
			node = sst->child(node, str[pos]);

			if(node == 0) // no match
				return 0;

			uchar* label = sst->edge(node);

			if(!Matches(str, label, pos))
				break;
			if(pos == length)
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
		if(runs.size() == 0) {
			policy = NULL;
			return;
		}

		int length = 0;
		for(unsigned int i=0; i<runs.size(); i++)
			length += strlen((char*)runs[i]) + 1;
		uchar* runsStr = new uchar[length];
		int pos = 0;
		for(unsigned int i=0; i<runs.size(); i++) {
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

		if(policy == NULL) return actions;

		for(int i=0; i<history.Size(); i++) {
			uchar* str = toString(history, -1, i);
			// cout << "Query: " << str << endl;
			ulong match = Search(static_cast<SSTree*>(policy), str);
			// cout << "Match: " << match << " " << policy->pathlabel(match) << endl;

			if(match != 0) {
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

/*---------------------------------------------------------------------------*/
class SuffixPolicyMapImpl : public SuffixPolicy {
private:
	mutable unsigned action_root_seed_;
	map<History, vector<int>> suffix_action_map;
	int count;

public:
	SuffixPolicyMapImpl(const DSPOMDP* model, Belief* belief = NULL);

	static string Name();

public:
	void Clear();
	int Count(); 

	void Put(const History& history, int action);
	void ConstructMapping();

	vector<int> SearchForLongestSuffixMatchActions(const History& history) const;
};

#endif
