#ifndef NODE_H
#define NODE_H

#include "pomdp.h"
#include "util/util.h"
#include "random_streams.h"
#include "util/logging.h"

using namespace logging;

class QNode;

/* A belief/value/AND node in the search tree. */
class VNode {
protected:
	vector<State*> particles_; // Used in DESPOT
	Belief* belief_; // Used in AEMS
	int depth_;
	QNode* parent_;
	uint64_t edge_;

	vector<QNode*> children_;

	ValuedAction default_move_; // Value and action given by default policy
	double lower_bound_;
	double upper_bound_;

public:
	VNode* vstar;
	double likelihood; // Used in AEMS
	double utility_upper_bound;

	VNode(vector<State*>&& particles, int depth = 0, QNode* parent = NULL, uint64_t edge = -1);
	VNode(Belief* belief, int depth = 0, QNode* parent = NULL, uint64_t edge = -1);
	~VNode();

	Belief* belief() const;
	const vector<State*>& particles() const;
	int depth() const;
	void parent(QNode* parent);
	QNode* parent();
	uint64_t edge();

	vector<QNode*>& children();
	QNode* Child(int action);
	int Size() const;
	int PolicyTreeSize() const;

	void default_move(ValuedAction move);
	ValuedAction default_move() const;
	void lower_bound(double value);
	double lower_bound() const;
	void upper_bound(double value);
	double upper_bound() const;

	bool IsLeaf();

	void PrintTree(int depth = -1, ostream& os = cout);
	void PrintPolicyTree(int depth = -1, ostream& os = cout);

	void Free(const DSPOMDP& model);
};

/*---------------------------------------------------------------------------*/

/* A Q-node/AND-node (child of a belief node) of the search tree. */
class QNode {
protected:
	VNode* parent_;
	int edge_;
	map<uint64_t, VNode*> children_;
	double lower_bound_;
	double upper_bound_;

public:
	double default_value;
	double utility_upper_bound;
	double step_reward;
	double likelihood;
	VNode* vstar;

	QNode(VNode* parent, int edge);
	~QNode();

	void parent(VNode* parent);
	VNode* parent();
	int edge();
	map<uint64_t, VNode*>& children();
	VNode* Child(uint64_t obs);
	int Size() const;
	int PolicyTreeSize() const;

	void lower_bound(double value);
	double lower_bound() const;
	void upper_bound(double value);
	double upper_bound() const;
};

#endif
