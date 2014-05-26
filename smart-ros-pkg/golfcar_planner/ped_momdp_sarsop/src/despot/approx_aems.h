#ifndef APPROXAEMS_H
#define APPROXAEMS_H

#include "solver.h"
#include "pomdp.h"
#include "belief.h"
#include "node.h"
#include "globals.h"
#include "despot.h"

class APPROXAEMS : public Solver {
private:
  VNode* root_;
	SearchStatistics statistics_;

public:
  APPROXAEMS(const DSPOMDP* model, Belief* belief);

	int Search();
  void Update(int act, uint64_t obs);

private:
	void APPROXAEMSSearch();
	static void InitLowerBound(VNode* vnode, const DSPOMDP* model);
	static void InitUpperBound(VNode* vnode, const DSPOMDP* model);

	static void Expand(QNode* qnode, const DSPOMDP* model);
	static void Expand(VNode* vnode, const DSPOMDP* model);
	static void Backup(VNode* vnode);
	static void Update(VNode* vnode);
	static void Update(QNode* qnode);
	static double ApproxError(VNode* vnode);
	static vector<State*> Tau(const DSPOMDP* model, const vector<State*>& particles, int action, uint64_t obs);

	static int OptimalAction(VNode* vnode);
};

#endif
