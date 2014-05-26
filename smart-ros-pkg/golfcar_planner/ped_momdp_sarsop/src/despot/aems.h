#ifndef AEMS_H
#define AEMS_H

#include "solver.h"
#include "pomdp.h"
#include "belief.h"
#include "node.h"
#include "globals.h"

class AEMS : public Solver {
private:
  VNode* root_;
	SearchStatistics statistics_;
	const BeliefMDP* model_;

public:
  AEMS(const DSPOMDP* model, Belief* belief);

	int Search();
	void Update(int action, uint64_t obs);

private:
	static void InitLowerBound(VNode* vnode, const BeliefMDP* model, History& history);
	static void InitUpperBound(VNode* vnode, const BeliefMDP* model, History& history);

	static void Expand(QNode* qnode, const BeliefMDP* model, History& history);
	static void Expand(VNode* vnode, const BeliefMDP* model, History& history);
	static void Backup(VNode* vnode);
	static void Update(VNode* vnode);
	static void Update(QNode* qnode);
	static VNode* FindMaxApproxErrorLeaf(VNode* root);
	static void FindMaxApproxErrorLeaf(VNode* vnode, double likelihood, double& bestAE, VNode*& bestNode);
	static void FindMaxApproxErrorLeaf(QNode* qnode, double likelihood, double& bestAE, VNode*& bestNode);

	static int OptimalAction(VNode* vnode);
	static double Likelihood(QNode* qnode);
	static double AEMS2Likelihood(QNode* qnode);
};

#endif
