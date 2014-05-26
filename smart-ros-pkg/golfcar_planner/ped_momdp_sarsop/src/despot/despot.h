#ifndef DESPOT_H
#define DESPOT_H

#include "solver.h"
#include "pomdp.h"
#include "belief.h"
#include "node.h"
#include "globals.h"
#include "history.h"
#include "random_streams.h"

class DESPOT : public Solver {
	//static class VNode { };
private:
	RandomStreams& streams_;
  VNode* root_;
	SearchStatistics statistics_;

public:
  DESPOT(const DSPOMDP* model, Belief* belief, RandomStreams& streams);

	int Search();

	static void InitLowerBound(VNode* vnode, const DSPOMDP* model, RandomStreams& streams, History& history);
	static void InitUpperBound(VNode* vnode, const DSPOMDP* model, RandomStreams& streams, History& history);

	static void Expand(VNode* vnode, const DSPOMDP* model, RandomStreams& streams, History& history);
	static void Backup(VNode* vnode);

	static double Weight(QNode* qnode);
	static double Weight(VNode* vnode);
	static double DiscountedGap(VNode* vnode);

//private:
public:
	void HSVISearch();
	void HSVIBSearch();

	static void FillHistory(VNode* vnode, History& history);
	static void Expand(QNode* qnode, const DSPOMDP* model, RandomStreams& streams, History& history);
	static void Update(VNode* vnode);
	static void Update(QNode* qnode);
	static VNode* Prune(VNode* vnode, int& pruned_action, double& pruned_value);
	static QNode* Prune(QNode* qnode, double& pruned_value);
	static double WEU(VNode* vnode);
	static double WEU(VNode* vnode, double epsilon);
	static VNode* SelectBestWEUNode(QNode* qnode);
	static QNode* SelectBestUpperBoundNode(VNode* vnode);
	static int OptimalAction(VNode* vnode);

	/*
	// Record all the (history, optimal action) pairs for the suffix policy
	void RetrieveHistoryActionMapping(SuffixPolicyLowerBound* suffix_policy);
	// Helper function
	private:	void RetrieveHistoryActionMapping(unique_ptr<VNode>& node,
			History& history,
			SuffixPolicyLowerBound* suffix_policy);
	*/
};

#endif
