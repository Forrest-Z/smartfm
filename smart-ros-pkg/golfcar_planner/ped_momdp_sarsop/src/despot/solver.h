#ifndef SOLVER_H
#define SOLVER_H

#include "globals.h"
#include "history.h"

class DSPOMDP;
class Belief;

struct SearchStatistics {
	double initial_lb,
				 initial_ub,
				 final_lb,
				 final_ub;
	double time_search;
	double time_path;
	double time_backup;
	double time_node_expansion;
	int num_policy_nodes;
	int num_tree_nodes;
	int num_expanded_nodes;
	int num_tree_particles;
	int num_particles_before_search;
	int num_particles_after_search;
	int num_trials;
	int longest_trial_length;

	SearchStatistics();

	friend ostream& operator<<(ostream& os, const SearchStatistics& statitics);
};


class Solver {
protected:
	const DSPOMDP* model_;
	Belief* belief_;
	History history_;

public:
	Solver(const DSPOMDP* model, Belief* belief);
	virtual ~Solver();

	virtual int Search() = 0;
	virtual void Update(int act, uint64_t obs);

	void belief(Belief* b);
	Belief* belief();
};

#endif
