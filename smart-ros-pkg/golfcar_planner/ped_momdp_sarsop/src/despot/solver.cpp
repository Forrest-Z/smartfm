#include "solver.h"
#include "util/logging.h"
#include "pomdp.h"
#include "belief.h"

SearchStatistics::SearchStatistics() :
	initial_lb(Globals::NEG_INFTY),
	initial_ub(Globals::POS_INFTY),
	final_lb(Globals::NEG_INFTY),
	final_ub(Globals::POS_INFTY),
	time_search(0),
	time_path(0),
	time_backup(0),
	time_node_expansion(0),
	num_policy_nodes(0),
	num_tree_nodes(0),
	num_expanded_nodes(0),
	num_tree_particles(0),
	num_particles_before_search(0),
	num_particles_after_search(0),
	num_trials(0),
	longest_trial_length(0)
{}

ostream& operator<<(ostream& os, const SearchStatistics& statistics) {
	os << "Initial bounds: (" << statistics.initial_lb << ", " << statistics.initial_ub << ")" << endl;
	os << "Final bounds: (" << statistics.final_lb << ", " << statistics.final_ub << ")" << endl;
	os << "Time (s): path / expansion / backup / total = " << statistics.time_path
		<< " / " << statistics.time_node_expansion
		<< " / " << statistics.time_backup
		<< " / " << statistics.time_search << endl;
	os << "Trials: no. / max length = " << statistics.num_trials
		<< " / " << statistics.longest_trial_length << endl;
	os << "# nodes: expanded / total / policy = " << statistics.num_expanded_nodes
		<< " / " << statistics.num_tree_nodes
		<< " / " << statistics.num_policy_nodes << endl;
	os << "# particles: initial / final / tree = " << statistics.num_particles_before_search << " / " << statistics.num_particles_after_search << " / " << statistics.num_tree_particles; // << endl;
	return os;
}

Solver::Solver(const DSPOMDP* model, Belief* belief) :
	model_(model),
	belief_(belief),
	history_(History()) {
}

Solver::~Solver() {
	delete belief_;
}

void Solver::Update(int action, uint64_t obs) {
	logi("- Updating belief and history with action ", action, " and observation ", obs);
	belief_->Update(action, obs);
	history_.Add(action, obs);
	logi("  Done");
}

void Solver::belief(Belief* b) {
	belief_ = b;
	//cout << "Initial belief " << *belief_ << endl;
	history_.Truncate(0);
}

Belief* Solver::belief() {
	return belief_;
}
