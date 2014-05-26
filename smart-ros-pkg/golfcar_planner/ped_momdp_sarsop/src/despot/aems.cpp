#include "aems.h"

// TODO: reuse previous search tree

AEMS::AEMS(const DSPOMDP* model, Belief* belief)
	: Solver(model, belief),
	root_(NULL)
{
	model_ = static_cast<const BeliefMDP*>(model);
}

int AEMS::Search() {
	if (root_ == NULL) {
		root_ = new VNode(belief_->MakeCopy());
		InitLowerBound(root_, model_, history_);
		InitUpperBound(root_, model_, history_);
	}

	statistics_ = SearchStatistics();
	model_->PrintBelief(*belief_);
	//cout << *belief_ << endl;
	cout << "# active particles before search  = " << model_->num_active_particles << endl;
  clock_t begin = clock();
	statistics_.initial_lb = root_->lower_bound();
	statistics_.initial_ub = root_->upper_bound();

	int num_active_particles = model_->num_active_particles;
	do {
		VNode* promising_node = FindMaxApproxErrorLeaf(root_);
		assert(promising_node->IsLeaf());

		int hist_size = history_.Size();
		clock_t start = clock();
		Expand(promising_node, model_, history_);
		statistics_.time_node_expansion += (clock() - start) / CLOCKS_PER_SEC;
		Backup(promising_node);
		history_.Truncate(hist_size);

		statistics_.num_trials ++;
		statistics_.num_expanded_nodes ++;
  } while ((double)(clock() - begin) / CLOCKS_PER_SEC < Globals::config.time_per_move
			&& (root_->upper_bound() - root_->lower_bound()) > 1e-6);

	statistics_.num_tree_particles = model_->num_active_particles - num_active_particles;
	statistics_.num_tree_nodes = root_->Size();

	statistics_.final_lb = root_->lower_bound();
	statistics_.final_ub = root_->upper_bound();
	statistics_.time_search = (double)(clock() - begin) / CLOCKS_PER_SEC;

	cout << "# active particles after search  = " << model_->num_active_particles << endl;
	cout << "# active particles after destructing tree  = " << model_->num_active_particles << endl;

	cout << statistics_ << endl;

	if (VERBOSITY >= INFO) {
		root_->PrintTree();
	}

	int astar = OptimalAction(root_);
	//delete root_;
	return astar;
}

int AEMS::OptimalAction(VNode* vnode) {
	double lower = Globals::NEG_INFTY;
	int action = -1;
	for (int a=0; a<vnode->children().size(); a++) {
		QNode* qnode = vnode->Child(a);
		if (qnode->lower_bound() > lower) {
			lower = qnode->lower_bound();
			action = a;
		}
	}
	return action;
}

VNode* AEMS::FindMaxApproxErrorLeaf(VNode* root) {
	double bestAE = Globals::NEG_INFTY;
	VNode* bestNode = NULL;
	FindMaxApproxErrorLeaf(root, 1.0, bestAE, bestNode);
	return bestNode;
}

void AEMS::FindMaxApproxErrorLeaf(VNode* vnode, double likelihood, double& bestAE, VNode*& bestNode) {
	if (vnode->IsLeaf()) {
		double curAE = likelihood * vnode->likelihood * Discount(vnode->depth()) * (vnode->upper_bound() - vnode->lower_bound());
		if (curAE > bestAE) {
			bestAE = curAE;
			bestNode = vnode;
		}
	} else {
		for (int a=0; a<vnode->children().size(); a++) {
			FindMaxApproxErrorLeaf(vnode->Child(a), likelihood, bestAE, bestNode);
		}
	}
}

void AEMS::FindMaxApproxErrorLeaf(QNode* qnode, double likelihood, double& bestAE, VNode*& bestNode) {
	likelihood *= Likelihood(qnode);
	for (auto& it : qnode->children()) {
		VNode* vnode = it.second;
		FindMaxApproxErrorLeaf(vnode, likelihood, bestAE, bestNode);
	}
}

double AEMS::Likelihood(QNode* qnode) {
	return AEMS2Likelihood(qnode);
}

double AEMS::AEMS2Likelihood(QNode* qnode) {
	VNode* vnode = qnode->parent();
	QNode* qstar = NULL;
	for (int action = 0; action<vnode->children().size(); action++) {
		QNode* child = vnode->Child(action);

		if (qstar == NULL || child->upper_bound() > qstar->upper_bound())
			qstar = child;
	}

	return qstar == qnode;
}

void AEMS::Update(VNode* vnode) {
	if (vnode->IsLeaf()) return;

	double lower = Globals::NEG_INFTY;
	double upper = Globals::NEG_INFTY;

	for (int action = 0; action<vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);

		lower = max(lower, qnode->lower_bound());
		upper = max(upper, qnode->upper_bound());
	}

	if (lower > vnode->lower_bound())
		vnode->lower_bound(lower);
	if (upper < vnode->upper_bound())
		vnode->upper_bound(upper);
}

void AEMS::Update(QNode* qnode) {
	double lower = qnode->step_reward;
	double upper = qnode->step_reward;

	for (auto& it : qnode->children()) {
		VNode* vnode = it.second;

		lower += Discount() * vnode->likelihood * vnode->lower_bound();
		upper += Discount() * vnode->likelihood * vnode->upper_bound();
	}

	if (lower > qnode->lower_bound())
		qnode->lower_bound(lower);
	if (upper < qnode->upper_bound())
		qnode->upper_bound(upper);
}

void AEMS::Backup(VNode* vnode) {
	int iter = 0;
	logi("- Backup ", vnode, " at depth ", vnode->depth());
	while(true) {
		logd(" Iter ", iter++, " ", vnode);

		Update(vnode);
		logd(" Updated vnode ", vnode);

		QNode* parentq = vnode->parent();
		if (parentq == NULL)
			break;

		Update(parentq);
		logd(" Updated Q-node to (", parentq->lower_bound(), ", ", parentq->upper_bound(), ")");

		vnode = parentq->parent();
	}
	logi("* Backup complete!");
}

void AEMS::InitLowerBound(VNode* vnode, const BeliefMDP* model, History& history) {
	double value = model->LowerBound(vnode->belief()).value;
	vnode->lower_bound(value);
}

void AEMS::InitUpperBound(VNode* vnode, const BeliefMDP* model, History& history) {
	double value = model->UpperBound(vnode->belief());
	vnode->upper_bound(value);
}

void AEMS::Expand(VNode* vnode, const BeliefMDP* model, History& history) {
	vector<QNode*>& children = vnode->children();
	logi("- Expanding vnode ", vnode);
	for (int action = 0; action<model->NumActions(); action++) {
		logi(" Action ", action);
		QNode* qnode = new QNode(vnode, action);
		children.push_back(qnode);

		Expand(qnode, model, history);
	}
	logi("* Expansion complete!");
}

void AEMS::Expand(QNode* qnode, const BeliefMDP* model, History& history) {
	VNode* parent = qnode->parent();
	int action = qnode->edge();
	map<uint64_t, VNode*>& children = qnode->children();

	const Belief* belief = parent->belief();
	// cout << *belief << endl;

	double step_reward = model->StepReward(belief, qnode->edge());

	map<uint64_t, double> obss;
	model->Observe(belief, qnode->edge(), obss);

	double lower_bound = step_reward;
	double upper_bound = step_reward;

	/*
	cout << "Observations" << endl;
	double sum = 0;
	for (auto& it : obss) {
		cout << "(" << it.first << ", " << it.second << ") ";
		sum += it.second;
	}
	cout << endl;
	cout << sum << endl;
	*/

	// Create new belief nodes
	for (auto& it : obss) {
		uint64_t obs = it.first;
		double weight = it.second;
		logi(" Creating node for obs ", obs, " with weight ", weight);
		VNode* vnode = new VNode(model->Tau(belief, action, obs), parent->depth()+1, qnode, obs);
		vnode->likelihood = weight;
		logi(" New node created!");
		children[obs] = vnode;

		InitLowerBound(vnode, model, history);
		InitUpperBound(vnode, model, history);

		lower_bound += weight * Discount() * vnode->lower_bound();
		upper_bound += weight * Discount() * vnode->upper_bound();
	}

	qnode->step_reward = step_reward;
	qnode->lower_bound(lower_bound);
	qnode->upper_bound(upper_bound);
}

void AEMS::Update(int action, uint64_t obs) {
	logi("- Updating belief, history and root with action ", action, " and observation ", obs);

	cout << "Full" << endl;
	root_->PrintTree(1);
	VNode* node = root_->Child(action)->Child(obs);
	root_->Child(action)->children().erase(obs);
	delete root_;
	root_ = node;
	root_->likelihood = 1.0;
	root_->parent(NULL);

	cout << "Reused" << endl;
	root_->PrintTree();

	belief_ = root_->belief();
	// belief_->Update(action, obs);

	/*
	Belief* new_belief = model_->Tau(belief_, action, obs);
	delete belief_;
	belief_ = new_belief;
	*/

	history_.Add(action, obs);

	logi("  Done");
}
