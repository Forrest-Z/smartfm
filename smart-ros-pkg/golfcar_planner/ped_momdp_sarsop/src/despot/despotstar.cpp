#include "despotstar.h"
#include "despot.h"
#include <thread>

DESPOTSTAR::DESPOTSTAR(const DSPOMDP* model, Belief* belief, RandomStreams& streams)
	: Solver(model, belief),
	streams_(streams),
	root_(NULL)
{
}

VNode* DESPOTSTAR::Trial(VNode* root) {
	VNode* cur = root;

	int hist_size = history_.Size();

	do {
		if (cur->depth() > statistics_.longest_trial_length)
			statistics_.longest_trial_length = cur->depth();

		ExploitBlockers(cur);

		if (Gap(cur) == 0)
			break;

		if (cur->IsLeaf()) {
			double start = clock();
			Expand(cur, model_, streams_, history_);
			statistics_.time_node_expansion += (double) (clock() - start) / CLOCKS_PER_SEC;
			statistics_.num_expanded_nodes ++;
			statistics_.num_tree_particles += cur->particles().size();
		}

		double start = clock();
		QNode* qstar = SelectBestUpperBoundNode(cur);
		VNode* next = SelectBestWEUNode(qstar);
		statistics_.time_path += (clock() - start) / CLOCKS_PER_SEC;

		if (next == NULL) { break; }

		cur = next;
		history_.Add(qstar->edge(), cur->edge());
	} while(cur->depth() < Globals::config.search_depth && WEU(cur) > 0);

	history_.Truncate(hist_size);

	return cur;
}

void DESPOTSTAR::ExploitBlockers(VNode* vnode) {
	if (Globals::config.pruning_constant <= 0)
		return;

	// int count = 0;
	VNode* cur = vnode;
	while (cur != NULL) {
		VNode* blocker = FindBlocker(cur);

		if (blocker != NULL) {
			if (cur->parent() == NULL || blocker == cur) {
				double value = cur->default_move().value;
				cur->lower_bound(value);
				cur->upper_bound(value);
				cur->utility_upper_bound = value;
			} else {
				const map<uint64_t, VNode*>& siblings = cur->parent()->children();
				for (auto& it : siblings) {
					VNode* node = it.second;
					double value = node->default_move().value;
					node->lower_bound(value);
					node->upper_bound(value);
					node->utility_upper_bound = value;
				}
			}

			Backup(cur);

			if (cur->parent() == NULL)
				cur = NULL;
			else
				cur = cur->parent()->parent();
		} else {
			break;
		}
	}
}

double DESPOTSTAR::HSVISearch(VNode* root, SearchStatistics& statistics, double timeout) {
	streams_.position(0);
	// cout << streams_ << endl;
	// TODO: this can make lookahead upper bound fail
	// streams_ = RandomStreams(Seeds::Next(Globals::config.n_particles), Globals::config.search_depth);
	logi("Performing HSVI search");
	logi("Initializing lower and upper bounds at the root node...");
	InitBounds(root, model_, streams_, history_);
	logi("Done.");

	statistics.initial_lb = root->lower_bound();
	statistics.initial_ub = root->upper_bound();

	double used_time = 0;
	do {
		double start = get_time_second();
		VNode* cur = Trial(root);
		used_time += get_time_second() - start;

		start = get_time_second();
		Backup(cur);
		statistics.time_backup += get_time_second() - start;
		used_time += get_time_second() - start;

		// cout << statistics.num_trials << " " << root->upper_bound() << " " << root->lower_bound() << " " << (root->upper_bound() - root->lower_bound() > 1e-6) <<
			// " " << (root->upper_bound() - root->lower_bound()) << endl;
		statistics.num_trials ++;
  } while (used_time * (statistics.num_trials + 1.0) / statistics.num_trials < timeout &&
			(root->upper_bound() - root->lower_bound()) > 1e-6);

	/*
	cout << "target: " << root->lower_bound() << endl;
	CheckDESPOT(root, root->lower_bound());
	CheckDESPOTSTAR(root, root->lower_bound());
	*/

	statistics.num_policy_nodes = root->PolicyTreeSize();
	statistics.num_tree_nodes = root->Size();
	statistics.final_lb = root->lower_bound();
	statistics.final_ub = root->upper_bound();
	statistics.time_search = used_time;

	return root->lower_bound();
}

void DESPOTSTAR::Compare() {
	vector<State*> particles = belief_->Sample(Globals::config.n_particles);
	VNode* root = new VNode(move(particles));
	root->Free(*model_);
	SearchStatistics statistics;
	HSVISearch(root, statistics, Globals::config.time_per_move);

	CheckDESPOT(root, root->lower_bound());
	CheckDESPOTSTAR(root, root->lower_bound());
	delete root;
}

void DESPOTSTAR::InitLowerBound(VNode* vnode, const DSPOMDP* model, RandomStreams& streams, History& history) {
	streams.position(vnode->depth());
	ValuedAction move = model->LowerBound(vnode->particles(), streams, history);
	move.value *= Discount(vnode->depth());
	vnode->default_move(move);
	vnode->lower_bound(move.value);
}

void DESPOTSTAR::InitUpperBound(VNode* vnode, const DSPOMDP* model, RandomStreams& streams, History& history) {
	streams.position(vnode->depth());
	double upper = model->UpperBound(vnode->particles(), streams, history);
	vnode->utility_upper_bound = upper * Discount(vnode->depth());
	upper = upper * Discount(vnode->depth()) - Globals::config.pruning_constant;
	vnode->upper_bound(upper);
}

void DESPOTSTAR::InitBounds(VNode* vnode, const DSPOMDP* model, RandomStreams& streams, History& history) {
	InitLowerBound(vnode, model, streams, history);
	InitUpperBound(vnode, model, streams, history);
	if (vnode->upper_bound() < vnode->lower_bound()) {
		vnode->upper_bound(vnode->lower_bound());
	}
}

int DESPOTSTAR::Search() {
	double start = get_time_second();
	// cout << *belief_ << endl;
	vector<State*> particles = belief_->Sample(Globals::config.n_particles);
	for (int i = 0; i < particles.size(); i ++) {
		particles[i]->scenario_id = i;
		// cout << "Sampled particle " << i << endl;
		// model_->PrintState(*particles[i]);
	}

	root_ = new VNode(move(particles));
	statistics_ = SearchStatistics();
	statistics_.num_particles_before_search = model_->num_active_particles;
	cout << "Time initializing search " << (get_time_second() - start) << endl;

	start = get_time_second();
	HSVISearch(root_, statistics_, Globals::config.time_per_move);
	//root_->PrintPolicyTree();
	root_->PrintTree(1);
	statistics_.num_particles_after_search = model_->num_active_particles;
	cout << "Time for search " << (get_time_second() - start) <<endl;

	cout << "Default action " << root_->default_move() << endl;
	logi("Freeing particles...");
	start = get_time_second();
	root_->Free(*model_);
	logi("Done!");

	if (VERBOSITY >= INFO) {
		root_->PrintTree();
	}

	int astar = OptimalAction(root_);
	delete root_;
	cout << "Time cleaning search " << (get_time_second() - start) << endl;
	cout << statistics_ << endl;

	return astar;
}

double DESPOTSTAR::CheckDESPOT(const VNode* vnode, double regularized_value) {
	cout << "--------------------------------------------------------------------------------" << endl;

	const vector<State*>& particles = vnode->particles();
	vector<State*> copy;
	for (auto& particle : particles)
		copy.push_back(model_->Copy(particle));
	VNode* root = new VNode(move(copy));

	double pruning_constant = Globals::config.pruning_constant;
	Globals::config.pruning_constant = 0;

	streams_.position(0);
	InitBounds(root, model_, streams_, history_);

	double used_time = 0;
	int num_trials = 0, prev_num = 0;
	double pruned_value;
	do {
		double start = get_time_second();
		VNode* cur = Trial(root);
		num_trials ++;
		used_time += get_time_second() - start;

		start = get_time_second();
		Backup(cur);
		used_time += get_time_second() - start;

		if (double (num_trials - prev_num) > 0.05 * prev_num) {
			int pruned_action;
			Globals::config.pruning_constant = pruning_constant;
			VNode* pruned = Prune(root, pruned_action, pruned_value);
			Globals::config.pruning_constant = 0;
			prev_num = num_trials;

			pruned->Free(*model_);
			delete pruned;

			cout << "# trials = " << num_trials << "; target = " << regularized_value << ", current = " << pruned_value << ", l = " << root->lower_bound() << ", u = " << root->upper_bound() << "; time = " << used_time << endl;

			if (pruned_value >= regularized_value)
				break;
		}
  } while (true);


	cout << "DESPOT: # trials = " << num_trials << "; target = " << regularized_value << ", current = " << pruned_value << ", l = " << root->lower_bound() << ", u = " << root->upper_bound() << "; time = " << used_time << endl;
	Globals::config.pruning_constant = pruning_constant;
	cout << "--------------------------------------------------------------------------------" << endl;

	root->Free(*model_);
	delete root;

	return used_time;
}

double DESPOTSTAR::CheckDESPOTSTAR(const VNode* vnode, double regularized_value) {
	cout << "--------------------------------------------------------------------------------" << endl;

	const vector<State*>& particles = vnode->particles();
	vector<State*> copy;
	for (auto& particle : particles)
		copy.push_back(model_->Copy(particle));
	VNode* root = new VNode(move(copy));

	streams_.position(0);
	InitBounds(root, model_, streams_, history_);

	double used_time = 0;
	int num_trials = 0;
	do {
		double start = clock();
		VNode* cur = Trial(root);
		num_trials ++;
		used_time += double (clock() - start) / CLOCKS_PER_SEC;

		start = clock();
		Backup(cur);
		used_time += double (clock() - start) / CLOCKS_PER_SEC;
  } while (root->lower_bound() < regularized_value);

	cout << "DESPOTSTAR: # trials = " << num_trials << "; target = " << regularized_value << ", current = " << root->lower_bound() << ", l = " << root->lower_bound() << ", u = " << root->upper_bound() << "; time = " << used_time << endl;
	cout << "--------------------------------------------------------------------------------" << endl;

	root->Free(*model_);
	delete root;

	return used_time;
}

VNode* DESPOTSTAR::Prune(VNode* vnode, int& pruned_action, double& pruned_value) {
	VNode* pruned_v = new VNode(vector<State*>(), vnode->depth(), NULL, vnode->edge());

	vector<QNode*>& children = vnode->children();
	int astar = -1;
	double nustar = Globals::NEG_INFTY;
	QNode* qstar = NULL;
	for (auto& qnode : children) {
		double nu;
		QNode* pruned_q = Prune(qnode, nu);

		if (nu > nustar) {
			nustar = nu;
			astar = qnode->edge();

			if (qstar != NULL)
				delete qstar; // ugly

			qstar = pruned_q;
		} else {
			delete pruned_q; // ugly
		}
	}

	if (nustar < vnode->default_move().value) {
		nustar = vnode->default_move().value;
		astar = vnode->default_move().action;
		delete qstar; // ugly
	} else {
		pruned_v->children().push_back(qstar);
		qstar->parent(pruned_v);
	}

	pruned_v->lower_bound(vnode->lower_bound()); // for debugging
	pruned_v->upper_bound(vnode->upper_bound());

	pruned_action = astar;
	pruned_value = nustar;

	return pruned_v;
}

QNode* DESPOTSTAR::Prune(QNode* qnode, double& pruned_value) {
	QNode* pruned_q = new QNode(NULL, qnode->edge());
	pruned_value = qnode->step_reward - Globals::config.pruning_constant;
	auto& children = qnode->children();
	for (auto& it : children) {
		int astar;
		double nu;
		VNode* pruned_v = Prune(it.second, astar, nu);
		if (nu == it.second->default_move().value) {
			delete pruned_v; // ugly
		} else {
			pruned_q->children()[it.first] = pruned_v;
			pruned_v->parent(pruned_q);
		}
		pruned_value += nu;
	}

	pruned_q->lower_bound(qnode->lower_bound()); // for debugging
	pruned_q->upper_bound(qnode->upper_bound()); // for debugging

	return pruned_q;
}

int DESPOTSTAR::OptimalAction(VNode* vnode) {
	double lower = Globals::NEG_INFTY;
	int action = -1;
	for (int a=0; a < vnode->children().size(); a ++) {
		QNode* qnode = vnode->Child(a);
		if (qnode->lower_bound() > lower) {
			lower = qnode->lower_bound();
			action = a;
		}
	}

	if (vnode->default_move().value > lower) {
		action = vnode->default_move().action;
	}

	return action;
}

double DESPOTSTAR::Gap(VNode* vnode) {
	return (vnode->upper_bound() - vnode->lower_bound());
}

double DESPOTSTAR::WEU(VNode* vnode) {
	return WEU(vnode, Globals::config.xi);
}

double DESPOTSTAR::WEU(VNode* vnode, double xi) {
	VNode* root = vnode;
	while(root->parent() != NULL) {
		root = root->parent()->parent();
	}
	return Gap(vnode) - xi * Weight(vnode) * Gap(root);
}

VNode* DESPOTSTAR::SelectBestWEUNode(QNode* qnode) {
	double weustar = Globals::NEG_INFTY;
	VNode* vstar = NULL;
	for (auto& it : qnode->children()) {
		VNode* vnode = it.second;

		double weu = WEU(vnode);
		if (weu >= weustar) {
			weustar = weu;
			vstar = vnode->vstar;
		}
	}
	return vstar;
}

QNode* DESPOTSTAR::SelectBestUpperBoundNode(VNode* vnode) {
	int astar = -1;
	double upperstar = Globals::NEG_INFTY;
	for (int action = 0; action < vnode->children().size(); action ++) {
		QNode* qnode = vnode->Child(action);

		if (qnode->upper_bound() > upperstar) {
			upperstar = qnode->upper_bound();
			astar = action;
		}
	}
	assert(astar >= 0);
	return vnode->Child(astar);
}

double DESPOTSTAR::Weight(VNode* vnode) {
	return State::Weight(vnode->particles());
}

double DESPOTSTAR::Weight(QNode* qnode) {
	double weight = 0;
	for (auto& it : qnode->children())
		weight += Weight(it.second);
	return weight;
}

void DESPOTSTAR::Update(VNode* vnode) {
	if (vnode->IsLeaf()) return;

	double lower = vnode->default_move().value;
	double upper = vnode->default_move().value;
	double utility_upper = Globals::NEG_INFTY;

	for (int action = 0; action < vnode->children().size(); action ++) {
		QNode* qnode = vnode->Child(action);

		lower = max(lower, qnode->lower_bound());
		upper = max(upper, qnode->upper_bound());
		utility_upper = max(utility_upper, qnode->utility_upper_bound);
	}

	if (lower > vnode->lower_bound())
		vnode->lower_bound(lower);
	if (upper < vnode->upper_bound())
		vnode->upper_bound(upper);
	if (utility_upper < vnode->utility_upper_bound)
		vnode->utility_upper_bound = utility_upper;
}

void DESPOTSTAR::Update(QNode* qnode) {
	double lower = qnode->step_reward;
	double upper = qnode->step_reward;
	double utility_upper = qnode->step_reward + Globals::config.pruning_constant;

	for (auto& it : qnode->children()) {
		VNode* vnode = it.second;

		lower += vnode->lower_bound();
		upper += vnode->upper_bound();
		utility_upper += vnode->utility_upper_bound;
	}

	if (lower > qnode->lower_bound())
		qnode->lower_bound(lower);
	if (upper < qnode->upper_bound())
		qnode->upper_bound(upper);
	if (utility_upper < qnode->utility_upper_bound)
		qnode->utility_upper_bound = utility_upper;
}

void DESPOTSTAR::Backup(VNode* vnode) {
	int iter = 0;
	logi("- Backup ", vnode, " at depth ", vnode->depth());
	while(true) {
		logd(" Iter ", iter, " ", vnode);

		Update(vnode);

		QNode* parentq = vnode->parent();
		if (parentq == NULL)
			break;

		Update(parentq);
		logd(" Updated Q-node to (", parentq->lower_bound(), ", ", parentq->upper_bound(), ")");

		vnode = parentq->parent();
		iter ++;
	}
	logi("* Backup complete!");
}

VNode* DESPOTSTAR::FindBlocker(VNode* vnode) {
	VNode* cur = vnode;
	int count = 1;
	while (cur != NULL) {
		if (cur->utility_upper_bound - count * Globals::config.pruning_constant <= cur->default_move().value) {
			break;
		}
		count ++;
		if (cur->parent() == NULL)
			cur = NULL;
		else
			cur = cur->parent()->parent();
	}
	return cur;
}

void DESPOTSTAR::Expand(VNode* vnode, const DSPOMDP* model, RandomStreams& streams,
		History& history) {
	vector<QNode*>& children = vnode->children();
	logi("- Expanding vnode ", vnode);
	for (int action = 0; action < model->NumActions(); action ++) {
		logi(" Action ", action);
		QNode* qnode = new QNode(vnode, action);
		children.push_back(qnode);

		Expand(qnode, model, streams, history);
	}
	logi("* Expansion complete!");
}

void DESPOTSTAR::Expand(QNode* qnode, const DSPOMDP* model, RandomStreams& streams,
		History& history) {
	VNode* parent = qnode->parent();
	streams.position(parent->depth());
	map<uint64_t, VNode*>& children = qnode->children();

	const vector<State*>& particles = parent->particles();

	double step_reward = 0;

	// Partition particles by observation
	map<uint64_t, vector<State*>> partitions;
	uint64_t obs;
	double reward;
	logi(" ", particles.size(), " particles");
	for (auto& particle: particles) {
		logd(" Original: ", *particle);

		State* copy = model->Copy(particle);

		logd(" Before step: ", *copy);

		bool terminal = model->Step(*copy,
				streams.Entry(copy->scenario_id), qnode->edge(), reward, obs);

		step_reward += reward * copy->weight;

		logd(" After step: ", *copy, " ", reward * copy->weight, " ", reward, " ", copy->weight);

		if (!terminal) {
			partitions[obs].push_back(copy);
		} else {
			model->Free(copy);
		}
	}
	step_reward = Discount(parent->depth()) * step_reward - Globals::config.pruning_constant;

	double lower_bound = step_reward;
	double upper_bound = step_reward;

	// Create new belief nodes 
	// cout << partitions.size() << " obs" << endl;
	for (auto& it : partitions) {
		uint64_t obs = it.first;
		logi(" Creating node for obs ", obs);
		VNode* vnode = new VNode(move(partitions[obs]), parent->depth()+1, qnode, obs);
		logi(" New node created!");
		children[obs] = vnode;

		history.Add(qnode->edge(), obs);
		InitBounds(vnode, model, streams, history);
		history.RemoveLast();
		logi(" New node's bounds: (", vnode->lower_bound(), ", ", vnode->upper_bound(), ")");

		lower_bound += vnode->lower_bound();
		upper_bound += vnode->upper_bound();
	}

	qnode->step_reward = step_reward;
	qnode->lower_bound(lower_bound);
	qnode->upper_bound(upper_bound);
	qnode->utility_upper_bound = upper_bound + Globals::config.pruning_constant;

	qnode->default_value = lower_bound; // for debugging
}

