#include "despot.h"

DESPOT::DESPOT(const DSPOMDP* model, Belief* belief, RandomStreams& streams)
	: Solver(model, belief),
	streams_(streams),
	root_(NULL)
{
}

void DESPOT::HSVISearch() {
	logi("Performing HSVI search");
	// streams_ =  RandomStreams(Seeds::Next(Globals::config.n_particles), Globals::config.search_depth);
	streams_.position(0);
  clock_t begin = clock();
	logi("Initializing lower bound at the root node...");
	InitLowerBound(root_, model_, streams_, history_);
	logi("Done.");
	logi("Initializing upper bound at the root node...");
	InitUpperBound(root_, model_, streams_, history_);
	logi("Done.");

	statistics_.initial_lb = root_->lower_bound();
	statistics_.initial_ub = root_->upper_bound();

	// root_->PrintTree();
	// cout << endl;

	do {
		VNode* cur = root_;

		int hist_size = history_.Size();

		do {
			if (cur->depth() > statistics_.longest_trial_length)
				statistics_.longest_trial_length = cur->depth();

			if (cur->IsLeaf()) {
				double start = clock();

				Expand(cur, model_, streams_, history_);
				//root_->PrintTree(2);  cout << endl;

				statistics_.num_expanded_nodes ++;
				statistics_.num_tree_particles += cur->particles().size();
				statistics_.time_node_expansion += (double) (clock() - start) / CLOCKS_PER_SEC;
			}

			if (cur->IsLeaf()) break; // Not expanded

			double start = clock();
			QNode* qstar = SelectBestUpperBoundNode(cur);
			VNode* next = SelectBestWEUNode(qstar);

			statistics_.time_path += (clock() - start) / CLOCKS_PER_SEC; 
			if (next == NULL) { break; }
			cur = next;
			// cout << qstar->edge() << " " << cur->edge() << endl;
			history_.Add(qstar->edge(), cur->edge());
		} while(cur->depth() < Globals::config.search_depth && WEU(cur) > 0);
		history_.Truncate(hist_size);

		// root_->PrintTree(0);
		// cout << endl;

		double start = clock();
		Backup(cur);
		statistics_.time_backup += (clock() - start) / CLOCKS_PER_SEC;

		// root_->PrintTree();
		// cout << endl;

		/*
		if (statistics_.num_trials % 100 == 0) {
			int pruned_action;
			double pruned_value;
			VNode* pruned = Prune(root_, pruned_action, pruned_value);
			delete pruned;
			cout << statistics_.num_trials << " " << pruned_value << " " << root_->lower_bound() << " " << root_->upper_bound() << " " << (clock() - begin) / CLOCKS_PER_SEC << endl;
		}
		*/

		if (VERBOSITY >= VERBOSE) {
			cout << "Tree at trial " << statistics_.num_trials << endl;
			root_->PrintTree();
			cout << endl;
		}
		statistics_.num_trials ++;
  } while ((double)(clock() - begin) / CLOCKS_PER_SEC < Globals::config.time_per_move && 
			(root_->upper_bound() - root_->lower_bound()) > 1e-6);

	statistics_.num_tree_nodes = root_->Size();
	statistics_.final_lb = root_->lower_bound(); 
	statistics_.final_ub = root_->upper_bound();
	statistics_.time_search = (double)(clock() - begin) / CLOCKS_PER_SEC;
}

void DESPOT::HSVIBSearch() {
	streams_.position(0);
  clock_t begin = clock();
	InitLowerBound(root_, model_, streams_, history_);
	InitUpperBound(root_, model_, streams_, history_);
	statistics_.initial_lb = root_->lower_bound(); 
	statistics_.initial_ub = root_->upper_bound();

	do { 
		int hist_size = history_.Size();
		VNode* promising_node = root_->vstar;
		assert(promising_node->IsLeaf());

		// root_->PrintTree();
		// cout << endl;

		FillHistory(promising_node, history_);
		Expand(promising_node, model_, streams_, history_);
		Backup(promising_node);
		history_.Truncate(hist_size);

		statistics_.num_trials ++;
		statistics_.num_expanded_nodes ++;

		if (VERBOSITY >= DEBUG) {
			root_->PrintTree();
			cout << endl;
		}
  } while ((double)(clock() - begin) / CLOCKS_PER_SEC < Globals::config.time_per_move
			&& (root_->upper_bound() - root_->lower_bound()) > 1e-6
			&& root_->vstar != NULL);

	statistics_.num_tree_nodes = root_->Size();
	statistics_.final_lb = root_->lower_bound(); 
	statistics_.final_ub = root_->upper_bound();
	statistics_.time_search = (double)(clock() - begin) / CLOCKS_PER_SEC;
}

void DESPOT::InitLowerBound(VNode* vnode, const DSPOMDP* model, RandomStreams& streams, History& history) {
	streams.position(vnode->depth());
	ValuedAction move = model->LowerBound(vnode->particles(), streams, history);
	vnode->default_move(move);
	vnode->lower_bound(move.value);
}

void DESPOT::InitUpperBound(VNode* vnode, const DSPOMDP* model, RandomStreams& streams, History& history) {
	streams.position(vnode->depth());
	vnode->upper_bound(model->UpperBound(vnode->particles(), streams, history));
}

void DESPOT::FillHistory(VNode* cur, History& history) {
	if (cur->parent() == NULL)
		return;

	int a = cur->edge();
	int o = cur->parent()->edge();
	cur = cur->parent()->parent();
	FillHistory(cur, history);
	history.Add(a, o);
}

int DESPOT::Search() {
	// cout << *belief_ << endl;
	cout << "# active particles before search = " << model_->num_active_particles << endl;
	int count = model_->num_active_particles;
	vector<State*> particles = belief_->Sample(Globals::config.n_particles);
	// cout << "Sampled particles" << endl;
	for (int i=0; i<particles.size(); i++) {
		assert(particles[i]->scenario_id == i);
		// cout << *particles[i] << endl;
	}

	// cout << "# vnodes before search = " << VNode::num_vnodes << endl;
	// cout << "# qnodes before search = " << QNode::num_qnodes << endl;

	statistics_ = SearchStatistics();
	root_ = new VNode(move(particles));
	statistics_.num_tree_particles += root_->particles().size();

	const_cast<DSPOMDP*>(model_)->scenario_lower_bound()->Reset();

	HSVISearch();
	//HSVIBSearch();
	cout << "# active particles after search = " << model_->num_active_particles << endl;
	cout << "tree particles " << statistics_.num_tree_particles << endl;
	cout << (model_->num_active_particles - count) << " particles created during search" << endl;

	logi("Freeing particles...");
	count = model_->num_active_particles;
	root_->Free(*model_);
	cout << (count - model_->num_active_particles) << " particles destroyed" << endl;
	cout << "# active particles after destructing tree = " << model_->num_active_particles << endl;
	logi("Done!");

	// PrintLocs();
	cout << statistics_ << endl;

	if (VERBOSITY >= INFO) {
		root_->PrintTree();
	}

  if (Globals::config.pruning_constant) {
		int pruned_action;
		double pruned_value;

    VNode* pruned = Prune(root_, pruned_action, pruned_value);

		pruned->Free(*model_);
		//cout << "Unpruned tree " << root_->Size() << endl;
		//root_->PrintTree();
		cout << pruned_action << " " << pruned_value << endl;
		cout << "Pruned tree " << pruned->Size() << endl;
		//pruned->PrintTree();
		delete pruned;
		delete root_;

		return pruned_action;
  } else  {
		// root_->PrintTree(1);
		int astar = OptimalAction(root_);
		//lb_.Learn(root_);
		delete root_;
		return astar;
	}
}

VNode* DESPOT::Prune(VNode* vnode, int& pruned_action, double& pruned_value) {
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

	if (nustar < Discount(vnode->depth()) * vnode->default_move().value) {
		nustar = Discount(vnode->depth()) * vnode->default_move().value;
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

QNode* DESPOT::Prune(QNode* qnode, double& pruned_value) {
	QNode* pruned_q = new QNode(NULL, qnode->edge());
	pruned_value = Discount(qnode->parent()->depth()) * qnode->step_reward - Globals::config.pruning_constant;
	auto& children = qnode->children();
	for (auto& it : children) {
		int astar;
		double nu;
		VNode* pruned_v = Prune(it.second, astar, nu);
		if (nu == Discount(it.second->depth()) * it.second->default_move().value) {
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

int DESPOT::OptimalAction(VNode* vnode) {
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

double DESPOT::DiscountedGap(VNode* vnode) {
	if (vnode == NULL)
		return Globals::NEG_INFTY;

	return Discount(vnode->depth()) * (vnode->upper_bound() - vnode->lower_bound());
}

double DESPOT::WEU(VNode* vnode) {
	return WEU(vnode, 0.95);
}

double DESPOT::WEU(VNode* vnode, double xi) {
	VNode* root = vnode;
	while(root->parent() != NULL) {
		root = root->parent()->parent();
	}
	return Discount(-vnode->depth()) * (DiscountedGap(vnode) - xi * Weight(vnode) * DiscountedGap(root));
}

VNode* DESPOT::SelectBestWEUNode(QNode* qnode) {
	double gapstar = Globals::NEG_INFTY;
	VNode* vstar = NULL;
	for (auto& it : qnode->children()) {
		VNode* vnode = it.second;

		double gap = WEU(vnode);
		if (gap >= gapstar) {
			gapstar = gap;
			vstar = vnode->vstar;
		}
	}
	return vstar;
}

QNode* DESPOT::SelectBestUpperBoundNode(VNode* vnode) {
	int astar = -1;
	double upperstar = Globals::NEG_INFTY;
	for (int action = 0; action<vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);

		if (qnode->upper_bound() > upperstar) {
			upperstar = qnode->upper_bound();
			astar = action;
		}
	}
	return vnode->Child(astar);
}

double DESPOT::Weight(VNode* vnode) {
	return State::Weight(vnode->particles());
}

double DESPOT::Weight(QNode* qnode) {
	double weight = 0;
	for (auto& it : qnode->children())
		weight += Weight(it.second);
	return weight;
}

void DESPOT::Update(VNode* vnode) {
	if (vnode->IsLeaf()) return;

	double lower = Globals::NEG_INFTY;
	double upper = Globals::NEG_INFTY;

	// double gapstar = Globals::NEG_INFTY;
	// VNode* vstar = NULL;
	for (int action = 0; action<vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);

		/*
		if (upper > qnode->upper_bound()) { // Make sense?
			upper = qnode->upper_bound();
			vstar = qnode->vstar;
		}
		*/

		lower = max(lower, qnode->lower_bound());
		upper = max(upper, qnode->upper_bound());

		/*
		double gap = DiscountedGap(qnode->vstar);
		if (gap > gapstar) {
			gapstar = gap;
			vstar = qnode->vstar;
		}
		*/
	}

	if (lower > vnode->lower_bound())
		vnode->lower_bound(lower);
	if (upper < vnode->upper_bound())
		vnode->upper_bound(upper);
	// vnode->vstar = vstar;
}

void DESPOT::Update(QNode* qnode) {
	double lower = qnode->step_reward;
	double upper = qnode->step_reward;

	//double gapstar = Globals::NEG_INFTY;
	//VNode* vstar = NULL;
	for (auto& it : qnode->children()) {
		VNode* vnode = it.second;

		lower += Discount() * vnode->lower_bound();
		upper += Discount() * vnode->upper_bound();

		/*
		double gap = DiscountedGap(vnode->vstar);
		if (gap >= gapstar) {
			gapstar = gap;
			vstar = vnode->vstar;
		}
		*/
	}

	if (lower > qnode->lower_bound())
		qnode->lower_bound(lower);
	if (upper < qnode->upper_bound())
		qnode->upper_bound(upper);

	// qnode->vstar = vstar;
}

void DESPOT::Backup(VNode* vnode) {
	int iter = 0;
	logi("- Backup ", vnode, " at depth ", vnode->depth());
	while(true) {
		logd(" Iter ", iter++, " ", vnode);

		Update(vnode);
		logd(" Updated V-node vstar = ", vnode->vstar);

		QNode* parentq = vnode->parent();
		if (parentq == NULL)
			break;

		Update(parentq);
		logd(" Updated Q-node to (", parentq->lower_bound(), ", ", parentq->upper_bound(), ")");

		vnode = parentq->parent();
	}
	logi("* Backup complete!");
}

void DESPOT::Expand(VNode* vnode, const DSPOMDP* model, RandomStreams& streams,
		History& history) {
	vector<QNode*>& children = vnode->children();
	logi("- Expanding vnode ", vnode);
	for (int action = 0; action<model->NumActions(); action++) {
		logi(" Action ", action);
		QNode* qnode = new QNode(vnode, action);
		children.push_back(qnode);

		Expand(qnode, model, streams, history);
	}
	logi("* Expansion complete!");
}

void DESPOT::Expand(QNode* qnode, const DSPOMDP* model, RandomStreams& streams,
		History& history) {
	VNode* parent = qnode->parent();
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
		// Track(concat(copy), "expand->copy");

		logd(" Before step: ", *copy);

		bool terminal = model->Step(*copy,
				streams.Entry(copy->scenario_id, parent->depth()), qnode->edge(), reward, obs);

		step_reward += reward * copy->weight;

		logd(" After step: ", *copy, " ", reward * copy->weight, " ", reward, " ", copy->weight);

		if (!terminal) {
			partitions[obs].push_back(copy);
		} else {
			model->Free(copy);
		}
	}

	double lower_bound = step_reward;
	double upper_bound = step_reward;

	double gapstar = Globals::NEG_INFTY;
	VNode* vstar = NULL;
	// Create new belief nodes 
	for (auto& it : partitions) {
		uint64_t obs = it.first;
		logi(" Creating node for obs ", obs);
		VNode* vnode = new VNode(move(partitions[obs]), parent->depth()+1, qnode, obs);
		logi(" New node created!");
		children[obs] = vnode;

		history.Add(qnode->edge(), obs);
		// cout << "Init lower bound for " << qnode->edge() << " " << obs << endl;
		InitLowerBound(vnode, model, streams, history);
		InitUpperBound(vnode, model, streams, history);
		history.RemoveLast();

		lower_bound += Discount() * vnode->lower_bound();
		upper_bound += Discount() * vnode->upper_bound();

		double gap = DiscountedGap(vnode->vstar);
		if (gap > gapstar) {
			gapstar = gap;
			vstar = vnode->vstar;
		}
	}

	qnode->step_reward = step_reward;
	qnode->lower_bound(lower_bound);
	qnode->upper_bound(upper_bound);
	qnode->vstar = vstar;

	qnode->default_value = lower_bound; // for debugging
}

