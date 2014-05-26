#include "despot.h"
#include "approx_aems.h"

APPROXAEMS::APPROXAEMS(const DSPOMDP* model, Belief* belief)
	: Solver(model, belief),
	root_(NULL)
{
}

void APPROXAEMS::APPROXAEMSSearch() {
  clock_t begin = clock();
	InitLowerBound(root_, model_);
	InitUpperBound(root_, model_);
	statistics_.initial_lb = root_->lower_bound(); 
	statistics_.initial_ub = root_->upper_bound();

	int num_active_particles = model_->num_active_particles;
	do { 
		VNode* promising_node = root_->vstar;
		assert(promising_node->IsLeaf());

		// cout << promising_node << endl;
		// root_->PrintTree();
		// cout << endl;

		clock_t start = clock();
		Expand(promising_node, model_);
		statistics_.time_node_expansion += (clock() - start) / CLOCKS_PER_SEC;
		Backup(promising_node);

		statistics_.num_trials ++;
		statistics_.num_expanded_nodes ++;

		if(VERBOSITY >= DEBUG) {
			root_->PrintTree();
			cout << endl;
		}
  } while ((double)(clock() - begin) / CLOCKS_PER_SEC < Globals::config.time_per_move
			&& (root_->upper_bound() - root_->lower_bound()) > 1e-6
			&& root_->vstar != NULL);

	statistics_.num_tree_particles = model_->num_active_particles - num_active_particles;
	statistics_.num_tree_nodes = root_->Size();

	statistics_.final_lb = root_->lower_bound(); 
	statistics_.final_ub = root_->upper_bound();
	statistics_.time_search = (double)(clock() - begin) / CLOCKS_PER_SEC;
}

int APPROXAEMS::Search() {
	vector<State*> particles = belief_->Sample(Globals::config.n_particles);

	root_ = new VNode(move(particles));
	statistics_ = SearchStatistics();

	cout << "# active particles before search  = " << model_->num_active_particles << endl;
	APPROXAEMSSearch();
	cout << "# active particles after search  = " << model_->num_active_particles << endl;

	logi("Freeing particles...");
	root_->Free(*model_);
	logi("Done!");
	cout << "# active particles after destructing tree  = " << model_->num_active_particles << endl;

	cout << statistics_ << endl;

	if(VERBOSITY >= INFO) {
		root_->PrintTree();
	}

	int astar = OptimalAction(root_);
	return astar;
}

int APPROXAEMS::OptimalAction(VNode* vnode) {
	double lower = Globals::NEG_INFTY;
	int action = -1;
	for(int a=0; a<vnode->children().size(); a++) {
		QNode* qnode = vnode->Child(a);
		if(qnode->lower_bound() > lower) {
			lower = qnode->lower_bound();
			action = a;
		}
	}
	return action;
}

double APPROXAEMS::ApproxError(VNode* vnode) {
	if(vnode == NULL)
		return Globals::NEG_INFTY;

	return vnode->likelihood * Discount(vnode->depth()) * (vnode->upper_bound() - vnode->lower_bound());
}

void APPROXAEMS::Update(VNode* vnode) {
	if(vnode->IsLeaf()) return;

	double lower = Globals::NEG_INFTY;
	double upper = Globals::NEG_INFTY;
	int astar = -1;

	for(int action = 0; action<vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);

		if(qnode->upper_bound() > upper) {
			astar = qnode->edge();
		}

		lower = max(lower, qnode->lower_bound());
		upper = max(upper, qnode->upper_bound());
	}

	if(lower > vnode->lower_bound())
		vnode->lower_bound(lower);
	if(upper < vnode->upper_bound())
		vnode->upper_bound(upper);

	if(vnode->lower_bound() > vnode->upper_bound()) {
		vnode->PrintTree();
		exit(0);
	}

	//cout << "astar is " << astar << endl;
	vnode->vstar = vnode->Child(astar)->vstar;
}

void APPROXAEMS::Update(QNode* qnode) {
	double lower = qnode->step_reward;
	double upper = qnode->step_reward;

	double aestar = Globals::NEG_INFTY;
	VNode* vstar = NULL;
	for(auto& it : qnode->children()) {
		VNode* vnode = it.second;

		lower += Discount() * vnode->lower_bound();
		upper += Discount() * vnode->upper_bound();

		double ae = ApproxError(vnode->vstar);
		if(ae >= aestar) {
			aestar = ae;
			vstar = vnode->vstar;
		}
	}

	if(lower > qnode->lower_bound())
		qnode->lower_bound(lower);
	if(upper < qnode->upper_bound())
		qnode->upper_bound(upper);
	qnode->vstar = vstar;
}

void APPROXAEMS::Backup(VNode* vnode) {
	int iter = 0;
	logi("- Backup ", vnode, " at depth ", vnode->depth());
	while(true) {
		logd(" Iter ", iter++, " ", vnode);

		Update(vnode);
		logd(" Updated V-node vstar = ", vnode->vstar);

		QNode* parentq = vnode->parent();
		if(parentq == NULL)
			break;

		Update(parentq);
		logd(" Updated Q-node to (", parentq->lower_bound(), ", ", parentq->upper_bound(), ")");

		vnode = parentq->parent();
	}
	logi("* Backup complete!");
}

void APPROXAEMS::InitLowerBound(VNode* vnode, const DSPOMDP* model) {
	double value = model->LowerBound(vnode->particles()).value;
	vnode->lower_bound(value);
}

void APPROXAEMS::InitUpperBound(VNode* vnode, const DSPOMDP* model) {
	double value = 0;
	for(auto& particle : vnode->particles())
		value += particle->weight * model->UpperBound(*particle);
	vnode->upper_bound(value);
}

void APPROXAEMS::Expand(VNode* vnode, const DSPOMDP* model) {
	vector<QNode*>& children = vnode->children();
	logi("- Expanding vnode ", vnode);
	for(int action = 0; action<model->NumActions(); action++) {
		logi(" Action ", action);
		QNode* qnode = new QNode(vnode, action);
		children.push_back(qnode);

		Expand(qnode, model);
	}
	logi("* Expansion complete!");
}

void APPROXAEMS::Expand(QNode* qnode, const DSPOMDP* model) {
	VNode* parent = qnode->parent();
	int action = qnode->edge();
	map<uint64_t, VNode*>& children = qnode->children();

	const vector<State*>& particles = parent->particles();

	double step_reward = 0;

	// Sample observations
	map<uint64_t, double> obss;
	uint64_t obs;
	double reward;
	logi(" ", particles.size(), " particles");
	for (auto& particle: particles) {
		State* copy = model->Copy(particle);
		bool terminal = model->Step(*copy, action, reward, obs);
		step_reward += copy->weight * reward;
		model->Free(copy);

		if (!terminal) {
			obss[obs] += 1.0 / particles.size();
		}
	}

	double lower_bound = step_reward;
	double upper_bound = step_reward;

	double errorstar = Globals::NEG_INFTY;
	VNode* vstar = NULL;
	// Create new belief nodes 
	for (auto& it : obss) {
		uint64_t obs = it.first;
		double weight = it.second;
		logi(" Creating node for obs ", obs, " with weight ", weight);
		VNode* vnode = new VNode(Tau(model, particles, action, obs) , parent->depth()+1, qnode, obs);
		vnode->likelihood = qnode->parent()->likelihood * weight;
		logi(" New node created!");
		children[obs] = vnode;

		InitLowerBound(vnode, model);
		InitUpperBound(vnode, model);

		lower_bound += weight * Discount() * vnode->lower_bound();
		upper_bound += weight * Discount() * vnode->upper_bound();

		double error = ApproxError(vnode);
		if(error > errorstar) {
			errorstar = error;
			vstar = vnode;
		}
	}

	qnode->step_reward = step_reward;
	qnode->lower_bound(lower_bound);
	qnode->upper_bound(upper_bound);
	qnode->vstar = vstar;
}

vector<State*> APPROXAEMS::Tau(const DSPOMDP* model, const vector<State*>& particles, int action, uint64_t obs) {
	vector<State*> belief;
	double reward;
	uint64_t tmp_obs;
	double total_weight = 0;
	for(auto& particle : particles) {
		State* copy = model->Copy(particle);
		bool terminal = model->Step(*copy, action, reward, tmp_obs);
		double prob = model->ObsProb(obs, *copy, action);

		if(!terminal && prob) {
			copy->weight *= prob;
			total_weight += copy->weight;
			belief.push_back(copy);
			//cout << " created particle " << *copy << " " << prob << endl;
		} else {
			model->Free(copy);
		}
	}

	//cout << "inside tau " << belief.size() << " " << total_weight << endl;
	double weight_square_sum = 0;
	for(State* particle : belief) {
		particle->weight /= total_weight;
		weight_square_sum += particle->weight * particle->weight;
		//cout << particle->weight << endl;
	}

	// Resample if the effective number of particles is "small"
	double num_effective_particles = 1.0 / weight_square_sum;
	if(belief.size() > 0 && (num_effective_particles < belief.size() / 20.0)) {
		vector<State*> new_belief = Belief::Sample(belief.size(), belief, model);
		for(State* particle : belief)
			model->Free(particle);

		belief = new_belief;
	}

	return belief;
}

void APPROXAEMS::Update(int act, uint64_t obs) {
	logi("- Updating belief with action action ", act, " and observation ", obs);
	belief_->Update(act, obs);
	logi("  Done");
}
