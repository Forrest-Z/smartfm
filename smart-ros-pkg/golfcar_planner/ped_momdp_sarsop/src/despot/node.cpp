#include "node.h"
#include "despot.h"

VNode::VNode(vector<State*>&& particles, int depth, QNode* parent, uint64_t edge)
		: particles_(particles), belief_(NULL), depth_(depth), parent_(parent), edge_(edge),
		vstar(this), likelihood(1) {
	logi("Constructed vnode with ", particles_.size(), " particles");
	for (int i=0; i<particles_.size(); i++)
		logd(" ", i, " = ", *particles_[i]);
}

VNode::VNode(Belief* belief, int depth, QNode* parent, uint64_t edge)
		: belief_(belief), depth_(depth), parent_(parent), edge_(edge),
		vstar(this), likelihood(1) {
}

VNode::~VNode() { 
	for (auto& child : children_)
		delete child;
	children_.clear();

	if (belief_ != NULL)
		delete belief_;
}

Belief* VNode::belief() const { return belief_; }
const vector<State*>& VNode::particles() const { return particles_; }
int VNode::depth() const { return depth_; }
void VNode::parent(QNode* parent) { parent_ = parent; }
QNode* VNode::parent() { return parent_; }
uint64_t VNode::edge() { return edge_; }

vector<QNode*>& VNode::children() { return children_; }
QNode* VNode::Child(int action) { return children_[action]; }

int VNode::Size() const {
	int size = 1;
	for (auto& qnode : children_)
		size += qnode->Size();
	return size;
}

int VNode::PolicyTreeSize() const {
	if (children_.size() == 0)
		return 0;

	QNode* best = NULL;
	for (auto& child : children_) {
		if (best == NULL || child->lower_bound() > best->lower_bound())
			best = child;
	}
	return best->PolicyTreeSize();
}

void VNode::default_move(ValuedAction move) { default_move_ = move; }
ValuedAction VNode::default_move() const { return default_move_; }
void VNode::lower_bound(double value) { lower_bound_ = value; }
double VNode::lower_bound() const { return lower_bound_; }
void VNode::upper_bound(double value) { upper_bound_ = value; }
double VNode::upper_bound() const { return upper_bound_; }

bool VNode::IsLeaf() { return children_.size() == 0; }

void VNode::Free(const DSPOMDP& model) {
	for (auto& particle : particles_) {
		model.Free(particle);
		// Untrack(concat(particle));
	}

	for (int a=0; a<children().size(); a++) {
		QNode* qnode = Child(a);
		for (auto& it : qnode->children()) {
			it.second->Free(model);
		}
	}
}

void VNode::PrintPolicyTree(int depth, ostream& os) {
	if (depth != -1 && this->depth() > depth) return;

  vector<QNode*>& qnodes = children();
	if (qnodes.size() == 0) {
		int astar = this->default_move().action;
		os << this << "-a=" << astar << endl;
	} else {
		QNode* qstar = NULL;
		for (int a = 0; a < qnodes.size(); a++) {
			QNode* qnode = qnodes[a];
			if (qstar == NULL || qnode->lower_bound() > qstar->lower_bound()) {
				qstar = qnode;
			}
		}

		os << this << "-a=" << qstar->edge() << endl;

		vector<uint64_t> labels;
		map<uint64_t, VNode*>& vnodes = qstar->children();
		for (auto& it : vnodes)
			labels.push_back(it.first);

		for (int i=0; i<labels.size(); i++) {
			if (depth == -1 || this->depth()+1 <= depth) {
				os << repeat("|   ", this->depth()) << "| o=" << labels[i] << ": "; 
				qstar->Child(labels[i])->PrintPolicyTree(depth, os);
			}
		}
	}
}

void VNode::PrintTree(int depth, ostream& os) {
	if (depth != -1 && this->depth() > depth) return;

	os << this << "-"
		<< "("
		<< "l:" << this->lower_bound() 
		<< ", d:" << this->default_move().value
		<< ", u:" << this->upper_bound()
		// << ", "  << vstar 
		<< ", w:" << DESPOT::Weight(this)
		<< ", weu:" << DESPOT::WEU(this)
		//<< ", p:" << this->likelihood
		//<< ", wu:" << this->likelihood * DESPOT::DiscountedGap(this)
		<< ")" 
		// << " " << DESPOT::DiscountedGap(this) << " " << DESPOT::Weight(this)
		<< endl;

	/*
	if (this->IsLeaf()) {
		for (State* particle : this->particles()) {
			os << repeat("|   ", this->depth()) << *particle << endl;
		}
	}
	*/

  vector<QNode*>& qnodes = children();
  for (int a = 0; a < qnodes.size(); a++) {
		QNode* qnode = qnodes[a];

		vector<uint64_t> labels;
		map<uint64_t, VNode*>& vnodes = qnode->children();
		for (auto& it : vnodes)
			labels.push_back(it.first);

		os << repeat("|   ", this->depth()) 
			<< qnode << "-"
			<< "a=" << qnode->edge() << ": " 
			// << qnode << "-"
			<< "(l:" << qnode->lower_bound() 
			<< ", d:" << qnode->default_value
			<< ", u:" << qnode->upper_bound()
			// << ", " << qnode->vstar 
			<< ", r:" << qnode->step_reward 
			<< ")" 
			<< endl;

		for (int i=0; i<labels.size(); i++) {
			if (depth == -1 || this->depth()+1 <= depth) {
				os << repeat("|   ", this->depth()) << "| o=" << labels[i] << ": "; 
				qnode->Child(labels[i])->PrintTree(depth, os);
			}
		}
  }
}

/*---------------------------------------------------------------------------*/

QNode::QNode(VNode* parent, int edge) 
	: parent_(parent), edge_(edge), vstar(NULL) {
}


QNode::~QNode() {
	for (auto& it : children_) {
		delete it.second;
	}
	children_.clear();
}

void QNode::parent(VNode* parent) { parent_ = parent; }
VNode* QNode::parent() { return parent_; }
int QNode::edge() { return edge_; }
map<uint64_t, VNode*>& QNode::children() { return children_; }
VNode* QNode::Child(uint64_t obs) { return children_[obs]; }

int QNode::Size() const {
	int size = 0;
	for (auto& it : children_)
		size += it.second->Size();
	return size;
}

int QNode::PolicyTreeSize() const {
	int size = 0;
	for (auto& it : children_) {
		size += it.second->PolicyTreeSize();
	}
	return 1 + size;
}

void QNode::lower_bound(double value) { lower_bound_ = value; }
double QNode::lower_bound() const { return lower_bound_; }
void QNode::upper_bound(double value) { upper_bound_ = value; }
double QNode::upper_bound() const { return upper_bound_; }
