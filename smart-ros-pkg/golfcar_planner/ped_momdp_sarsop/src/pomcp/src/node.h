#ifndef NODE_H
#define NODE_H

#include "beliefstate.h"
#include <stdio.h>
#include "utils.h"
#include <iostream>
#include <map>

using namespace std;

class HISTORY;
class SIMULATOR;
class QNODE;
class VNODE;

//typedef long OBS_TYPE;
typedef long long OBS_TYPE;

//-----------------------------------------------------------------------------
// Efficient computation of value from alpha vectors
// Only used for explicit POMDPs
struct ALPHA
{
	std::vector<double> AlphaSum;
	double MaxValue;
};

//-----------------------------------------------------------------------------

template<class COUNT>
class VALUE
{
public:

	void Set(double count, double value)
	{
		Count = count;
		Total = value * count;
	}

	void Add(double totalReward)
	{
		Count += 1.0;
		Total += totalReward;
	}

	void Add(double totalReward, COUNT weight)
	{
		Count += weight;
		Total += totalReward * weight;
	}

	double GetValue() const
	{
		return Count == 0 ? Total : Total / Count;
	}

	COUNT GetCount() const
	{
		return Count;
	}

private:

	COUNT Count;
	double Total;
};

//-----------------------------------------------------------------------------

class VNODE : public MEMORY_OBJECT
{
public:
	static VNODE* NULL_NODE;

	VALUE<int> Value;

	void Initialise();
	static VNODE* Create();
	static void Free(VNODE* vnode, const SIMULATOR& simulator);
	static void FreeAll();

	QNODE& Child(int c) { return Children[c]; }
	const QNODE& Child(int c) const { return Children[c]; }
	BELIEF_STATE& Beliefs() { return BeliefState; }
	const BELIEF_STATE& Beliefs() const { return BeliefState; }

	void SetChildren(int count, double value);
	static void Reset(VNODE*vnode);
	static void PartFree(VNODE* vnode,int,int, const SIMULATOR& simulator);
	void FreeNode(VNODE*vnode,const SIMULATOR&simulator);

	void DisplayValue(HISTORY& history, int maxDepth, std::ostream& ostr) const;
	void DisplayPolicy(HISTORY& history, int maxDepth, std::ostream& ostr) const;

	static int NumChildren;
	
	int counter;
	bool inserted;
	int visit_count;
	bool merged;
	int visited;
	VNODE*merge_node;
	int  merge_index;

	std::vector<QNODE> Children;
	static MEMORY_POOL<VNODE> VNodePool;
private:

	
	BELIEF_STATE BeliefState;

};


//-----------------------------------------------------------------------------

class QNODE
{
public:
	VALUE<int> Value;
	VALUE<double> AMAF;

	void Initialise();

	VNODE*& Child(OBS_TYPE c) {
		map<OBS_TYPE, VNODE*>::iterator it = Children.find(c);
		if(it == Children.end()) {
			Children[c] = 0;
			//cout << &Children[c] << endl;
			return Children[c];
		} else
			return it->second;
	}
	/*
	VNODE* Child(OBS_TYPE c) const {
		map<OBS_TYPE, VNODE*>::const_iterator it = Children.find(c);
		if(it == Children.end()) {
			return VNODE::NULL_NODE;
		} else
			return it->second;
	}
	*/
	//VNODE*& Child(OBS_TYPE c) { return Children[c]; }
	//VNODE* Child(OBS_TYPE c) const { return Children[c]; }
	ALPHA& Alpha() { return AlphaData; }
	const ALPHA& Alpha() const { return AlphaData; }

	void DisplayValue(HISTORY& history, int maxDepth, std::ostream& ostr) const;
	void DisplayPolicy(HISTORY& history, int maxDepth, std::ostream& ostr) const;

	static long long NumChildren;


	//std::vector<VNODE*> Children;
	std::map<OBS_TYPE, VNODE*> Children;
	private:
	ALPHA AlphaData;

friend class VNODE;
};

#endif // NODE_H
