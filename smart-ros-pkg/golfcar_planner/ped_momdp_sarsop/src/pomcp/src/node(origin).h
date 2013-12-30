#ifndef NODE_H
#define NODE_H

#include "beliefstate.h"
#include "utils.h"
#include <iostream>

class HISTORY;
class SIMULATOR;
class QNODE;
class VNODE;

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

class QNODE
{
public:

    VALUE<int> Value;
    VALUE<double> AMAF;

    void Initialise();

    VNODE*& Child(int c) { return Children[c]; }
    VNODE* Child(int c) const { return Children[c]; }
    ALPHA& Alpha() { return AlphaData; }
    const ALPHA& Alpha() const { return AlphaData; }

    void DisplayValue(HISTORY& history, int maxDepth, std::ostream& ostr) const;
    void DisplayPolicy(HISTORY& history, int maxDepth, std::ostream& ostr) const;
    static int NumChildren;
	std::vector<VNODE*> Children;
	//void Copy(QNODE*qnode,const SIMULATOR & simulator);
private:

    
    ALPHA AlphaData;

friend class VNODE;
};

//-----------------------------------------------------------------------------

class VNODE : public MEMORY_OBJECT
{
public:

    VALUE<int> Value;

    void Initialise();
    static VNODE* Create();
    static void Free(VNODE* vnode, const SIMULATOR& simulator);
	static void PartFree(VNODE* vnode,int,int, const SIMULATOR& simulator);
	void FreeNode(VNODE*vnode,const SIMULATOR&simulator);
    static void FreeAll();
	static void Reset(VNODE*vnode);
    QNODE& Child(int c) { return Children[c]; }
    const QNODE& Child(int c) const { return Children[c]; }
    BELIEF_STATE& Beliefs() { return BeliefState; }
    const BELIEF_STATE& Beliefs() const { return BeliefState; }

    void SetChildren(int count, double value);

    void DisplayValue(HISTORY& history, int maxDepth, std::ostream& ostr) const;
    void DisplayPolicy(HISTORY& history, int maxDepth, std::ostream& ostr) const;
	MEMORY_POOL<VNODE> GetPool ()  const {return VNodePool;}


    static int NumChildren;

	bool merged;
	int visited;
	VNODE*merge_node;
	int  merge_index;
	std::vector<QNODE> Children;
	//void Copy(VNODE*vnode,const SIMULATOR & simulator);
    static MEMORY_POOL<VNODE> VNodePool;

	int counter;
	bool inserted;
	int visit_count;
private:

    BELIEF_STATE BeliefState;

};

#endif // NODE_H
