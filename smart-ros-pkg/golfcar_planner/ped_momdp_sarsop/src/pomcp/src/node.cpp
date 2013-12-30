#include "node.h"
#include "history.h"
#include "utils.h"

using namespace std;

//-----------------------------------------------------------------------------

long long QNODE::NumChildren = 0;

void QNODE::Initialise()
{
		//cout << " initialize " << NumChildren << endl;
		// MODIFIED: This does not work if the number of observations is large.
		/*
    assert(NumChildren);
    //Children.resize(NumChildren);
    for (int observation = 0; observation < QNODE::NumChildren; observation++)
        Children[observation] = 0;
		*/
		Children.clear();
    AlphaData.AlphaSum.clear();
}

void QNODE::DisplayValue(HISTORY& history, int maxDepth, ostream& ostr) const
{
    history.Display(ostr);
    ostr << ": " << Value.GetValue() << " (" << Value.GetCount() << ")\n";
    if (history.Size() >= maxDepth)
        return;

		/* TODO
    for (int observation = 0; observation < NumChildren; observation++)
    {
        if (Children[observation])
        {
            history.Back().Observation = observation;
            Children[observation]->DisplayValue(history, maxDepth, ostr);
        }
    }
		*/
}

void QNODE::DisplayPolicy(HISTORY& history, int maxDepth, ostream& ostr) const
{
    history.Display(ostr);
    ostr << ": " << Value.GetValue() << " (" << Value.GetCount() << ")\n";
    if (history.Size() >= maxDepth)
        return;

		/* TODO
    for (int observation = 0; observation < NumChildren; observation++)
    {
        if (Children[observation])
        {
            history.Back().Observation = observation;
            Children[observation]->DisplayPolicy(history, maxDepth, ostr);
        }
    }
		*/
}

//-----------------------------------------------------------------------------

MEMORY_POOL<VNODE> VNODE::VNodePool;

int VNODE::NumChildren = 0;

VNODE* VNODE::NULL_NODE = 0;

void VNODE::Initialise()
{
    assert(NumChildren);
    Children.resize(VNODE::NumChildren);
    for (int action = 0; action < VNODE::NumChildren; action++)
        Children[action].Initialise();
}

VNODE* VNODE::Create()
{
    VNODE* vnode = VNodePool.Allocate();
	vnode->counter=1;
	vnode->inserted=false;
	vnode->merged=false;
	vnode->visited=0;
	vnode->visit_count=0;
    vnode->Initialise();
    return vnode;
}

void VNODE::Free(VNODE* vnode, const SIMULATOR& simulator) //MODIFIED
{
    vnode->BeliefState.Free(simulator);
    VNodePool.Free(vnode);
    for (int action = 0; action < VNODE::NumChildren; action++) {
			QNODE qnode = vnode->Child(action);
			map<OBS_TYPE, VNODE*> Children = qnode.Children;
			int count = 1;
			for(map<OBS_TYPE, VNODE*>::iterator it = Children.begin(); it != Children.end(); it++) {
				if(it->second) {
					Free(it->second, simulator);
				}
			}
		}

		/*
    vnode->BeliefState.Free(simulator);
    VNodePool.Free(vnode);
    for (int action = 0; action < VNODE::NumChildren; action++)
        for (int observation = 0; observation < QNODE::NumChildren; observation++)
            if (vnode->Child(action).Child(observation))
                Free(vnode->Child(action).Child(observation), simulator);
								*/
}

void VNODE::Reset(VNODE*vnode)
{
	vnode->inserted=false;
	vnode->counter=1;
	vnode->merged=false;
	vnode->visited=0;
	//vnode->visit_count=0;
    for (int action = 0; action < VNODE::NumChildren; action++)
        for (int observation = 0; observation < QNODE::NumChildren; observation++)
            if (vnode->Child(action).Child(observation))
                Reset(vnode->Child(action).Child(observation));
}

void VNODE::PartFree(VNODE*vnode,int act,int obs,const SIMULATOR&simulator)
{
    vnode->BeliefState.Free(simulator);
    VNodePool.Free(vnode);
	//cout<<VNodePool.GetFreeListSize()<<endl;
    for (int action = 0; action < VNODE::NumChildren; action++)
        for (int observation = 0; observation < QNODE::NumChildren; observation++)
		{
			if(action==act&&observation==obs) continue;
            if (vnode->Child(action).Child(observation))
                Free(vnode->Child(action).Child(observation), simulator);
		}

}


void VNODE::FreeNode(VNODE*vnode,const SIMULATOR&simulator)
{
	vnode->BeliefState.Free(simulator);
    VNodePool.Free(vnode);
}

void VNODE::FreeAll()
{
	VNodePool.DeleteAll();
}

void VNODE::SetChildren(int count, double value)
{
    for (int action = 0; action < NumChildren; action++)
    {
        QNODE& qnode = Children[action];
        qnode.Value.Set(count, value);
        qnode.AMAF.Set(count, value);
    }
}

void VNODE::DisplayValue(HISTORY& history, int maxDepth, ostream& ostr) const
{
    if (history.Size() >= maxDepth)
        return;

    for (int action = 0; action < NumChildren; action++)
    {
        history.Add(action);
        Children[action].DisplayValue(history, maxDepth, ostr);
        history.Pop();
    }
}

void VNODE::DisplayPolicy(HISTORY& history, int maxDepth, ostream& ostr) const
{
    if (history.Size() >= maxDepth)
        return;

    double bestq = -Infinity;
    int besta = -1;
    for (int action = 0; action < NumChildren; action++)
    {
        if (Children[action].Value.GetValue() > bestq)
        {
            besta = action;
            bestq = Children[action].Value.GetValue();
        }
    }

    if (besta != -1)
    {
        history.Add(besta);
        Children[besta].DisplayPolicy(history, maxDepth, ostr);
        history.Pop();
    }
}

//-----------------------------------------------------------------------------
