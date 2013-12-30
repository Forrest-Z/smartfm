#include "node.h"
#include "history.h"
#include "utils.h"

using namespace std;

//-----------------------------------------------------------------------------

int QNODE::NumChildren = 0;

void QNODE::Initialise()
{
    assert(NumChildren);
    Children.resize(NumChildren);
    for (int observation = 0; observation < QNODE::NumChildren; observation++)
	{
		//Children[observation]=new VNODE*;
		Children[observation] = 0;
	}
    AlphaData.AlphaSum.clear();
}

void QNODE::DisplayValue(HISTORY& history, int maxDepth, ostream& ostr) const
{
    history.Display(ostr);
    ostr << ": " << Value.GetValue() << " (" << Value.GetCount() << ")\n";
    if (history.Size() >= maxDepth)
        return;

    for (int observation = 0; observation < NumChildren; observation++)
    {
        if (Children[observation])
        {
            history.Back()._Observation = observation;
            Children[observation]->DisplayValue(history, maxDepth, ostr);
        }
    }
}

void QNODE::DisplayPolicy(HISTORY& history, int maxDepth, ostream& ostr) const
{
    history.Display(ostr);
    ostr << ": " << Value.GetValue() << " (" << Value.GetCount() << ")\n";
    if (history.Size() >= maxDepth)
        return;

    for (int observation = 0; observation < NumChildren; observation++)
    {
        if (Children[observation])
        {
            history.Back()._Observation = observation;
            Children[observation]->DisplayPolicy(history, maxDepth, ostr);
        }
    }
}

/*
void QNODE::Copy(QNODE*qnode,const SIMULATOR & simulator)
{
	Value=qnode->Value;
	for(int i=0;i<NumChildren;i++)
	{
		if((*(qnode->Children[i]))==0)  
		{
			(*Children[i])=0;
		}
		else
		{
			Children[i]=qnode->Children[i];
			(*Children[i])=VNODE::Create();
			Children[i]->Copy(qnode->Children[i],simulator);
		}
	}
}*/

//-----------------------------------------------------------------------------

MEMORY_POOL<VNODE> VNODE::VNodePool;

int VNODE::NumChildren = 0;

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

void VNODE::Free(VNODE* vnode, const SIMULATOR& simulator)
{
	/*
	if(vnode->counter>1) 
	{
		--vnode->counter;
		return;
	}*/

    vnode->BeliefState.Free(simulator);
    VNodePool.Free(vnode);
	//cout<<VNodePool.GetFreeListSize()<<endl;
    for (int action = 0; action < VNODE::NumChildren; action++)
        for (int observation = 0; observation < QNODE::NumChildren; observation++)
            if (vnode->Child(action).Child(observation))
                Free(vnode->Child(action).Child(observation), simulator);
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

/*
void VNODE::Copy(VNODE*vnode,const SIMULATOR & simulator)
{
	BeliefState.Move(vnode->BeliefState);
	Value=vnode->Value;
	for(int action=0;action<VNODE::NumChildren;action++)
	{
		Children[action].Copy(&vnode->Children[action],simulator);
	}
}*/

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
	ostr<<"vn: "; 
	history.Display(ostr);
	ostr<<" "<<BeliefState.GetNumSamples();
	ostr<<endl;	

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
