#include "pocman.h"
#include "utils.h"
#include "math.h"
#include <algorithm>
#include <fstream>

using namespace std;
using namespace UTILS;

TIGER::TIGER()
{
	Discount=0.95;
	NumObservations=2;
	RewardRange=100;
	NumActions=3;
}


STATE*TIGER::Copy(const STATE &state) const
{
	const TIGER_STATE& tigerstate=safe_cast<const TIGER_STATE&>(state);
	TIGER_STATE*newstate=MemoryPool.Allocate();
	*newstate=tigerstate;
	return newstate;
}

void TIGER::FreeState(STATE* state) const
{
    TIGER_STATE* tigerstate = safe_cast<TIGER_STATE*>(state);
    MemoryPool.Free(tigerstate);
}

bool TIGER::Same(const BELIEF_STATE& beliefState1,const BELIEF_STATE& beliefState2,double & distance)
{
	const TIGER_STATE*tigerstate1;
	const TIGER_STATE*tigerstate2;
	int g1=0,g2=0;
	for(int i=0;i<beliefState1.GetNumSamples();i++)
	{
		tigerstate1=safe_cast<const TIGER_STATE*>(beliefState1.GetSample(i));
		if(tigerstate1->tigerpos==0) g1++;
	}
	for(int i=0;i<beliefState2.GetNumSamples();i++)
	{
		tigerstate2=safe_cast<const TIGER_STATE*>(beliefState2.GetSample(i));
		if(tigerstate2->tigerpos==0) g2++;
	}
	double p1=(g1+0.0)/beliefState1.GetNumSamples();
	double p2=(g2+0.0)/beliefState2.GetNumSamples();
	distance=fabs(p1-p2);
	if(distance>0.05) return false;
	return true;
}
VNODE* TIGER::Find(VNODE*v1,const HISTORY &h)
{
	bool found=false;
	int i;
	double distance;
	double min_dist=1;
	int min_index=-1;
	for(i=0;i<vnode_list.size();i++)
	{
		//if(vnode_list[i]==v1) 	return 0;

		if(Same(v1->Beliefs(),vnode_list[i]->Beliefs(),distance))
		{
			found=true;
			if(distance<min_dist)
			{
				min_index=i;
				min_dist=distance;
			}
		}
	}
	if(found)
	{
			return vnode_list[min_index];
	}
	else
	{
			vnode_list.push_back(v1);
			history_list.push_back(h);
			v1->inserted=true;
			return v1;
	}
	
}
VNODE* TIGER::MapBelief(VNODE*vn,VNODE*r,const HISTORY & h,int historyDepth)
{
	//return vn;
	if(vn==0) 	return 0;
	if(vn->inserted) return vn;
	BELIEF_STATE&beliefState=vn->Beliefs();
	//if(beliefState.GetNumSamples()<1000000)   	return vn;
	if(beliefState.GetNumSamples()<30)   
	{
		//VNODE*parent;
		//if(
		return vn;
	}
/*

	else if(beliefState.GetNumSamples()<20)
	{

		for(int i=0;i<20;i++)
		{
			VNODE*prt=r;
			for(int i=historyDepth;i<h.Size()-1;i++)
			{
				prt=prt->Child(h[i]._Action).Child(h[i]._Observation);
			}
			//VNODE*prt=vn.GetParent();
			STATE*prt_state=prt->Beliefs().CreateSample(*this);
			double rwd;
			int obs;
			Step(*prt_state,h.Back()._Action,obs,rwd);
			if(prt->Child(h.Back()._Action).Child(obs))
				prt->Child(h.Back()._Action).Child(obs)->Beliefs().AddSample(prt_state);
			//vn->Beliefs().AddSample(prt_state);
		}

		return vn;

	}*/
	VNODE*nn=Find(vn,h);
	/*
	if(nn==vn)
	{
		vn->merged=true;
		vn->merge_node=nn;
	}*/
	return nn;	

	//BeliefPool.Find(beliefState);
}
void TIGER::DisplayBeliefs(const BELIEF_STATE& beliefState, std::ostream&) const
{
	int leftnum=0;
	for (int i=0;i<beliefState.GetNumSamples();i++)
	{
    	const TIGER_STATE* tigerstate = safe_cast<const TIGER_STATE*>(beliefState.GetSample(i));	
    	if(tigerstate->tigerpos==0) 	leftnum++;		
	}
	cout<<"current tiger belief : "<<(leftnum+0.0)/beliefState.GetNumSamples()<<" "<<1-(leftnum+0.0)/beliefState.GetNumSamples()<<endl;
	
}
STATE* TIGER::CreateStartState() const
{
    TIGER_STATE* tigerstate = MemoryPool.Allocate();
    tigerstate->tigerpos=rand()%2;
    return tigerstate;
}

bool TIGER::Step(STATE& state,int action, int & observation,double&reward) const
{
	TIGER_STATE& tigerstate=safe_cast<TIGER_STATE&>(state);
	reward=0;
	double noise=0.15;
	if(action==LISTEN)
	{
		if(Bernoulli(noise))
		{
			observation=1-tigerstate.tigerpos;
		}
		else
		{
			observation=tigerstate.tigerpos;
		}
		reward=-1;
		return false;
	}
	else if(action==OPEN_LEFT)
	{
		if(tigerstate.tigerpos==1) reward=10;
		else 					   reward=-100;
		if(Bernoulli(0.5)) 	tigerstate.tigerpos=0;
		else 				tigerstate.tigerpos=1;
		if(Bernoulli(0.5)) 	observation=0;
		else 				observation=1;
		return false;
	}
	else
	{
		if(tigerstate.tigerpos==0) reward=10;
		else    				   reward=-100;
		if(Bernoulli(0.5)) 	tigerstate.tigerpos=0;
		else 				tigerstate.tigerpos=1;
		if(Bernoulli(0.5)) 	observation=0;
		else 				observation=1;
		return false;
	}
}


double TIGER::TransFn(int s1,int a,int s2)
{
	if(a==LISTEN)   //no change
	{
		return (s1==s2);
	}
	else 	  //open 
	{
		return 0.5;
	}
}

double TIGER::GetReward(int s,int a)
{
	if(a==LISTEN) 	return -1;
	else if(a==OPEN_LEFT)
	{
		if(s==1)   return 10;//right
		else       return -100;
	}
	else
	{
		if(s==1) 	return -100;
		else 		return 10;
	}
}
bool TIGER::Init_QMDP()
{
	for(int lv=0;lv<1000;lv++)   //num of iterations
	{
		for(int s1=0;s1<2;s1++)   //s1=0 left 
		{
			for(int j=0;j<NumActions;j++)
			{
				qValue[s1][j]=0;
				double prob=0;
				for(int s2=0;s2<2;s2++)
				{
					prob+=TransFn(s1,j,s2);
					qValue[s1][j]+=TransFn(s1,j,s2)*Value[s2];
				}
				if(prob!=0)
				{
					qValue[s1][j]/=prob;
				}
				qValue[s1][j]*=Discount;
				if(qValue[s1][j]+GetReward(s1,j)>Value[s1]) 	Value[s1]=qValue[s1][j]+GetReward(s1,j);
			}
			//Value[s1]=MaxQ(s1)+GetReward(s1);
		}
	}
	prob_l=0.5;
	cout<<"QMDP Initialization Finished!"<<endl;
}

void TIGER::DisplayQMDP()
{
	cout<<"QMDP Values : "<<Value[0]<<" "<<Value[1]<<endl;
}

void TIGER::UpdateBelief(int a,int o)
{
	if(a!=0)   
	{
		prob_l=0.5;
		return;
	}
	double obs_l=prob_l*0.85+(1-prob_l)*0.15;
	double prob_b,prob_ab;
	if(o==0)
	{
			prob_b=obs_l;
			prob_ab=prob_l*0.85;
	}
	else
	{
			prob_b=1-obs_l;
			prob_ab=prob_l*0.15;
	}
	prob_l=prob_ab/prob_b;
}

int TIGER::QMDP_SelectAction()
{
	int best_a=-1;
	double best_value=-10000;
	for(int a=0;a<NumActions;a++)
	{
		double q=(qValue[0][a]+GetReward(0,a))*prob_l+(qValue[1][a]+GetReward(1,a))*(1-prob_l);
		if(q>best_value)
		{
			best_value=q;
			best_a=a;
		}
	}
	return best_a;
}


void TIGER::GeneratePreferred(const STATE& state, const HISTORY& history,
    vector<int>& actions, const STATUS& status) const
{
	//cout<<"using knowledge!!!!!!!!!!!!!!!!"<<endl;
	int pos=0;
	for(int t=history.Size()-1;t>=0;t--)
	{
		if(history[t]._Action!=LISTEN) break;
		if(history[t]._Observation==0) pos--;
		else 						  pos++;
	}
	if(pos>=2)  actions.push_back(OPEN_LEFT);
	else if(pos<=-2) actions.push_back(OPEN_RIGHT);
	else 	actions.push_back(LISTEN);
}

