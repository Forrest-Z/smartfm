#include "mcts.h"
#include "testsimulator.h"
#include <math.h>
#include "battleship.h"
#include <algorithm>
#include "time.h"
#include <sys/time.h>
#include "boost/timer.hpp"


using namespace std;
using namespace UTILS;

//-----------------------------------------------------------------------------
int reuse_count;
int merge_count;
int root_count;
MCTS::PARAMS::PARAMS()
:  
	Verbose(0),
    MaxDepth(100),
    NumSimulations(200000),
    NumStartStates(10000),
    UseTransforms(false),
    NumTransforms(0),
    MaxAttempts(0),
    ExpandCount(1),
    ExplorationConstant(1),
    UseRave(false),
    RaveDiscount(1.0),
    RaveConstant(0.01),
    DisableTree(false),
	UseQmdp(false),
	UseTime(false),
	Reuse(false),
	LeaveValue(-1)
{
}

/*temporarily remove the const keyword for convienience*/
//MCTS::MCTS(const SIMULATOR& simulator, const PARAMS& params)
MCTS::MCTS(SIMULATOR& simulator, const PARAMS& params)
:   Simulator(simulator),
    Params(params),
    TreeDepth(0)
{

    VNODE::NumChildren = Simulator.GetNumActions();
    QNODE::NumChildren = Simulator.GetNumObservations();

    Root = ExpandNode(Simulator.CreateStartState());
	Rollout_Root=0;

    for (int i = 0; i < Params.NumStartStates; i++)
        Root->Beliefs().AddSample(Simulator.CreateStartState());

//	pedproblem_c->vnode_list.push_back(Root);
//	pedproblem_c->history_list.push_back(History);
	if(ModelParams::debug)
	{
   		 cout<<"Initial Beliefs"<<endl;
    	Simulator.DisplayBeliefs(Root->Beliefs(), cout);
	}
	o=0;
	l=0;
	r=0;
	sum=0;
}
void MCTS::ReSample(STATE*state)
{
	PEDESTRIAN_DYNAMIC_UTOWN_STATE*real_state=safe_cast<PEDESTRIAN_DYNAMIC_UTOWN_STATE*>(state);
	for (int i = 0; i < Root->Beliefs().GetNumSamples(); i++)
	{
		PEDESTRIAN_DYNAMIC_UTOWN_STATE*my_state=safe_cast<PEDESTRIAN_DYNAMIC_UTOWN_STATE*>(Root->Beliefs().GetSample(i));
		my_state->PedPos.X=real_state->PedPos.X;
		my_state->PedPos.Y=real_state->PedPos.Y;
		my_state->RobPos.Y=real_state->RobPos.Y;
	}
}
MCTS::MCTS(SIMULATOR& simulator, const PARAMS& params,STATE*state)
:   Simulator(simulator),
    Params(params),
    TreeDepth(0)
{
	rollout_out.open("rollout.txt");
    VNODE::NumChildren = Simulator.GetNumActions();
    QNODE::NumChildren = Simulator.GetNumObservations();

    Root = ExpandNode(Simulator.CreateStartState());
	Rollout_Root=0;

    for (int i = 0; i < Params.NumStartStates; i++)
	{
		PEDESTRIAN_DYNAMIC_UTOWN_STATE*new_state=safe_cast<PEDESTRIAN_DYNAMIC_UTOWN_STATE*>(Simulator.CreateStartState());
		PEDESTRIAN_DYNAMIC_UTOWN_STATE*real_state=safe_cast<PEDESTRIAN_DYNAMIC_UTOWN_STATE*>(state);
		new_state->PedPos.X=real_state->PedPos.X;
		new_state->PedPos.Y=real_state->PedPos.Y;
		new_state->RobPos.Y=real_state->RobPos.Y;
		new_state->Vel=real_state->Vel;
	//	new_state->Goal=real_state->Goal;     //uncomment this line for fully observability
        Root->Beliefs().AddSample(new_state);

	}

//	pedproblem_c->vnode_list.push_back(Root);
//	pedproblem_c->history_list.push_back(History);
	if(ModelParams::debug)
	{
		cout<<"Initial Beliefs"<<endl;
		Simulator.DisplayBeliefs(Root->Beliefs(), cout);
	}
	o=0;
	l=0;
	r=0;
	sum=0;
}


MCTS::~MCTS()
{
    VNODE::Free(Root, Simulator);
    //VNODE::FreeAll();
	//cout<<"Freeall"<<endl;
}

int ro_count=0;
bool MCTS::Update(int action, OBS_TYPE observation, double reward, STATE*obs_state)
{		

	//pedproblem_c->map_time=0;
	if(Params.Verbose>=1)
	{
		cout<<"reuse count "<<reuse_count<<endl;
		//cout<<"ro count"<<ro_count<<endl;
		ro_count=0;
		cout<<"merge count "<<merge_count<<endl;
		//cout<<"root count "<<root_count<<endl;
		cout<<"history list size "<<pedproblem_c->history_list.size()<<endl;
		cout<<"vnode list size "<<pedproblem_c->vnode_list.size()<<endl;
		cout<<"Root Value:"<<Root->Value.GetValue()<<endl;
	}
	//cout<<"vnode old size "<<pedproblem_c->vnode_old.size()<<endl;
	reuse_count=0;
	merge_count=0;
	root_count=0;
	o=l=r=sum=0;
    History.Add(action, observation);
    BELIEF_STATE beliefs;

    // Find matching vnode from the rest of the tree
    QNODE& qnode = Root->Child(action);
    VNODE* vnode = qnode.Child(observation);

	for(int i=0;i<10000;i++)
	{
		
	}
	//if(false)
	if(Params.UseQmdp==true)
	{
		cout<<"UseQMDP"<<endl;
		Simulator.UseQmdp=false;
		//for(int i=0;i<Params.NumStartStates;i++)
		for(int i=0;i<10000;i++)
		{
			STATE* state = Root->Beliefs().CreateSample(Simulator);
			OBS_TYPE obs_temp;
			double rwd_temp;
			Simulator.Step(*state,action,obs_temp,rwd_temp);
			if(obs_temp==observation)
			{
				beliefs.AddSample(state);
			}
			//cout<<obs_temp<<endl;
		}
		Simulator.UseQmdp=true;
	}
	else if (vnode)
    {
        beliefs.Copy(vnode->Beliefs(), Simulator);

    }
	
	int n=beliefs.GetNumSamples();
	int trials=0;
	/*
	while(n<Params.NumStartStates&&trials<10000)
	{
		OBS_TYPE obs;
		double rwd_temp;
		STATE* state = Root->Beliefs().CreateSample(Simulator);
		Simulator.Step(*state,action,obs,rwd_temp);
		if(obs==observation)   //found new consistent particle
		{
			beliefs.AddSample(state);	
			n++;
		}
		trials++;
	}*/


   	if(beliefs.GetNumSamples()==0)
    {
      //  if (Params.Verbose >= 1)
            cout << "No matching node found,need to resample" << endl;
			cerr << "No matching node found,need to resample" << endl;
			/*
			for (int i = 0; i < Params.NumStartStates; i++)
				beliefs.AddSample(Simulator.CreateStartState());
			cout<<"resampling finished"<<endl;
			*/
			//just keep the previous belief
			//beliefs=Root->Beliefs();
			beliefs.Copy(Root->Beliefs(),Simulator);
			if(ModelParams::debug) 			Simulator.DisplayBeliefs(beliefs,cout);
    }

	//if(vnode)	Simulator.ModifyObsStates(beliefs,obs_state);
	Simulator.ModifyObsStates(beliefs,obs_state);
	if (Params.Verbose >= 1)
		cout << "Matched " << beliefs.GetNumSamples() << " states" << endl;

    // Generate transformed states to avoid particle deprivation
    if (Params.UseTransforms)
        AddTransforms(Root, beliefs);

    // If we still have no particles, fail
    if (beliefs.Empty() && (!vnode || vnode->Beliefs().Empty()))
        return false;

	//cout<<"start display"<<endl;
    if (Params.Verbose >= 1)
        Simulator.DisplayBeliefs(beliefs, cout);

	//cout<<"finish display"<<endl;


    // Find a state to initialise prior (only requires fully observed state)
    const STATE* state = 0;
	//cout<<"start getting sample"<<endl;
	
	if(beliefs.GetNumSamples()==0)
	{
		cout<<"zero belief error!"<<endl;
	}

	/*
    if (vnode && !vnode->Beliefs().Empty())
        state = vnode->Beliefs().GetSample(0);
    else
        state = beliefs.GetSample(0);*/

	state=beliefs.GetSample(0);

	//cout<<"Prior State is!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
	//Simulator.DisplayState(*state,cout);
	//cout<<"end getting sample"<<endl;
	//cout<<"num of particles "<<beliefs.GetNumSamples()<<endl;
    // Delete old tree and create new root

	
	//DisplayValue(20,cout);


	//cout<<"start free"<<endl;
	//root_list.push_back(Root);	
	cout<<"before free"<<endl;
    VNODE::Free(Root, Simulator);
	cout<<"after free"<<endl;
	//VNODE::PartFree(Root,action,observation,Simulator);
	//cout<<"end free"<<endl;

	//cout<<"start clearing"<<endl;

	/*	
	merge_out<<"start printing history list!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
	for(int i=0;i<pedproblem_c->history_list.size();i++)
	{
		HISTORY h=pedproblem_c->history_list[i];
		for(int i=0;i<h.Size();i++)
			merge_out<<"a:"<<h[i]._Action<<" o:"<<h[i]._Observation<<" ";	
		merge_out<<endl;
		VNODE*vn=pedproblem_c->vnode_list[i];
		pedproblem_c->DisplayBeliefs(vn->Beliefs(),merge_out);
	}*/
	//std::copy(pedproblem_c->vnode_list.begin(),pedproblem_c->vnode_list.end(),pedproblem_c->vnode_old.begin());
//	pedproblem_c->vnode_old.clear();
//	pedproblem_c->vnode_old.insert(pedproblem_c->vnode_old.begin(),pedproblem_c->vnode_list.begin(),pedproblem_c->vnode_list.end());
	pedproblem_c->vnode_list.clear();
	pedproblem_c->history_list.clear();
 
	//cout<<"end clearing"<<endl;
	//cout<<"!!!!!!!!!!!!!!!vnodepool size!!!!!!!!!!!!!!! "<<Root->VNodePool.GetFreeListSize()<<endl;
	//cout<<"start expanding"<<endl;
	
    VNODE* newRoot = ExpandNode(state);
//	pedproblem_c->UpdateModel(0);


	//VNODE*newRoot=vnode;
	//VNODE::Reset(vnode);
	//cout<<"end expanding"<<endl;
    newRoot->Beliefs() = beliefs;
    Root = newRoot;
	cout<<"here"<<endl;
	//Rollout_Root=vnode;
	


	/*
	pedproblem_c->DisplayBeliefs(beliefs,cout);
	
	pedproblem_c->UpdateModel(0);
	state=Root->Beliefs().GetSample(0);
    const PEDESTRIAN_DYNAMIC_UTOWN_STATE* PEDESTRIAN_DYNAMIC_UTOWN_STATE = safe_cast<const PEDESTRIAN_DYNAMIC_UTOWN_STATE*>(state);

	int px=PEDESTRIAN_DYNAMIC_UTOWN_STATE->PedPos.X;
	int py=PEDESTRIAN_DYNAMIC_UTOWN_STATE->PedPos.Y;
	cout<<"px py "<<px<<" "<<py<<endl;
	pedproblem_c->map[px][py]=0;
	cout<<endl;
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<10;j++)
			cout<<pedproblem_c->map[i][j];
		cout<<endl;
	}*/



    return true;
}

int MCTS::SelectAction()
{
    if (Params.DisableTree)
    {
        RolloutSearch();
    }
    else
    {
        UCTSearch();
    }
    return GreedyUCB(Root, false);
}

void MCTS::RolloutSearch()
{
	std::vector<double> totals(Simulator.GetNumActions(), 0.0);
	int historyDepth = History.Size();
	std::vector<int> legal;
	assert(BeliefState().GetNumSamples() > 0);
	if(BeliefState().GetNumSamples()==0)
	{
		cout<<"zero belief error!"<<endl;
	}
	Simulator.GenerateLegal(*BeliefState().GetSample(0), GetHistory(), legal, GetStatus());
	random_shuffle(legal.begin(), legal.end());

	for (int i = 0; i < Params.NumSimulations; i++)
	{
		int action = legal[i % legal.size()];
		STATE* state = Root->Beliefs().CreateSample(Simulator);
		Simulator.Validate(*state);

		OBS_TYPE observation;
		double immediateReward, delayedReward, totalReward;
		bool terminal = Simulator.Step(*state, action, observation, immediateReward);

		VNODE*& vnode = Root->Child(action).Child(observation);
		if (!vnode && !terminal)
		{
			vnode = ExpandNode(state);
			AddSample(vnode, *state);
		}
		History.Add(action, observation);

		delayedReward = Rollout(*state);		
		totalReward = immediateReward + Simulator.GetDiscount() * delayedReward;
		Root->Child(action).Value.Add(totalReward);

		Simulator.FreeState(state);
		History.Truncate(historyDepth);


	}
}

int curr_sim;
long long merge_time;
long long rollout_time;
long long map_time;

int historyDepth;
void MCTS::UCTSearch()
{
	if(Params.Verbose>=1) 	cout<<"rollout_time"<<rollout_time<<endl;
	rollout_time=0;
	merge_time=0;
	map_time=0;
    ClearStatistics();
    historyDepth = History.Size();
	boost::timer timer;
	
	pedproblem_c->vnode_list.push_back(Root);
	pedproblem_c->history_list.push_back(History);
	Root->inserted=true;

	int n;
    for (n = 0; n < Params.NumSimulations; n++)
    {

		//cout<<Root->Value.GetCount()<<endl;
		cout<<n<<endl;
		curr_sim=n;
        STATE* state = Root->Beliefs().CreateSample(Simulator);
        Simulator.Validate(*state);
        Status.Phase = SIMULATOR::STATUS::TREE;
        if (Params.Verbose >= 2)
        {
            cout << "Starting simulation" << endl;
            Simulator.DisplayState(*state, cout);
        }

        TreeDepth = 0;
        PeakTreeDepth = 0;

        double totalReward = SimulateV(*state, Root);
		//cout<<"simV time "<<t<<endl;
        StatTotalReward.Add(totalReward);
        StatTreeDepth.Add(PeakTreeDepth);

        if (Params.Verbose >= 2)
            cout << "Total reward = " << totalReward << endl;
        if (Params.Verbose >= 3)
            DisplayValue(4, cout);
		//DisplayValue(20,cout);	
		//depth_out<<MaxTreeDepth<<endl;
		/*
		listen_out<<Root->Child(0).Value.GetCount()<<endl;
		openl_out<<Root->Child(1).Value.GetCount()<<endl;
		openr_out<<Root->Child(2).Value.GetCount()<<endl;*/

        Simulator.FreeState(state);
        History.Truncate(historyDepth);

		if((!Params.UseTime)||Params.UseQmdp)
		{
			if(n>30000) break;
		}
		else if(timer.elapsed()>1.0) 
		{
			break;
		}
		//if(n>10000) enable_reuse=true;
		//cout<<timer.elapsed()<<endl;
    }
	if(Params.Verbose>=1) 	cout<<"Number of Simulations : "<<n<<endl;
	/*
	printf("merge time %lld\n",merge_time);
	printf("rollout time %lld\n",rollout_time);
	printf("map time %lld\n",map_time);*/
	//ofstream out("count.txt");
	//
	
	//count_out<<Root->Child(0).Value.GetCount()<<" ";
	//count_out<<Root->Child(1).Value.GetCount()<<" ";
	//count_out<<Root->Child(2).Value.GetCount()<<endl;
	//cout<<"map time "<<pedproblem_c->map_time<<endl;
	//cout<<"map count"<<pedproblem_c->map_counter<<endl;
	if(Params.Verbose>=1) DisplayStatistics(cout);
	int begin=historyDepth;
	/*
	for(int i=0;i<pedproblem_c->history_list.size();i++)
	{
		HISTORY h=pedproblem_c->history_list[i];
		int end=h.Size();
		for(int i=begin;i<end;i++)
		{
			merge_out<<"a:"<<h[i]._Action<<" o:"<<h[i]._Observation<<" ";	
		}
		merge_out<<endl;
		pedproblem_c->DisplayBeliefs(pedproblem_c->vnode_list[i]->Beliefs(),merge_out);
	}*/

}

double MCTS::SimulateV(STATE& state, VNODE* vnode)
{
	vnode->visited++;
	vnode->visit_count++;
	//cout<<vnode->visit_count<<endl;
//	cout<<vnode->visited<<endl;
//	if(vnode->visited>2) return vnode->Value.GetValue();
	//cout<<TreeDepth<<endl;
	//cout<<"start simulateV"<<endl;
    int action = GreedyUCB(vnode, true);
	if(action==0) 	o++;
	else if(action==1) 	l++;
	else r++;
	sum++;
	/*
	listen_out<<o<<endl;
	openl_out<<l<<endl;
	openr_out<<r<<endl;*/

    PeakTreeDepth = TreeDepth;
    if (TreeDepth >= Params.MaxDepth) // search horizon reached
	{
		vnode->visited--;
        return 0;
	}

    //if (TreeDepth == 1)   AddSample(vnode, state);

	/*	
	if(vnode!=Root&&History.Back()._Action!=0)
	{
		return Root->Value.GetValue();	
	}*/
    QNODE& qnode = (vnode)->Child(action);
    double totalReward = SimulateQ(state, qnode, action);
    (vnode)->Value.Add(totalReward);
	/*
	if(vnode->merged)
	{
		vnode->merge_node->Value.Add(totalReward);
	}*/

	//cout<<"finish simulateV"<<endl;

    //AddRave(vnode, totalReward);
	vnode->visited--;
    return totalReward;
}


/*
QNODE* MCTS::QMerge(QNODE *n1, QNODE*n2)
{
	QNODE*nq=new QNODE;
	nq->Initialise();
	if(n1->Value.GetCount()==+LargeInteger||n1->Value.GetCount()==0) 
	{
		nq->Copy(n2,Simulator);	
		//delete n1;
		return nq;   	//both should be illegal node
	}https://www.facebook.com/
	if(n2->Value.GetCount()==+LargeInteger||n2->Value.GetCount()==0) 
	{
		//delete n2;
		nq->Copy(n1,Simulator);
		return nq;   	//both should be illegal node
	}
	double v1=n1->Value.GetValue();
	double v2=n2->Value.GetValue();
	int c1=n1->Value.GetCount();
	int c2=n2->Value.GetCount();
	//n2->Value.Set(c1+c2,(v1*c1+v2*c2)/(c1+c2));
	//if(n2==0)   cout<<"warning!!!!!!!!!!!!!!!!!!"<<endl;
	nq->Value.Set(c1+c2,(v1*c1+v2*c2)/(c1+c2));
	for(int i=0;i<n2->Children.size();i++)
	{
	//	nq->Children[i]=VNODE::Create();
		nq->Children[i]=n2->Children[i];
		(*nq->Children[i])=VMerge(*n1->Children[i],*n2->Children[i]);
	}

	//delete n1;
	return nq;	
}

VNODE* MCTS::VMerge(VNODE *n1, VNODE*n2)
{
	VNODE*nn=VNODE::Create();		
	if(n1==0)
	{
			if(n2==0) 	return 0;
			return n2;
			nn->Copy(n2,Simulator);
			//nn.Beliefs().Copy(n2.Beliefs(),Simulator);
			//VNODE::Free(n1, Simulator);
			return nn;
	}
	if(n2==0) 
	{
			if(n1==0)  	return 0;
			return n1;
			nn->Copy(n1,Simulator);
			//nn.Beliefs().Copy(n2.Beliefs(),Simulator);
			//VNODE::Free(n2);
			return nn;
	}
	double v1=n1->Value.GetValue();
	double v2=n2->Value.GetValue();
	int c1=n1->Value.GetCount();
	int c2=n2->Value.GetCount();
	nn->Value.Set(c1+c2,(v1*c1+v2*c2)/(c1+c2));
	nn->Beliefs().Move(n1->Beliefs());
	nn->Beliefs().Move(n2->Beliefs());
	for(int i=0;i<n2->Children.size();i++)
	{
		QNODE* q=QMerge(&n1->Children[i],&n2->Children[i]);
		nn->Children[i]=*q;
	}
	//VNODE::Free(n1);
	//VNODE::Free(n2);
	//n2->Value.GetValue();	
	//delete n1;
	return nn;
}*/

QNODE* MCTS::QMerge(QNODE *n1, QNODE*n2)
{
	//QNODE*nq=new QNODE;
	//nq->Initialise();
	if(n1->Value.GetCount()==+LargeInteger||n1->Value.GetCount()==0) 
	{
		//nq->Copy(n2,Simulator);	
		//delete n1;
		return n2;   	//both should be illegal node
	}
	if(n2->Value.GetCount()==+LargeInteger||n2->Value.GetCount()==0) 
	{
		//delete n2;
		//nq->Copy(n1,Simulator);
		return n1;   	//both should be illegal node
	}
	double v1=n1->Value.GetValue();
	double v2=n2->Value.GetValue();
	int c1=n1->Value.GetCount();
	int c2=n2->Value.GetCount();
	//n2->Value.Set(c1+c2,(v1*c1+v2*c2)/(c1+c2));
	//if(n2==0)   cout<<"warning!!!!!!!!!!!!!!!!!!"<<endl;
	n2->Value.Set(c1+c2,(v1*c1+v2*c2)/(c1+c2));
	map<OBS_TYPE,VNODE*>::iterator it=n2->Children.begin();
	/*
	for(int i=0;i<n2->Children.size();i++)
	{
	//	nq->Children[i]=VNODE::Create();
		//n2->Children[i]=n2->Children[i];
		n2->Children[i]=VMerge(n1->Children[i],n2->Children[i]);
	}*/
	for(;it!=n2->Children.end();it++)
	{
		if(n1->Children.find(it->first)==n1->Children.end()) continue;
		it->second=VMerge(n1->Children[it->first],it->second);
	}

	//delete n1;
	return n2;	
}

VNODE* MCTS::VMerge(VNODE *n1, VNODE*n2)
{
	//VNODE*nn=VNODE::Create();		
	if(n1==0)
	{
			//return 0;
			return n2;
			if(n2==0) 	return 0;
			return n2;
			//nn->Copy(n2,Simulator);
			//nn.Beliefs().Copy(n2.Beliefs(),Simulator);
			//VNODE::Free(n1, Simulator);
			//return nn;
	}
	if(n2==0) 
	{
			//reposition the pointer,add the counter
			/*
			n1->counter++;
			cout<<n1<<endl;
			return n1;*/
			return 0;
			if(n1==0)  	return 0;
			return n1;
			//nn->Copy(n1,Simulator);
			//nn.Beliefs().Copy(n2.Beliefs(),Simulator);
			//VNODE::Free(n2);
			//return nn;
	}
	double v1=n1->Value.GetValue();
	double v2=n2->Value.GetValue();
	int c1=n1->Value.GetCount();
	int c2=n2->Value.GetCount();

	const ROCKSAMPLE_STATE *rockstate1;
	const ROCKSAMPLE_STATE*rockstate2;
	if(n1->Beliefs().GetNumSamples()==0||n2->Beliefs().GetNumSamples()==0) 
	{
		cout<<"Zero Belief Error!"<<endl;
	}

	/*
    rockstate1=safe_cast<const ROCKSAMPLE_STATE*>(n1->Beliefs().GetSample(0));
	rockstate2=safe_cast<const ROCKSAMPLE_STATE*>(n2->Beliefs().GetSample(0));
	if((rockstate1->AgentPos.X!=rockstate2->AgentPos.X)||(rockstate1->AgentPos.Y!=rockstate2->AgentPos.Y))
	{
		cout<<"Agent Pos doesn't Match!"<<endl;
	}*/


	n2->Value.Set(c1+c2,(v1*c1+v2*c2)/(c1+c2));
	//n2->Beliefs().Copy(n1->Beliefs(),Simulator);
	//nn->Value.Set(c1+c2,(v1*c1+v2*c2)/(c1+c2));
	//nn->Beliefs().Move(n1->Beliefs());
	//nn->Beliefs().Move(n2->Beliefs());
	for(int i=0;i<n2->Children.size();i++)
	{	
			double v1=n1->Children[i].Value.GetValue();
			double v2=n2->Children[i].Value.GetValue();
			int c1=n1->Children[i].Value.GetCount();
			int c2=n2->Children[i].Value.GetCount();
			if(c1==+LargeInteger||c1==0)
			{
					n2->Children[i].Value.Set(c2,v2);
			}
			else if(c2==+LargeInteger||c2==0)
			{
					n2->Children[i].Value.Set(c1,v1);
			}
			else
			{
					//n2->Value.Set(c1+c2,(v1*c1+v2*c2)/(c1+c2));
					//if(n2==0)   cout<<"warning!!!!!!!!!!!!!!!!!!"<<endl;
					n2->Children[i].Value.Set(c1+c2,(v1*c1+v2*c2)/(c1+c2));
			}
			map<OBS_TYPE,VNODE*>::iterator it=n2->Children[i].Children.begin();
			for(;it!=n2->Children[i].Children.end();it++)
			{
					if(n1->Children[i].Children.find(it->first)==n1->Children[i].Children.end()) continue;
					VNODE*tmp=VMerge(n1->Children[i].Children[it->first],it->second);	
					it->second=tmp;
					//n2->Children[i].Children[j]=VMerge(n1->Children[i].Children[j],n2->Children[i].Children[j]);
			}
				//n2->Children[i]=(*QMerge(&n1->Children[i],&n2->Children[i]));
		//nn->Children[i]=*q;
	}
	//VNODE::Free(n1);
	//VNODE::Free(n2);
	//n2->Value.GetValue();	
	//delete n1;
	return n2;
}

void MCTS::Merge(VNODE *n1, VNODE *n2)
{
	//VNODE*tmp=n1;
	//
	//
	//cout<<"Merge start"<<endl;
	
	//VNODE*tmp=(*n2);
	//
	/*
	if(n1->Beliefs().GetNumSamples()>n2->Beliefs().GetNumSamples())
	{
		return;
		//return ;
		VNODE*tmp=n1;
		n1=n2;
		n2=tmp;
	}*/

	/*
	const ROCKSAMPLE_STATE *rockstate1;
	const ROCKSAMPLE_STATE*rockstate2;
	
    rockstate1=safe_cast<const ROCKSAMPLE_STATE*>(n1->Beliefs().GetSample(0));
	rockstate2=safe_cast<const ROCKSAMPLE_STATE*>(n2->Beliefs().GetSample(0));
	if((rockstate1->AgentPos.X!=rockstate2->AgentPos.X)||(rockstate1->AgentPos.Y!=rockstate2->AgentPos.Y))
	{
		cout<<"Agent Pos doesn't Match!"<<endl;
	}*/

	/*	
	double v1=n1->Value.GetValue();
	double v2=n2->Value.GetValue();
	int c1=n1->Value.GetCount();
	int c2=n2->Value.GetCount();
	n2->Value.Set(c1+c2,(v1*c1+v2*c2)/(c1+c2));*/
	VMerge(n1,n2);
    //ROCrockstate=safe_cast<const ROCKSAMPLE_STATE&>(*p_rockstate);
	//Simulator.DisplayBeliefs(n1->Beliefs(),cout);
	//Simulator.DisplayBeliefs(n2->Beliefs(),cout);
	//VNODE::Free(tmp,Simulator);
	/*
	n1->merged=true;
	n1->merge_index=pedproblem_c->FindIndex(n2);
	*/
	//cout<<"Merge Finished"<<endl;
	//return nn;
	//delete tmp;
}

double MCTS::SimulateQ(STATE& state, QNODE& qnode, int action)
{

	OBS_TYPE observation;
	double immediateReward, delayedReward = 0;


	if (Simulator.HasAlpha())
		Simulator.UpdateAlpha(qnode, state);
	bool terminal = Simulator.Step(state, action, observation, immediateReward);
	assert(observation >= 0 && observation < Simulator.GetNumObservations());
	History.Add(action, observation);

	if (Params.Verbose >= 3)
	{
		Simulator.DisplayAction(action, cout);
		Simulator.DisplayObservation(state, observation, cout);
		Simulator.DisplayReward(immediateReward, cout);
		Simulator.DisplayState(state, cout);
	}
	/*
	   VNODE*vnode; 
	   if(action!=0)
	   {
	//vnode = Root;
	return Root->Value.GetValue();
	}
	else
	{
	vnode = qnode.Child(observation);
	if (!vnode && !terminal && qnode.Value.GetCount() >= Params.ExpandCount)
	vnode=qnode.Child(observation) = ExpandNode(&state);
	}*/

	VNODE* &vnode = qnode.Child(observation);
	//VNODE** vnode_pt;

	//vnode = qnode.Child(observation);
	if (!vnode && !terminal && qnode.Value.GetCount() >= Params.ExpandCount)
		vnode = ExpandNode(&state);



	// this is for the convinience of the belief reuse 
	if(vnode)
	{
		AddSample(vnode, state);
	}

	if (!terminal)
	{
		TreeDepth++;
		/*
		   if(TreeDepth>MaxTreeDepth)
		   {
		   MaxTreeDepth=TreeDepth;
		   }*/


		VNODE * vnode_new;

		//VNODE* vnode_new=pedproblem_c->MapBelief(vnode,Root,History);
		//cout<<"belief mapped"<<endl;
	//	bool qmdp = true;

		if(!vnode)
		{
			if(Params.LeaveValue==-1)  //reserve -1 for random rollout
				delayedReward=Rollout(state);
			else
				delayedReward=Params.LeaveValue;
			//delayedReward=-10000;
			//delayedReward=0;
			//delayedReward=User_Rollout(state);
			//delayedReward=pedproblem_c->QMDP(&state);

		}
		else 
		{
			//vnode_new=pedproblem_c->MapBelief(vnode,Root,History);
			//cout<<"start map belief"<<endl;
			//boost::timer;		

			if(Params.Reuse&&(vnode->merged==true))
			{
				reuse_count++;
				//SimulateV(state,vnode);
				vnode_new=vnode->merge_node;
		
				//SimulateV(state,vnode);
				
				delayedReward=(vnode_new)->Value.GetValue();
				

			}
			//vnode_new=pedproblem_c->MapBelief(vnode,Root,History);
			else if(Params.Reuse&&(vnode_new=pedproblem_c->MapBelief(vnode,Root,History,historyDepth))&&vnode!=vnode_new)
			{
				merge_count++;	
				//SimulateV(state,vnode);
				/*
				   double v1=vnode->Value.GetValue();
				   double v2=vnode_new->Value.GetValue();
				   int c1=vnode->Value.GetCount();
				   int c2=vnode_new->Value.GetCount();*/

				//n2->Value.Set(c1+c2,(v1*c1+v2*c2)/(c1+c2));
				//delayedReward=(v1*c1+v2*c2)/(c1+c2);

				//cout<<"merge star"<<endl;
				//SimulateV(state,vnode);
				//cout<<"start merge"<<endl;
				Merge(vnode,vnode_new);
				//cout<<"end merge"<<endl;
				vnode->merged=true;
				vnode->merge_node=vnode_new;	
					
		
				int begin=0;
				int end=History.Size();
				HISTORY h=pedproblem_c->history_list[pedproblem_c->FindIndex(vnode_new)];
				delayedReward=(vnode_new)->Value.GetValue();	


			}
			else
				//vnode_new=pedproblem_c->FindByIndex(vnode->merge_index);
			{		
				delayedReward = SimulateV(state, vnode);
				/*
				   if(vnode==vnode_new)
				   {
				   cout<<"vnode and vnode_new equal!!!!!!!!!!!!!!!!!!"<<endl;
				   }
				   int begin=historyDepth;
				   int end=History.Size();
				   merge_out<<"vnode history"<<endl;
				   for(int i=begin;i<end;i++)
				   {
				   merge_out<<"a:"<<History[i]._Action<<" o:"<<History[i]._Observation<<" ";	
				   }
				   merge_out<<endl;
				   HISTORY h=pedproblem_c->history_list[pedproblem_c->FindIndex(vnode_new)];
				   end=h.Size();
				   for(int i=begin;i<end;i++)
				   {
				   merge_out<<"a:"<<h[i]._Action<<" o:"<<h[i]._Observation<<" ";	
				   }
				   merge_out<<endl;
				   merge_out<<"vnode beliefs :"<<endl;
				   pedproblem_c->DisplayBeliefs(vnode->Beliefs(),merge_out);
				   merge_out<<"vnode_new beliefs :"<<endl;
				   pedproblem_c->DisplayBeliefs(vnode_new->Beliefs(),merge_out);

				   reuse_count++;
				   if(vnode_new==Root)
				   {
				   root_count++;
				   }
				   delayedReward=(vnode_new)->Value.GetValue();*/

			}
			//DisplayValue(20,cout);
		}
		TreeDepth--;
	}
	/*
	   if(!terminal)
	   {
	   TreeDepth++;
	   if(vnode)
	   delayedReward=SimulateV(state,vnode);
	   else
	   delayedReward=Rollout(state);
	   }*/

	double totalReward = immediateReward + Simulator.GetDiscount() * delayedReward;

	//	(*parent)->Child(action).Value.Add(totalReward);
	qnode.Value.Add(totalReward);
	//cout<<"finish simulate Q"<<endl;

	return totalReward;
}

void MCTS::AddRave(VNODE* vnode, double totalReward)
{
	double totalDiscount = 1.0;
	for (int t = TreeDepth; t < History.Size(); ++t)
	{
		QNODE& qnode = vnode->Child(History[t]._Action);
		qnode.AMAF.Add(totalReward, totalDiscount);
		totalDiscount *= Params.RaveDiscount;
	}
}

VNODE* MCTS::ExpandNode(const STATE* state)
{
	VNODE* vnode = VNODE::Create();
	vnode->Value.Set(0, 0);
	Simulator.Prior(state, History, vnode, Status);

	if (Params.Verbose >= 2)
	{
		cout << "Expanding node: ";
		History.Display(cout);
		cout << endl;
	}

	return vnode;
}

void MCTS::AddSample(VNODE* node, const STATE& state)
{
	STATE* sample = Simulator.Copy(state);
	node->Beliefs().AddSample(sample);
	if (Params.Verbose >= 2)
	{
		cout << "Adding sample:" << endl;
		Simulator.DisplayState(*sample, cout);
	}
}

int MCTS::GreedyUCB(VNODE* vnode, bool ucb) 
{
	static vector<int> besta;
	besta.clear();
	double bestq = -Infinity;
	int N = vnode->Value.GetCount();
	double logN = log(N + 1);
	bool hasalpha = Simulator.HasAlpha();

	for (int action = 0; action < Simulator.GetNumActions(); action++)
	{
		double q, alphaq;
		int n, alphan;

		QNODE& qnode = vnode->Child(action);
		q = qnode.Value.GetValue();
		n = qnode.Value.GetCount();

		if (Params.UseRave && qnode.AMAF.GetCount() > 0)
		{
			double n2 = qnode.AMAF.GetCount();
			double beta = n2 / (n + n2 + Params.RaveConstant * n * n2);
			q = (1.0 - beta) * q + beta * qnode.AMAF.GetValue();
		}

		if (hasalpha && n > 0)
		{
			Simulator.AlphaValue(qnode, alphaq, alphan);
			q = (n * q + alphan * alphaq) / (n + alphan);
			//cout << "N = " << n << ", alphaN = " << alphan << endl;
			//cout << "Q = " << q << ", alphaQ = " << alphaq << endl;
		}

		if (ucb)
		{
			//	ucbvalues_out<<FastUCB(N, n, logN)<<endl;
			q += FastUCB(N, n, logN);
		}

		if (q >= bestq)
		{
			if (q > bestq)
				besta.clear();
			bestq = q;
			besta.push_back(action);
		}
	}

	assert(!besta.empty());
	return besta[Random(besta.size())];
}

double MCTS::Rollout(STATE& state)
{
	Status.Phase = SIMULATOR::STATUS::ROLLOUT;
//	cout<<"starting rollout"<<endl;
	if (Params.Verbose >= 3)
		cout << "Starting rollout" << endl;

	double totalReward = 0.0;
	double discount = 1.0;
	bool terminal = false;
	int numSteps;

	/**************/


	/*************/
	for (numSteps = 0; numSteps + TreeDepth < Params.MaxDepth && !terminal; ++numSteps)
	{
		OBS_TYPE observation;
		double reward;

		int action = Simulator.SelectRandom(state, History, Status);
		//cout<<action<<endl;
		/*	
			if(action!=0) 
			{
			if(curr_sim<10000)
			totalReward+=200*discount;
			else 	totalReward+=Root->Value.GetValue()*discount;
			break;
			}*/
		terminal = Simulator.Step(state, action, observation, reward);

		History.Add(action, observation);

		if (Params.Verbose >= 4)
		{
			Simulator.DisplayAction(action, cout);
			Simulator.DisplayObservation(state, observation, cout);
			Simulator.DisplayReward(reward, cout);
			Simulator.DisplayState(state, cout);
		}

		totalReward += reward * discount;
		discount *= Simulator.GetDiscount();

	}

	StatRolloutDepth.Add(numSteps);
	if (Params.Verbose >= 3)
		cout << "Ending rollout after " << numSteps
			<< " steps, with total reward " << totalReward << endl;
	return totalReward;
}

double MCTS::User_Rollout(STATE& state)
{
	if(!Rollout_Root)
	{
		//ROS_INFO("default rollout");
		return Rollout(state);
	}
	
	int i;
	QNODE qnode;
	VNODE* vnode=Rollout_Root;
	int action;
	OBS_TYPE observation;
	double reward;
	//ROS_INFO("pre-computing!!!!!!!!!!!!!!!!!!");
	//ROS_INFO("History size %d",History.Size());
	
	for(i=historyDepth;i<History.Size();i++)
	{
		action=History[i]._Action;
		observation=History[i]._Observation;
		qnode = vnode->Child(action);
		vnode = qnode.Child(observation);
		if(!vnode)
		{
			//ROS_INFO("history is %s",out.c_str());
			return Rollout(state);
		}
		//cout<<action<<" "<<observation;

	}
	//ROS_INFO("history is %s",out.c_str());
	//ROS_INFO("prepare to roll out!!!!!!!!!!!!!!!!1");
	//we then implement the rollout policy from the current node
	int numSteps;
	double totalReward = 0.0;
	double discount = 1.0;
	bool terminal = false;
	//return vnode->Value.;
	for (numSteps = 0; numSteps + TreeDepth < Params.MaxDepth && !terminal; ++numSteps)
	{
		//ROS_INFO("user rollout");
		ro_count++;
		//action = GreedyAction(vnode);
		action=GreedyUCB(vnode,false);
		//action=Simulator.SelectRandom(state, History, Status);
		//action=1;
		//we also need to deal with some impossible states here

		terminal = Simulator.Step(state, action, observation, reward);
		totalReward += reward * discount;
		discount *= Simulator.GetDiscount();
		if(!terminal)
		{
			qnode = vnode->Child(action);
			vnode = qnode.Child(observation);
			if(!vnode)	//reach the end
			{
				totalReward+=Rollout(state);
				//totalReward+=0;
				break;
			}
		}
		else
		{
			break;
		}
	}

	return totalReward;
}
void MCTS::AddTransforms(VNODE* root, BELIEF_STATE& beliefs)
{
	int attempts = 0, added = 0;

	// Local transformations of state that are consistent with history
	while (added < Params.NumTransforms && attempts < Params.MaxAttempts)
	{
		STATE* transform = CreateTransform();
		if (transform)
		{
			beliefs.AddSample(transform);
			added++;
		}
		attempts++;
	}

	if (Params.Verbose >= 1)
	{
		cout << "Created " << added << " local transformations out of "
			<< attempts << " attempts" << endl;
	}
}

STATE* MCTS::CreateTransform() const
{
	OBS_TYPE stepObs;
	double stepReward;

	STATE* state = Root->Beliefs().CreateSample(Simulator);
	Simulator.Step(*state, History.Back()._Action, stepObs, stepReward);
	if (Simulator.LocalMove(*state, History, stepObs, Status))
		return state;
	Simulator.FreeState(state);
	return 0;
}

double MCTS::UCB[UCB_N][UCB_n];
bool MCTS::InitialisedFastUCB = true;

void MCTS::InitFastUCB(double exploration)
{
	cout << "Initialising fast UCB table... ";
	for (int N = 0; N < UCB_N; ++N)
		for (int n = 0; n < UCB_n; ++n)
			if (n == 0)
				UCB[N][n] = Infinity;
			else
				UCB[N][n] = exploration * sqrt(log(N + 1) / n);
	cout << "done" << endl;
	InitialisedFastUCB = true;
}

inline double MCTS::FastUCB(int N, int n, double logN) const
{
	if (InitialisedFastUCB && N < UCB_N && n < UCB_n)
		return UCB[N][n];

	if (n == 0)
		return Infinity;
	else
		return Params.ExplorationConstant * sqrt(logN / n);
}

void MCTS::ClearStatistics()
{
	StatTreeDepth.Clear();
	StatRolloutDepth.Clear();
	StatTotalReward.Clear();
}

void MCTS::DisplayStatistics(ostream& ostr) const
{
	if (Params.Verbose >= 1)
	{
		StatTreeDepth.Print("Tree depth", ostr);
		StatRolloutDepth.Print("Rollout depth", ostr);
		StatTotalReward.Print("Total reward", ostr);
		for(int i=0;i<Simulator.GetNumActions();i++)
		{
			cout<<"Action "<<i<<" Value: "<<Root->Children[i].Value.GetValue()<<endl;	
		}
	}

	if (Params.Verbose >= 2)
	{
		ostr << "Policy after " << Params.NumSimulations << " simulations" << endl;
		DisplayPolicy(6, ostr);
		ostr << "Values after " << Params.NumSimulations << " simulations" << endl;
		DisplayValue(6, ostr);
	}
}

void MCTS::DisplayValue(int depth, ostream& ostr) const
{
	HISTORY history;
	ostr << "MCTS Values:" << endl;
	Root->DisplayValue(history, depth, ostr);
}

void MCTS::DisplayPolicy(int depth, ostream& ostr) const
{
	HISTORY history;
	ostr << "MCTS Policy:" << endl;
	Root->DisplayPolicy(history, depth, ostr);
}

//-----------------------------------------------------------------------------

void MCTS::UnitTest()
{
	UnitTestGreedy();
	UnitTestUCB();
	UnitTestRollout();
	for (int depth = 1; depth <= 3; ++depth)
		UnitTestSearch(depth);
}

void MCTS::UnitTestGreedy()
{
	TEST_SIMULATOR testSimulator(5, 5, 0);
	PARAMS params;
	MCTS mcts(testSimulator, params);
	int numAct = testSimulator.GetNumActions();
	int numObs = testSimulator.GetNumObservations();

	VNODE* vnode = mcts.ExpandNode(testSimulator.CreateStartState());
	vnode->Value.Set(1, 0);
	vnode->Child(0).Value.Set(0, 1);
	for (int action = 1; action < numAct; action++)
		vnode->Child(action).Value.Set(0, 0);
	assert(mcts.GreedyUCB(vnode, false) == 0);
}

void MCTS::UnitTestUCB()
{
	TEST_SIMULATOR testSimulator(5, 5, 0);
	PARAMS params;
	MCTS mcts(testSimulator, params);
	int numAct = testSimulator.GetNumActions();
	int numObs = testSimulator.GetNumObservations();

	// With equal value, action with lowest count is selected
	VNODE* vnode1 = mcts.ExpandNode(testSimulator.CreateStartState());
	vnode1->Value.Set(1, 0);
	for (int action = 0; action < numAct; action++)
		if (action == 3)
			vnode1->Child(action).Value.Set(99, 0);
		else
			vnode1->Child(action).Value.Set(100 + action, 0);
	assert(mcts.GreedyUCB(vnode1, true) == 3);

	// With high counts, action with highest value is selected
	VNODE* vnode2 = mcts.ExpandNode(testSimulator.CreateStartState());
	vnode2->Value.Set(1, 0);
	for (int action = 0; action < numAct; action++)
		if (action == 3)
			vnode2->Child(action).Value.Set(99 + numObs, 1);
		else
			vnode2->Child(action).Value.Set(100 + numAct - action, 0);
	assert(mcts.GreedyUCB(vnode2, true) == 3);

	// _Action with low value and low count beats actions with high counts
	VNODE* vnode3 = mcts.ExpandNode(testSimulator.CreateStartState());
	vnode3->Value.Set(1, 0);
	for (int action = 0; action < numAct; action++)
		if (action == 3)
			vnode3->Child(action).Value.Set(1, 1);
		else
			vnode3->Child(action).Value.Set(100 + action, 1);
	assert(mcts.GreedyUCB(vnode3, true) == 3);

	// Actions with zero count is always selected
	VNODE* vnode4 = mcts.ExpandNode(testSimulator.CreateStartState());
	vnode4->Value.Set(1, 0);
	for (int action = 0; action < numAct; action++)
		if (action == 3)
			vnode4->Child(action).Value.Set(0, 0);
		else
			vnode4->Child(action).Value.Set(1, 1);
	assert(mcts.GreedyUCB(vnode4, true) == 3);
}

void MCTS::UnitTestRollout()
{
	TEST_SIMULATOR testSimulator(2, 2, 0);
	PARAMS params;
	params.NumSimulations = 1000;
	params.MaxDepth = 10;
	MCTS mcts(testSimulator, params);
	double totalReward;
	for (int n = 0; n < mcts.Params.NumSimulations; ++n)
	{
		STATE* state = testSimulator.CreateStartState();
		mcts.TreeDepth = 0;
		totalReward += mcts.Rollout(*state);
	}
	double rootValue = totalReward / mcts.Params.NumSimulations;
	double meanValue = testSimulator.MeanValue();
	assert(fabs(meanValue - rootValue) < 0.1);
}

void MCTS::UnitTestSearch(int depth)
{
	TEST_SIMULATOR testSimulator(3, 2, depth);
	PARAMS params;
	params.MaxDepth = depth + 1;
	params.NumSimulations = pow(double(10), double(depth + 1));
	MCTS mcts(testSimulator, params);
	mcts.UCTSearch();
	double rootValue = mcts.Root->Value.GetValue();
	double optimalValue = testSimulator.OptimalValue();
	assert(fabs(optimalValue - rootValue) < 0.1);
}

//-----------------------------------------------------------------------------
