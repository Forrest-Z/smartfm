#include "pedestrian_changelane.h"
#include "utils.h"
#include <string>
#include <math.h>
#include <fstream>
#include <iomanip>
#include "util_uniform.h"

using namespace std;
using namespace UTILS;


const int CRASH_PENALTY=-10000;
const int GOAL_REWARD=1000;

PEDESTRIAN_CHANGELANE::PEDESTRIAN_CHANGELANE(int x_size=4,int y_size=9):X_SIZE(x_size),Y_SIZE(y_size)		
{
	//NumObservations=5*Size+1;
	Discount=0.95;
	NumObservations=MaxObs();  //Y_SIZE+5 is max length of rob_map
	//need to use different const
	RewardRange=-CRASH_PENALTY;
	//RewardRange=100000;
	NumActions=3;
	//UpdateModel(0);
	//current_horizon=0;
	PedestrianState1=MemoryPool.Allocate();
	PedestrianState2=MemoryPool.Allocate();

	startState.RobPos.Y=0;
	startState.Vel=1.0;

	double prob;
	for(int i=0;i<ModelParams::N_PED_IN;i++)
	{
		bool ok=false;
		int N,NX,NY,NG;
		while(!ok)
		{
			prob=(rand()+0.0)/RAND_MAX;
			N=prob*X_SIZE*Y_SIZE;
			NX=N/Y_SIZE;
			NY=N-(NX*Y_SIZE);
			if(fabs(NX-2)+fabs(NY-0)>3) {ok=true;}
		}
		prob=rand()/RAND_MAX;
		NG=ModelParams::NGOAL*prob;
		//ped_state->PedPoses.push_back(make_pair(COORD(NX,NY),NG));
		//ped_state->vec.push_back(1);
		startState.PedPoses[i]=(PedStruct(COORD(NX,NY),NG,i));
	}
	debug=false;

	//initial the rob model
	double noisyMove[3][3] /*vel, move*/ = {{0, 1, 0},
		{0, 1, 2},
		//{1, 2, 3}};
		{0, 1, 2}};
	memcpy(robotNoisyMove, noisyMove, sizeof(noisyMove));
	if(ModelParams::goodrob==0)
	{
	   double moveProbs[3][3] = {{0.9, 0.1, 0.0},
	   {0.1, 0.8, 0.1},
	//	{0.1, 0.8, 0.1}};
		{0.1, 0.1, 0.8}};
		memcpy(robotMoveProbs, moveProbs, sizeof(moveProbs));
	}
	else
	{

		double moveProbs[3][3] = {{1.0, 0.0, 0.0},
			{0.1, 0.8, 0.1},
			//	{0.1, 0.8, 0.1}};
			{0.1, 0.1, 0.8}};
		memcpy(robotMoveProbs, moveProbs, sizeof(moveProbs));
	}


	double velUpdate[3][3][3] /*action,vel,updated*/ = {
		{{0, 1, 0}, {0, 1, 2}, {0, 1, 2}},
		{{0, 1, 2}, {0, 1, 2}, {0, 1, 2}},
		{{0, 1, 0}, {0, 1, 0}, {0, 1, 2}}};
	memcpy(robotVelUpdate, velUpdate, sizeof(velUpdate));

	if(ModelParams::goodrob==0)
	{
		double updateProb[3][3][3] /*action,vel,updated*/ = {
			{{0.9, 0.1, 0.0}, {0.2, 0.7, 0.1}, {0.1, 0.1, 0.8}},
			{{0.2, 0.7, 0.1}, {0.1, 0.1, 0.8}, {0.1, 0.1, 0.8}},
			{{0.9, 0.1, 0.0}, {0.9, 0.1, 0.0}, {0.2, 0.7, 0.1}}};

		memcpy(robotUpdateProb, updateProb, sizeof(updateProb));
	}
	else
	{
		double updateProb[3][3][3] /*action,vel,updated*/ = {
		{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
		{{0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}},
		{{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.5, 0.0}}};
		memcpy(robotUpdateProb, updateProb, sizeof(updateProb));
	}
}

void PEDESTRIAN_CHANGELANE::SetStartState(PedestrianState state)
{
	startState=state;
}
STATE*PEDESTRIAN_CHANGELANE::Copy(const STATE &state) const
{
	const PedestrianState& ped_state=safe_cast<const PedestrianState&>(state);
	PedestrianState*newstate=MemoryPool.Allocate();
	*newstate=ped_state;
	return newstate;
} 
void PEDESTRIAN_CHANGELANE::Validate(const STATE& state) const
{
}

void PEDESTRIAN_CHANGELANE::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const PedestrianState& ped_state = safe_cast<const PedestrianState&>(state);
	cout<<"rob map size "<<rob_map.size()<<endl;
    ostr<<"Rob Pos : "<<rob_map[ped_state.RobPos.Y].first<<" "<<rob_map[ped_state.RobPos.Y].second<<endl;
	for(int i=0;i<ped_state.num;i++)
	{
		cout << "Ped Pos: " << ped_state.PedPoses[i].first.X << " " <<ped_state.PedPoses[i].first.Y <<endl;
		cout << "Goal: " << ped_state.PedPoses[i].second << endl;
		cout << "ID :  " <<ped_state.PedPoses[i].third<<endl;
	}
	ostr<<"Rob Vel : "<<ped_state.Vel<<endl;
}



STATE*PEDESTRIAN_CHANGELANE::CreateStartState() const
{
	PedestrianState*ped_state=MemoryPool.Allocate();
	*ped_state=startState;

	double prob;
	for(int i=0;i<ped_state->num;i++)
	{
		//int NG=rand()%ModelParams::NGOAL;
		double prob=(rand()+0.0) / RAND_MAX;
		int x=ped_state->PedPoses[i].first.X;
		int y=ped_state->PedPoses[i].first.Y;

		int sum=0;
		int g;
		for(g=0;g<ModelParams::NGOAL;g++)
		{
			if(fabs(sfm->local_goals[g][0]-x)<ModelParams::GOAL_DIST&&fabs(sfm->local_goals[g][1]-y)<ModelParams::GOAL_DIST)
			{sum++;}
		}
		int next=prob*sum;
		sum=0;
		for(g=0;g<ModelParams::NGOAL;g++)
		{
			if(fabs(sfm->local_goals[g][0]-x)<ModelParams::GOAL_DIST&&fabs(sfm->local_goals[g][1]-y)<ModelParams::GOAL_DIST)
			{sum++;}
			if(sum>next) break;
		}
		ped_state->PedPoses[i].second=g;
		//ped_state->PedPoses[i].second=3;
	}

	return ped_state;
}



void PEDESTRIAN_CHANGELANE::FreeState(STATE* state) const
{
    PedestrianState* ped_state = safe_cast<PedestrianState*>(state);
    MemoryPool.Free(ped_state);
}

vector<vector<double> > PEDESTRIAN_CHANGELANE::GetBeliefVector(const BELIEF_STATE& beliefs) const
{
	int goal_count[10][10]={0};
	cout<<"Current Belief "<<endl;
	cout<<"particles num "<<beliefs.GetNumSamples()<<endl;
	//if(particles.size()==0) return ;
	//cout<<"first particle "<<endl;
	//PrintState(particles[0]->state);

	PedestrianState*ped_state;
	ped_state=safe_cast<PedestrianState*>(beliefs.GetSample(0));
	if(beliefs.GetNumSamples()>0)
		DisplayState(*ped_state,cout);
	for(int i=0;i<beliefs.GetNumSamples();i++)
	{
		ped_state=safe_cast<PedestrianState*>(beliefs.GetSample(i));
		for(int j=0;j<ped_state->num;j++)
		{
			goal_count[j][ped_state->PedPoses[j].second]++;
		}
	}


	vector<vector<double> > belief_vec;
	for(int j=0;j<ped_state->num;j++)
	{
		vector<double> belief;
		for(int i=0;i<ModelParams::NGOAL;i++)
		{	
			belief.push_back((goal_count[j][i]+0.0)/beliefs.GetNumSamples());
		}
		belief_vec.push_back(belief);
	}
	return belief_vec;
}

void PEDESTRIAN_CHANGELANE::DisplayBeliefs(const BELIEF_STATE& beliefs, 
    std::ostream& ostr) const
{
	int goal_count[10][10]={0};
	cout<<"Current Belief "<<endl;
	cout<<"particles num "<<beliefs.GetNumSamples()<<endl;
	//if(particles.size()==0) return ;
	//cout<<"first particle "<<endl;
	//PrintState(particles[0]->state);

	PedestrianState*ped_state;
	ped_state=safe_cast<PedestrianState*>(beliefs.GetSample(0));
	if(beliefs.GetNumSamples()>0)
		DisplayState(*ped_state,cout);
	for(int i=0;i<beliefs.GetNumSamples();i++)
	{
		ped_state=safe_cast<PedestrianState*>(beliefs.GetSample(i));
		for(int j=0;j<ped_state->num;j++)
		{
			goal_count[j][ped_state->PedPoses[j].second]++;
		}
	}


	for(int j=0;j<ped_state->num;j++)
	{
		cout<<"Ped "<<j<<" Belief is ";
		for(int i=0;i<ModelParams::NGOAL;i++)
		{
			cout<<(goal_count[j][i]+0.0)/beliefs.GetNumSamples()<<" ";
		}
		cout<<endl;
	}
	/*
    PedestrianState ped_state;
    int goal_count[4]={0};
    double ped_posx=0,ped_posy=0;
    double num=0;



	ostr<<"Number of Beliefs:"<<beliefs.GetNumSamples()<<endl;
    for (int i=0;i<beliefs.GetNumSamples();i++)
    {
    	const STATE* p_state=beliefs.GetSample(i);
    	ped_state=safe_cast<const PedestrianState&>(*p_state);
    	goal_count[ped_state.Goal]++;
    	ped_posx+=ped_state.PedPos.X;
    	ped_posy+=ped_state.PedPos.Y;
    	num=num+1;
    }
    if(num!=0)
    {
			ostr<<"***********current belief***************************"<<endl;
			const PedestrianState*ts;
			ts=safe_cast<const PedestrianState*>(beliefs.GetSample(0));
			ostr<<"Rob Pos :"<<rob_map[ts->RobPos.Y].first<<" "<<rob_map[ts->RobPos.Y].second<<endl;
			ostr<<"Ped Pos :"<<ts->PedPos.X<<" "<<ts->PedPos.Y<<endl;
			ostr<<"Beliefs are "<<goal_count[0]/num<<" "<<goal_count[1]/num<<" "<<goal_count[2]/num<<" "<<goal_count[3]/num<<endl;
			ostr<<"****************************************************"<<endl;
			//ostr<<"Ped Pos X,Y "<<ped_posx/beliefs.GetNumSamples()<<ped_posy/beliefs.GetNumSamples()<<endl;
			//ostr<<"Ped Pos X,Y "<<ped_posx/beliefs.GetNumSamples()<<ped_posy/beliefs.GetNumSamples()<<endl;
    }
    else
    	    printf("Belief has no particle");*/
}

bool PEDESTRIAN_CHANGELANE::Same(const BELIEF_STATE& beliefState1,const BELIEF_STATE& beliefState2,double & distance)
{
	const PedestrianState*PedestrianState1;
	const PedestrianState*PedestrianState2;
	PedestrianState1=safe_cast<const PedestrianState*>(beliefState1.GetSample(0));
	PedestrianState2=safe_cast<const PedestrianState*>(beliefState2.GetSample(0));

	if((PedestrianState1->RobPos.Y)!=(PedestrianState2->RobPos.Y)) return false;
	if((PedestrianState1->Vel)!=(PedestrianState2->Vel)) return false;
	int robX1=rob_map[PedestrianState1->RobPos.Y].first;
	int robY1=rob_map[PedestrianState1->RobPos.Y].second;
	int robX2=rob_map[PedestrianState2->RobPos.Y].first;
	int robY2=rob_map[PedestrianState2->RobPos.Y].second;
	for(int i=0;i<PedestrianState1->num;i++)
	{
		int pedX1=PedestrianState1->PedPoses[i].first.X;
		int pedX2=PedestrianState2->PedPoses[i].first.X;
		int pedY1=PedestrianState1->PedPoses[i].first.Y;
		int pedY2=PedestrianState2->PedPoses[i].first.Y;
		if(!(pedX1==pedX2&&pedY1==pedY2)) return false;
		/*
		int dx1=PedestrianState1->PedPoses[i].first.X-robX1;
		int dx2=PedestrianState2->PedPoses[i].first.X-robX2;
		int dy1=PedestrianState1->PedPoses[i].first.Y-robY1;
		int dy2=PedestrianState2->PedPoses[i].first.Y-robY2;*/

	}


	int goal_counts1[20][20]={0};	
	int goal_counts2[20][20]={0};	
	int N1=beliefState1.GetNumSamples();
	int N2=beliefState2.GetNumSamples();
	if(N1>500) N1=50;
	if(N2>500) N2=50;

	for(int i=0;i<N1;i++)
	{
		
		PedestrianState1=safe_cast<const PedestrianState*>(beliefState1.GetSample(i));
		for(int j=0;j<PedestrianState1->num;j++)
		{
			goal_counts1[j][PedestrianState1->PedPoses[j].second]++;
		}
	}

	for(int i=0;i<N2;i++)
	{
		PedestrianState2=safe_cast<const PedestrianState*>(beliefState2.GetSample(i));
		for(int j=0;j<PedestrianState2->num;j++)
		{
			goal_counts2[j][PedestrianState2->PedPoses[j].second]++;
		}
	}

	distance=0.0;
	for(int j=0;j<PedestrianState1->num;j++)
		for(int g=0;g<ModelParams::NGOAL;g++)
		{
			double p1=(goal_counts1[j][g]+0.0)/N1;
			double p2=(goal_counts2[j][g]+0.0)/N2;
			if(fabs(p1-p2)>distance) distance=fabs(p1-p2);
			if(distance>0.1) return false;
		}
	return true;
}


VNODE* PEDESTRIAN_CHANGELANE::Find_old(VNODE*v1,const HISTORY &h)
{

	bool found=false;
	int i;
	double distance;
	double min_dist=1;
	int min_index=-1;
	for(i=0;i<vnode_old.size();i++)
	{
		//if(vnode_list[i]==v1) 	return 0;

		if(Same(v1->Beliefs(),vnode_old[i]->Beliefs(),distance))
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
		double val1=vnode_old[min_index]->Value.GetValue();
		double val2=v1->Value.GetValue();
		int c1=vnode_old[min_index]->Value.GetCount();
		int c2=v1->Value.GetCount();

		v1->Value.Set(c1+c2,(val1*c1+val2*c2)/(c1+c2));
		
		//delete the old
		vnode_old.erase(vnode_old.begin()+min_index);
		return v1; 
	}
	else
	{
		return 0;
	}
}
VNODE* PEDESTRIAN_CHANGELANE::Find(VNODE*v1,const HISTORY &h)
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
		//	Find_old(v1,h);
			vnode_list.push_back(v1);
			history_list.push_back(h);
			v1->inserted=true;
			return v1;
	}
	
}

VNODE* PEDESTRIAN_CHANGELANE::MapBelief(VNODE*vn,VNODE*r,const HISTORY &h,int historyDepth)
{

	//cout<<"Map Belief"<<endl;
	if(vn==0) 	return 0;
	if(vn->inserted) return vn;
	BELIEF_STATE&beliefState=vn->Beliefs();
	//if(beliefState.GetNumSamples()<1000000)   	return vn;

	if(vn->visit_count<20)   
	{
		//VNODE*parent;
		//if(
		return vn;
	}
	/*
	else if(vn->visit_count<20)
	{

		for(int i=0;i<20;i++)
		{
			VNODE*prt=r;
			for(int i=historyDepth;i<h.Size()-1;i++)
			{
				prt=prt->Child(h[i].Action).Child(h[i].Observation);
			}
			//VNODE*prt=vn.GetParent();
			STATE*prt_state=prt->Beliefs().CreateSample(*this);
			double rwd;
			int obs;
			Step(*prt_state,h.Back().Action,obs,rwd);
			if(prt->Child(h.Back().Action).Child(obs))
				prt->Child(h.Back().Action).Child(obs)->Beliefs().AddSample(prt_state);
			//vn->Beliefs().AddSample(prt_state);
		}

		return vn;

	}*/

//	if(vn->visit_count<20||beliefState.GetNumSamples()<20) return vn;


	VNODE*nn=Find(vn,h);

	/*
	if(nn==vn)
	{
		vn->merged=true;
		vn->merge_node=nn;
	}*/
	return nn;	
}

void PEDESTRIAN_CHANGELANE::ModifyObsStates(BELIEF_STATE &beliefs,STATE*obs_state) const
{
	if(debug)  cout<<"num samples to modify "<<beliefs.GetNumSamples()<<endl;
	PedestrianState* obs_ped_state=safe_cast<PedestrianState*>(obs_state);
	for(int i=0;i<beliefs.GetNumSamples();i++)
	{
		PedestrianState* curr_state=safe_cast<PedestrianState*>(beliefs.GetSample(i));	
		for(int j=0;j<obs_ped_state->num;j++)
		{
			obs_ped_state->PedPoses[j].second=-1;
			for(int k=0;k<curr_state->num;k++)
			{
				if(curr_state->PedPoses[k].third==obs_ped_state->PedPoses[j].third) {
					obs_ped_state->PedPoses[j].second=curr_state->PedPoses[k].second;
					break;
				}
			}
			if(obs_ped_state->PedPoses[j].second==-1)  //new state,assign a random goal
			{
				double prob=(rand()+0.0) / RAND_MAX;
				int x=obs_ped_state->PedPoses[j].first.X;
				int y=obs_ped_state->PedPoses[j].first.Y;

				int sum=0;
				int g;
				for(g=0;g<ModelParams::NGOAL;g++)
				{
					if(fabs(sfm->local_goals[g][0]-x)<ModelParams::GOAL_DIST&&fabs(sfm->local_goals[g][1]-y)<ModelParams::GOAL_DIST)
					{sum++;}
				}
				int next=prob*sum;
				sum=0;
				for(g=0;g<ModelParams::NGOAL;g++)
				{
					if(fabs(sfm->local_goals[g][0]-x)<ModelParams::GOAL_DIST&&fabs(sfm->local_goals[g][1]-y)<ModelParams::GOAL_DIST)
					{sum++;}
					if(sum>next) break;
				}
				//obs_ped_state->PedPoses[j].second=prob*ModelParams::NGOAL;
				obs_ped_state->PedPoses[j].second=g;
			}
			/*
			if(obs_ped_state->PedPoses[j].second==-1)
			{
				obs_ped_state->PedPoses[j].second=rand()%ModelParams::NGOAL;
			}*/
		}
		*curr_state=*obs_ped_state;
	}
}
void PEDESTRIAN_CHANGELANE::RobStep(PedestrianState& pedestrian_state, int action) const 
{
	int &robX=pedestrian_state.RobPos.X;
	int &robY=pedestrian_state.RobPos.Y;
	int &rob_vel=pedestrian_state.Vel;

	double vel_p=(rand()+0.0)/RAND_MAX;
	if(pedestrian_state.Vel==0)
	{
		if(vel_p<0.9)   
		{
		}
		else
		{
			robY++;
		}
		
	}
	else if(pedestrian_state.Vel==1)
	{
		if(vel_p<0.8) robY++;
		else if(vel_p<0.9) robY+=2;
	}
	else
	{
		if(vel_p<0.8) robY+=2;
		else if(vel_p<0.9) robY+=1;
	}

	if(robY>rob_map.size()-1) robY=rob_map.size()-1;

	double act_p=(rand()+0.0)/RAND_MAX;

	if(action==1)
	{
		if(rob_vel==0)
		{
			if(act_p<0.7) rob_vel++;
			else if(act_p<0.8) rob_vel+=2;
			else rob_vel=0;
		}
		else if(rob_vel==1)
		{
			if(act_p<0.8) rob_vel=2;
			else if(act_p<0.9) rob_vel=1;
			else rob_vel=0;
		}
		else if(rob_vel==2)
		{
			if(act_p<0.8) rob_vel=2;
			else if(act_p<0.9) rob_vel=1;
			else rob_vel=0;
		}
	}
	else if(action==2)
	{
		if(rob_vel==0)
		{
			if(act_p<0.9) {}
			else {rob_vel++;}
		}
		else if(rob_vel==1)
		{
			if(act_p<0.9){rob_vel--;}
		}
		else if(rob_vel==2)
		{
			if(act_p<0.7){rob_vel--;}
			else if(act_p<0.8) {}
			else {rob_vel-=2;}
		}	
	}
	else
	{
		if(rob_vel==0)
		{
			if(act_p<0.9) rob_vel=0;
			else rob_vel=1;
		}
		else if(rob_vel==1)
		{
			if(act_p<0.7) rob_vel=1;
			else if(act_p<0.8) rob_vel=2;
			else rob_vel=0;
		}
		else if(rob_vel==2)
		{
			if(act_p<0.8) rob_vel=2;
			else if(act_p<0.9) rob_vel=1;
			else 	rob_vel=0;
		}
	}


	if(rob_vel<0) rob_vel=0;
	if(rob_vel>2) rob_vel=2;



}


void PEDESTRIAN_CHANGELANE::PedStep(PedestrianState& state) const
{	
	if(sfm==0) {
		cerr<<"!!!!!!SFM not initialized"<<endl;
		return;
	}
	UtilUniform unif;
	unif.seed_=rand();
	sfm->ModelTransFast(state,unif);   //make transition using SFM Model
}

bool PEDESTRIAN_CHANGELANE::Step(STATE& state,int action, OBS_TYPE & observation,double&reward) const
{
	if(debug)
		cout<<"rob map size "<<rob_map.size()<<endl;
	PedestrianState& pedestrian_state=safe_cast<PedestrianState&>(state);
	reward=0;
	observation= 2<<63-1; //MaxObs()-1;


	int robY=pedestrian_state.RobPos.Y;
	int rob_vel=pedestrian_state.Vel;
	if(robY >= rob_map.size()-1) {
		reward = GOAL_REWARD;
		return true;
	}
	for(int i=0;i<pedestrian_state.num;i++)
	{
		int &pedX = pedestrian_state.PedPoses[i].first.X;
		int &pedY = pedestrian_state.PedPoses[i].first.Y;

		if(pedX==rob_map[robY].first && rob_map[robY].second==pedY) {
			reward = CRASH_PENALTY;
			return true;
		}


		if(pedX==rob_map[robY].first && pedY==rob_map[robY].second+1) {
			if((rob_vel==0 && action==1) || (rob_vel == 1 && action < 2) || rob_vel == 2) {
				reward = CRASH_PENALTY;
				return true;
			}
		}
		if(pedX==rob_map[robY].first && pedY==rob_map[robY].second+2) {
			if((rob_vel==1&&action==1) || (rob_vel == 2 && action < 2)) {
				reward=CRASH_PENALTY;
				return true;
			}
		}
	}
	
/*	
	if(pedX==rob_map[robY].first&&pedY==rob_map[robY].second+1)
	{
		if(rob_vel==0&&action==1)  {reward=CRASH_PENALTY;return true;}
		if(rob_vel==1&&action<2)   {reward=CRASH_PENALTY;return true;}
		if(rob_vel==2)     			{reward=CRASH_PENALTY;return true;}
	}
	if(pedX==rob_map[robY].first&&pedY==rob_map[robY].second+2)
	{
		if(rob_vel==1&&action==1)  {reward=CRASH_PENALTY;return true;}
		if(rob_vel==2&&action<2)  {reward=CRASH_PENALTY;return true;}
	}*/


	double p = (rand()+0.0)/RAND_MAX;
	robY += robotNoisyMove[rob_vel][lookup(robotMoveProbs[rob_vel], p)];
	//if(robY >= rob_map) robY = Y_SIZE - 1;
	if(robY>=rob_map.size()-1) robY=rob_map.size()-1;
	p = (rand()+0.0)/RAND_MAX;
	rob_vel = robotVelUpdate[action][rob_vel][lookup(robotUpdateProb[action][rob_vel], p)];

	//RobStep(pedestrian_state,action);
	sfm->debug=debug;
	PedStep(pedestrian_state);
	sfm->debug=false;

	pedestrian_state.Vel=rob_vel;
	pedestrian_state.RobPos.Y=robY;

	reward=-50;
	observation=Observe(pedestrian_state);
	if(debug)
		cout<<"observation "<<observation<<endl;
	rob_vel=0;
	robY=0;
	return false;
}

OBS_TYPE PEDESTRIAN_CHANGELANE::Observe(const PedestrianState state) const
{
	OBS_TYPE obs=0;// = state.Vel*(X_SIZE*Y_SIZE*rob_map.size())+state.RobPos.Y*(X_SIZE*Y_SIZE)+state.PedPos.X*Y_SIZE+state.PedPos.Y;
	OBS_TYPE robObs=state.Vel+state.RobPos.Y*ModelParams::VEL_N;
	OBS_TYPE robObsMax=ModelParams::VEL_N*ModelParams::RMMax;  //max length of the rob_map
	OBS_TYPE pedObsMax=ModelParams::XSIZE*ModelParams::YSIZE;
	OBS_TYPE pedObs=0;
	for(int i=0;i<state.num;i++)
	{
		OBS_TYPE this_obs=state.PedPoses[i].first.X*Y_SIZE+state.PedPoses[i].first.Y;	
		pedObs=pedObs*pedObsMax+this_obs;
	}
	obs=pedObs*robObsMax+robObs;
	return obs;	
}




