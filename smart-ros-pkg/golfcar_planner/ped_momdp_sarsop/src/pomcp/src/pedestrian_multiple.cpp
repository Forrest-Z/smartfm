#include "pedestrian_multiple.h"
#include "utils.h"
#include <string>
#include <math.h>
#include <fstream>

using namespace std;
using namespace UTILS;



PEDESTRIAN_MULTIPLE_STATE*PEDESTRIAN_MULTIPLE_state1;
PEDESTRIAN_MULTIPLE_STATE*PEDESTRIAN_MULTIPLE_state2;
const int CRUSH=-50000;
PEDESTRIAN_MULTIPLE::PEDESTRIAN_MULTIPLE(int size=9):Size(size)
{
	Discount=0.95;
	//NumObservations=5*Size+1;
	NumObservations=2000;
	RewardRange=50000;
	NumActions=3;
	//UpdateModel(0);
	//current_horizon=0;
	PEDESTRIAN_MULTIPLE_state1=MemoryPool.Allocate();
	PEDESTRIAN_MULTIPLE_state2=MemoryPool.Allocate();
	InitModel();
	//client_pt=&client;

}

STATE*PEDESTRIAN_MULTIPLE::Copy(const STATE &state) const
{
	const PEDESTRIAN_MULTIPLE_STATE& PEDESTRIAN_MULTIPLE_state=safe_cast<const PEDESTRIAN_MULTIPLE_STATE&>(state);
	PEDESTRIAN_MULTIPLE_STATE*newstate=MemoryPool.Allocate();
	*newstate=PEDESTRIAN_MULTIPLE_state;
	return newstate;
}

void PEDESTRIAN_MULTIPLE::Validate(const STATE& state) const
{
}

void PEDESTRIAN_MULTIPLE::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const PEDESTRIAN_MULTIPLE_STATE& PEDESTRIAN_MULTIPLE_state = safe_cast<const PEDESTRIAN_MULTIPLE_STATE&>(state);
    ostr<<"Rob Pos : "<<PEDESTRIAN_MULTIPLE_state.RobPos.X<<" "<<PEDESTRIAN_MULTIPLE_state.RobPos.Y<<endl;
    ostr<<"Ped Pos : "<<PEDESTRIAN_MULTIPLE_state.PedPos.X<<" "<<PEDESTRIAN_MULTIPLE_state.PedPos.Y<<endl;
}
STATE*PEDESTRIAN_MULTIPLE::CreateStartState() const
{
	PEDESTRIAN_MULTIPLE_STATE*PEDESTRIAN_MULTIPLE_state=MemoryPool.Allocate();
	PEDESTRIAN_MULTIPLE_state->RobPos.X=1;
	PEDESTRIAN_MULTIPLE_state->RobPos.Y=0;

	PEDESTRIAN_MULTIPLE_state->Vel=1.0;


	int rand_pose=rand()%40;
	PEDESTRIAN_MULTIPLE_state->PedPos.X=rand_pose/10;
	PEDESTRIAN_MULTIPLE_state->PedPos.Y=rand_pose-PEDESTRIAN_MULTIPLE_state->PedPos.X*10;


	int p=rand()%4;
	PEDESTRIAN_MULTIPLE_state->Goal=p;
	return PEDESTRIAN_MULTIPLE_state;
}

void PEDESTRIAN_MULTIPLE::FreeState(STATE* state) const
{
    PEDESTRIAN_MULTIPLE_STATE* PEDESTRIAN_MULTIPLE_state = safe_cast<PEDESTRIAN_MULTIPLE_STATE*>(state);
    MemoryPool.Free(PEDESTRIAN_MULTIPLE_state);
}


void PEDESTRIAN_MULTIPLE::InitModel()
{
	for(int i=0;i<4;i++)
	{
		if(i==1) continue;
		for(int j=0;j<10;j++)
		{
			map[i][j]=0;	
		}
	}
}


int PEDESTRIAN_MULTIPLE::map[4][10];
void PEDESTRIAN_MULTIPLE::UpdateModel(int i)
{
	for(int i=0;i<4;i++)
	{
		if(i==1) continue;
		for(int j=0;j<10;j++)
		{
			int p=rand()%3;
			//map[i][j]=0;
			if(p>0) 	map[i][j]=0;
			else 		map[i][j]=1;
			map[i][j]=0;
		}
	}
	/*
	cout<<endl;
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<10;j++)
			cout<<map[i][j];
		cout<<endl;
	}*/
}

void PEDESTRIAN_MULTIPLE::DisplayBeliefs(const BELIEF_STATE& beliefs, 
    std::ostream& ostr) const
{
    PEDESTRIAN_MULTIPLE_STATE PEDESTRIAN_MULTIPLE_state;
    int goal_count[4]={0};
    double ped_posx=0,ped_posy=0;
    double num=0;




    for (int i=0;i<beliefs.GetNumSamples();i++)
    {
    	const STATE* p_PEDESTRIAN_MULTIPLE_state=beliefs.GetSample(i);
    	PEDESTRIAN_MULTIPLE_state=safe_cast<const PEDESTRIAN_MULTIPLE_STATE&>(*p_PEDESTRIAN_MULTIPLE_state);
    	goal_count[PEDESTRIAN_MULTIPLE_state.Goal]++;
    	ped_posx+=PEDESTRIAN_MULTIPLE_state.PedPos.X;
    	ped_posy+=PEDESTRIAN_MULTIPLE_state.PedPos.Y;
    	num=num+1;
    }
    if(num!=0)
    {
			ostr<<"***********current belief***************************"<<endl;
			const PEDESTRIAN_MULTIPLE_STATE*ts;
			ts=safe_cast<const PEDESTRIAN_MULTIPLE_STATE*>(beliefs.GetSample(0));
			ostr<<"Rob Pos :"<<ts->RobPos.X<<" "<<ts->RobPos.Y<<endl;
			ostr<<"Ped Pos :"<<ts->PedPos.X<<" "<<ts->PedPos.Y<<endl;
			ostr<<"Beliefs are "<<goal_count[0]/num<<" "<<goal_count[1]/num<<" "<<goal_count[2]/num<<" "<<goal_count[3]/num<<endl;
			ostr<<"****************************************************"<<endl;
			//ostr<<"Ped Pos X,Y "<<ped_posx/beliefs.GetNumSamples()<<ped_posy/beliefs.GetNumSamples()<<endl;
			//ostr<<"Ped Pos X,Y "<<ped_posx/beliefs.GetNumSamples()<<ped_posy/beliefs.GetNumSamples()<<endl;
    }
    else
    	    printf("Belief has no particle");
}

bool PEDESTRIAN_MULTIPLE::Same(const BELIEF_STATE& beliefState1,const BELIEF_STATE& beliefState2,double & distance)
{
	const PEDESTRIAN_MULTIPLE_STATE*PEDESTRIAN_MULTIPLE_state1;
	const PEDESTRIAN_MULTIPLE_STATE*PEDESTRIAN_MULTIPLE_state2;
	PEDESTRIAN_MULTIPLE_state1=safe_cast<const PEDESTRIAN_MULTIPLE_STATE*>(beliefState1.GetSample(0));
	PEDESTRIAN_MULTIPLE_state2=safe_cast<const PEDESTRIAN_MULTIPLE_STATE*>(beliefState2.GetSample(0));
	int dx1=PEDESTRIAN_MULTIPLE_state1->PedPos.X-PEDESTRIAN_MULTIPLE_state1->RobPos.X;
	int dx2=PEDESTRIAN_MULTIPLE_state2->PedPos.X-PEDESTRIAN_MULTIPLE_state2->RobPos.X;
	int dy1=PEDESTRIAN_MULTIPLE_state1->PedPos.Y-PEDESTRIAN_MULTIPLE_state1->RobPos.Y;
	int dy2=PEDESTRIAN_MULTIPLE_state2->PedPos.Y-PEDESTRIAN_MULTIPLE_state2->RobPos.Y;
	
	//if((dx1!=dx2)||(dy1!=dy2)) 	return false;
	if((PEDESTRIAN_MULTIPLE_state1->PedPos.X!=PEDESTRIAN_MULTIPLE_state2->PedPos.X)||(PEDESTRIAN_MULTIPLE_state1->PedPos.Y!=PEDESTRIAN_MULTIPLE_state2->PedPos.Y)) return false;
	if((PEDESTRIAN_MULTIPLE_state1->RobPos.X!=PEDESTRIAN_MULTIPLE_state2->RobPos.X)||(PEDESTRIAN_MULTIPLE_state1->RobPos.Y!=PEDESTRIAN_MULTIPLE_state2->RobPos.Y)) return false;
	if(PEDESTRIAN_MULTIPLE_state1->Vel!=PEDESTRIAN_MULTIPLE_state2->Vel) 	return false;

	int g1=0,g2=0;
	for(int i=0;i<beliefState1.GetNumSamples();i++)
	{
		PEDESTRIAN_MULTIPLE_state1=safe_cast<const PEDESTRIAN_MULTIPLE_STATE*>(beliefState1.GetSample(i));
		if(PEDESTRIAN_MULTIPLE_state1->Goal==1) g1++;
	}
	for(int i=0;i<beliefState2.GetNumSamples();i++)
	{
		PEDESTRIAN_MULTIPLE_state2=safe_cast<const PEDESTRIAN_MULTIPLE_STATE*>(beliefState2.GetSample(i));
		if(PEDESTRIAN_MULTIPLE_state2->Goal==1) g2++;
	}

	double p1=(g1+0.0)/beliefState1.GetNumSamples();
	double p2=(g2+0.0)/beliefState2.GetNumSamples();
	distance=fabs(p1-p2);
	if(distance>0.05) return false;
	return true;
}


VNODE* PEDESTRIAN_MULTIPLE::Find_old(VNODE*v1,const HISTORY &h)
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
VNODE* PEDESTRIAN_MULTIPLE::Find(VNODE*v1,const HISTORY &h)
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

VNODE* PEDESTRIAN_MULTIPLE::MapBelief(VNODE*vn,VNODE*r,const HISTORY &h,int historyDepth)
{

	//cout<<"Map Belief"<<endl;
	//return vn;
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



bool PEDESTRIAN_MULTIPLE::Step(STATE& state,int action, int & observation,double&reward) const
{
	PEDESTRIAN_MULTIPLE_STATE& pedestrian_state=safe_cast<PEDESTRIAN_MULTIPLE_STATE&>(state);
	reward=0;
	observation=1000;
	int &pedX=pedestrian_state.PedPos.X;
	int &pedY=pedestrian_state.PedPos.Y;
	int pedX_old=pedX;
	int pedY_old=pedY;

	int &robX=pedestrian_state.RobPos.X;
	int &robY=pedestrian_state.RobPos.Y;
	int &rob_vel=pedestrian_state.Vel;

	if(robY>=9)
	{
		reward=1000;
		return true;
	}
	if(pedX==1&&robY==pedY)
	{
		reward=CRUSH;
		return true;
	}
	if(pedX==1&&robY==pedY+1)
	{
		if(rob_vel==0&&action==1)  {reward=CRUSH;return true;}
		if(rob_vel==1&&action<2)   {reward=CRUSH;return true;}
		if(rob_vel==2)     			{reward=CRUSH;return true;}
	}
	if(pedX==1&&robY==pedY+2)
	{
		if(rob_vel==1&&action==1)  {reward=CRUSH;return true;}
		if(rob_vel==2&&action<2)  {reward=CRUSH;return true;}
	}
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

	if(robY>9) robY=9;
//	pedestrian_state.RobPos.Y=pedestrian_state.RobPos.Y+pedestrian_state.Vel;

	double act_p=(rand()+0.0)/RAND_MAX;

	if(action==1)
	{
		if(act_p<0.8)  
		{
			rob_vel++;
		}
		else if(act_p<0.9) 
		{
			if(rob_vel==0) rob_vel+=2;
			else  rob_vel--;
		}
		else 
		{
			if(rob_vel==2) rob_vel-=2;
		}
	}
	else if(action==2)
	{
		if(act_p<0.8)
		{
			rob_vel--;	
		}
		else if(act_p<0.9)
		{
			if(rob_vel==2) rob_vel-=2;
			else 	rob_vel++;
		}
		else
		{
		    if(rob_vel==0) rob_vel+=2;
		}
	}
	else
	{
		if(act_p<0.8){}
		else if(act_p<0.9)
		{
			if(rob_vel==0) rob_vel++;
			else  rob_vel--;
		}
		else
		{
			if(rob_vel==0) rob_vel+=2;
			else if(rob_vel==1) rob_vel++;
			else rob_vel-=2;
		}
	}


	if(rob_vel<0) rob_vel=0;
	if(rob_vel>2) rob_vel=2;

	double p=(rand()+0.0)/RAND_MAX;




	if(pedestrian_state.Goal==0)
	{
		if(pedX==0&&pedY==0)
		{
			
		}
		else if(pedX==0)
		{
			if(pedY==9)
			{
				if(p<0.425) pedY--;
			}
			else
			{
				if(p<0.175) pedY++;
				else if(p<0.6) pedY--;
			}
		}
		else if(pedX==3)
		{
			if(pedY==0)
			{
				if(p<0.425) 	pedX--;
			}
			else if(pedY==9)
			{
				if(p<0.325) 	pedX--;
				else if(p<0.65) pedY--;
				else if(p<0.75) {pedX--;pedY--;}
			}
			else
			{
				if(p<0.25) pedX--;
				else if(p<0.575) pedY--;
				else if(p<0.675) pedY++;
				else if(p<0.775) {pedX--;pedY--;}
				else if(p<0.85)  {pedX--;pedY++;}
			}
		}
		else
		{
			if(pedY==0)
			{
				if(p<0.425) pedX--;
				else if(p<0.6) pedX++;
			}
			else if(pedY==9)
			{
				if(p<0.325) pedX--;
				else if(p<0.575) pedY--;
				else if(p<0.675) pedX++;
				else if(p<0.775) {pedX--;pedY--;}
				else if(p<0.85)  {pedX++;pedY--;}
			}
			else
			{
				if(p<0.25)  pedX--;
				else if(p<0.5) pedY--;
				else if(p<0.55) pedX++;
				else if(p<0.6) pedY++;
				else if(p<0.7) {pedX--;pedY--;}
				else if(p<0.75) {pedX++;pedY++;}
				else if(p<0.825) {pedX++;pedY--;}
				else if(p<0.9)   {pedX--;pedY++;}
			}
			
		}

	}
	else if(pedestrian_state.Goal==1)
	{
		if(pedX==3&&pedY==0)
		{
		
		}
		else if(pedX==3)
		{
			if(pedY==9)
			{
				if(p<0.425) pedY--;
			}
			else
			{
				if(p<0.175) pedY++;
				else if(p<0.6) pedY--;
			}
		}
		else if(pedX==0)
		{
			if(pedY==0)
			{
				if(p<0.425) 	pedX++;
			}
			else if(pedY==9)
			{
				if(p<0.325) 	pedX++;
				else if(p<0.65) pedY--;
				else if(p<0.75) {pedX++;pedY--;}
			}
			else
			{
				if(p<0.25) pedX++;
				else if(p<0.575) pedY--;
				else if(p<0.675) pedY++;
				else if(p<0.775) {pedX++;pedY--;}
				else if(p<0.85)  {pedX++;pedY++;}
			}
		}
		else
		{
			if(pedY==0)
			{
				if(p<0.425) pedX++;
				else if(p<0.6) pedX--;
			}
			else if(pedY==9)
			{
				if(p<0.325) pedX++;
				else if(p<0.575) pedY--;
				else if(p<0.675) pedX--;
				else if(p<0.775) {pedX++;pedY--;}
				else if(p<0.85)  {pedX--;pedY--;}
			}
			else
			{
				if(p<0.25)  pedX++;
				else if(p<0.5) pedY--;
				else if(p<0.55) pedX--;
				else if(p<0.6) pedY++;
				else if(p<0.7) {pedX++;pedY--;}
				else if(p<0.75) {pedX--;pedY++;}
				else if(p<0.825) {pedX--;pedY--;}
				else if(p<0.9)   {pedX++;pedY++;}
			}
			
		}


	}
	else if(pedestrian_state.Goal==2)
	{
		if(pedX==3&&pedY==9)
		{
		
		}
		else if(pedX==3)
		{
			if(pedY==0)
			{
				if(p<0.425) pedY++;
			}
			else
			{
				if(p<0.175) pedY--;
				else if(p<0.6) pedY++;
			}
		}
		else if(pedX==0)
		{
			if(pedY==9)
			{
				if(p<0.425) 	pedX++;
			}
			else if(pedY==0)
			{
				if(p<0.325) 	pedX++;
				else if(p<0.65) pedY++;
				else if(p<0.75) {pedX++;pedY++;}
			}
			else
			{
				if(p<0.25) pedX++;
				else if(p<0.575) pedY++;
				else if(p<0.675) pedY--;
				else if(p<0.775) {pedX++;pedY++;}
				else if(p<0.85)  {pedX++;pedY--;}
			}
		}
		else
		{
			if(pedY==9)
			{
				if(p<0.425) pedX++;
				else if(p<0.6) pedX--;
			}
			else if(pedY==0)
			{
				if(p<0.325) pedX++;
				else if(p<0.575) pedY++;
				else if(p<0.675) pedX--;
				else if(p<0.775) {pedX++;pedY++;}
				else if(p<0.85)  {pedX--;pedY++;}
			}
			else
			{
				if(p<0.25)  pedX++;
				else if(p<0.5) pedY++;
				else if(p<0.55) pedX--;
				else if(p<0.6) pedY--;
				else if(p<0.7) {pedX++;pedY++;}
				else if(p<0.75) {pedX--;pedY--;}
				else if(p<0.825) {pedX--;pedY++;}
				else if(p<0.9)   {pedX++;pedY--;}
			}
			
		}


	}
	if(pedestrian_state.Goal==3)
	{
		if(pedX==0&&pedY==9)
		{
		
		}
		else if(pedX==0)
		{
			if(pedY==0)
			{
				if(p<0.425) pedY++;
			}
			else
			{
				if(p<0.175) pedY--;
				else if(p<0.6) pedY++;
			}
		}
		else if(pedX==3)
		{
			if(pedY==9)
			{
				if(p<0.425) 	pedX--;
			}
			else if(pedY==0)
			{
				if(p<0.325) 	pedX--;
				else if(p<0.65) pedY++;
				else if(p<0.75) {pedX--;pedY++;}
			}
			else
			{
				if(p<0.25) pedX--;
				else if(p<0.575) pedY++;
				else if(p<0.675) pedY--;
				else if(p<0.775) {pedX--;pedY++;}
				else if(p<0.85)  {pedX--;pedY--;}
			}
		}
		else
		{
			if(pedY==9)
			{
				if(p<0.425) pedX--;
				else if(p<0.6) pedX++;
			}
			else if(pedY==0)
			{
				if(p<0.325) pedX--;
				else if(p<0.575) pedY++;
				else if(p<0.675) pedX++;
				else if(p<0.775) {pedX--;pedY++;}
				else if(p<0.85)  {pedX++;pedY++;}
			}
			else
			{
				if(p<0.25)  pedX--;
				else if(p<0.5) pedY++;
				else if(p<0.55) pedX++;
				else if(p<0.6) pedY--;
				else if(p<0.7) {pedX--;pedY++;}
				else if(p<0.75) {pedX++;pedY--;}
				else if(p<0.825) {pedX++;pedY++;}
				else if(p<0.9)   {pedX--;pedY--;}
			}
			
		}

	}

	if(map[pedX][pedY]==1)
	{
		pedX=pedX_old;
		pedY=pedY_old;
	}
	reward=-50;
	observation=1;
	observation=rob_vel*500+robY*41+pedX*11+pedY;
//	observation=pedestrian_state.PedPos.X*30+pedestrian_state.PedPos.Y;


	/*	
	if(p<0.1)
	{
		if(pedestrian_state.PedPos.X+1<4)
			observation=(pedestrian_state.PedPos.X+1)*30+pedestrian_state.PedPos.Y;
	}
	else if(p<0.2)
	{
		if(pedestrian_state.PedPos.X-1>0)
			observation=(pedestrian_state.PedPos.X-1)*30+pedestrian_state.PedPos.Y;
	}
	else if(p<0.3)
	{
		if(pedestrian_state.PedPos.Y+1<Size-1)
			observation=pedestrian_state.PedPos.X*30+pedestrian_state.PedPos.Y+1;
	}
	else if(p<0.4)
	{
		if(pedestrian_state.PedPos.Y-1>0)
			observation=pedestrian_state.PedPos.X*30+pedestrian_state.PedPos.Y-1;
	}
	else
	{
		observation=pedestrian_state.PedPos.X*30+pedestrian_state.PedPos.Y;
	}*/

	return false;
}



int PEDESTRIAN_MULTIPLE::GetNumOfStates()
{
	return 5*Size*Size*3*2;	
}
int PEDESTRIAN_MULTIPLE::StateToN(STATE* state)
{
	PEDESTRIAN_MULTIPLE_STATE* PEDESTRIAN_MULTIPLE_state=safe_cast<PEDESTRIAN_MULTIPLE_STATE*>(state);	
	int add_goal=(5*Size*Size*3)*(PEDESTRIAN_MULTIPLE_state->Goal==3);
	int add_vel=(5*Size*Size)*PEDESTRIAN_MULTIPLE_state->Vel;
	int add_Y1=(5*Size)*PEDESTRIAN_MULTIPLE_state->RobPos.Y;
	int add_Y2=5*PEDESTRIAN_MULTIPLE_state->PedPos.Y;
	int add_X=PEDESTRIAN_MULTIPLE_state->PedPos.X;
	int N=add_X+add_Y2+add_Y1+add_vel+add_goal;
	return N;
	//PEDESTRIAN_MULTIPLE_state.RobPos.Y
}

void PEDESTRIAN_MULTIPLE::NToState(int n,PEDESTRIAN_MULTIPLE_STATE*PEDESTRIAN_MULTIPLE_state)
{
	//5 lanes
	int N=n;
	//PEDESTRIAN_MULTIPLE_STATE*PEDESTRIAN_MULTIPLE_state=MemoryPool.Allocate();
	PEDESTRIAN_MULTIPLE_state->Goal=(N/(5*Size*Size*3)==0?1:3);
	N=N%(5*Size*Size*3);
	PEDESTRIAN_MULTIPLE_state->Vel=N/(5*Size*Size);
	N=N%(5*Size*Size);
	PEDESTRIAN_MULTIPLE_state->RobPos.Y=N/(5*Size);
	N=N%(5*Size);
	PEDESTRIAN_MULTIPLE_state->PedPos.Y=N/5;
	N=N%5;
	PEDESTRIAN_MULTIPLE_state->PedPos.X=N;
	//return PEDESTRIAN_MULTIPLE_state;
}


double PEDESTRIAN_MULTIPLE::TransFn(int s1,int a,int s2)
{
	
	NToState(s1,PEDESTRIAN_MULTIPLE_state1);
	NToState(s2,PEDESTRIAN_MULTIPLE_state2);
	//return 0.0;
	
	//3 terminating states
	
	
	/*strict condition*/
	
	if(PEDESTRIAN_MULTIPLE_state1->RobPos.Y==PEDESTRIAN_MULTIPLE_state1->PedPos.Y&&(2==PEDESTRIAN_MULTIPLE_state1->PedPos.X||1==PEDESTRIAN_MULTIPLE_state1->PedPos.X||3==PEDESTRIAN_MULTIPLE_state1->PedPos.X))
	{
		return 0.0;
	}
	
	if(PEDESTRIAN_MULTIPLE_state1->RobPos.Y+1==PEDESTRIAN_MULTIPLE_state1->PedPos.Y&&(2==PEDESTRIAN_MULTIPLE_state1->PedPos.X||1==PEDESTRIAN_MULTIPLE_state1->PedPos.X||3==PEDESTRIAN_MULTIPLE_state1->PedPos.X)&&PEDESTRIAN_MULTIPLE_state1->Vel>0)
	{
		return 0.0;
	}
	
	/*loose constraint*/
	/*
	if(PEDESTRIAN_MULTIPLE_state1->RobPos.Y==PEDESTRIAN_MULTIPLE_state1->PedPos.Y&&(2==PEDESTRIAN_MULTIPLE_state1->PedPos.X))
	{
		return 0.0;
	}
	
	if(PEDESTRIAN_MULTIPLE_state1->RobPos.Y+1==PEDESTRIAN_MULTIPLE_state1->PedPos.Y&&(2==PEDESTRIAN_MULTIPLE_state1->PedPos.X)&&PEDESTRIAN_MULTIPLE_state1->Vel>0)n.
	{
		return 0.0;
	}*/
	
	
	
	if(PEDESTRIAN_MULTIPLE_state1->RobPos.Y>=PEDESTRIAN_MULTIPLE_state1->PedPos.Y)	//overtake,finish
	{
		//ROS_INFO("enter");
		//ROS_INFO("goal %d ry %d px %d py %d",PEDESTRIAN_MULTIPLE_state1->Goal,PEDESTRIAN_MULTIPLE_state1->RobPos.Y,PEDESTRIAN_MULTIPLE_state1->PedPos.X,PEDESTRIAN_MULTIPLE_state1->PedPos.Y);
		return 0.0;
	}
	
	if(PEDESTRIAN_MULTIPLE_state1->RobPos.Y>=Size-1||PEDESTRIAN_MULTIPLE_state1->PedPos.Y>=Size-1)
	{
		return 0.0;
	}
	
	int delta;
	int action=a;
	if (action==ACT_ACC)
	{
		delta=PEDESTRIAN_MULTIPLE_state1->Vel+1;
		if(delta>2)	delta=2;
	}
	else if (action==ACT_DEC)
	{
		delta=PEDESTRIAN_MULTIPLE_state1->Vel-1;
		if(delta<0)	delta=0;
	}
	else
	{
		delta=PEDESTRIAN_MULTIPLE_state1->Vel;
	}
	
	if(PEDESTRIAN_MULTIPLE_state1->Goal==1&&delta==PEDESTRIAN_MULTIPLE_state2->Vel)
	{
		if(PEDESTRIAN_MULTIPLE_state2->Goal==1&&PEDESTRIAN_MULTIPLE_state2->RobPos.Y==PEDESTRIAN_MULTIPLE_state1->RobPos.Y+delta)		//no uncertainty in robot motion
		{	
			if(PEDESTRIAN_MULTIPLE_state2->PedPos.X==PEDESTRIAN_MULTIPLE_state1->PedPos.X&&PEDESTRIAN_MULTIPLE_state2->PedPos.Y==PEDESTRIAN_MULTIPLE_state1->PedPos.Y)
			{
				//ROS_INFO("enter");
				return 0.1;
			}
			if(PEDESTRIAN_MULTIPLE_state2->PedPos.X==PEDESTRIAN_MULTIPLE_state1->PedPos.X&&PEDESTRIAN_MULTIPLE_state2->PedPos.Y==PEDESTRIAN_MULTIPLE_state1->PedPos.Y+1)
			{
				if(PEDESTRIAN_MULTIPLE_state1->PedPos.X==0)	return 0.9;
				else	return 0.3;
			}
			if(PEDESTRIAN_MULTIPLE_state2->PedPos.X==PEDESTRIAN_MULTIPLE_state1->PedPos.X-1&&PEDESTRIAN_MULTIPLE_state2->PedPos.Y==PEDESTRIAN_MULTIPLE_state1->PedPos.Y)
			{
				if(PEDESTRIAN_MULTIPLE_state1->PedPos.Y==Size-1)	return 0.9;
				else	return 0.7;
			}
			
			/*noise case  go back*/
			if(PEDESTRIAN_MULTIPLE_state2->PedPos.X==PEDESTRIAN_MULTIPLE_state1->PedPos.X+1&&PEDESTRIAN_MULTIPLE_state2->PedPos.Y==PEDESTRIAN_MULTIPLE_state1->PedPos.Y)
			{
				return 0.06;
			}
		}
	}
	if(PEDESTRIAN_MULTIPLE_state1->Goal==3&&delta==PEDESTRIAN_MULTIPLE_state2->Vel)
	{
		if(PEDESTRIAN_MULTIPLE_state2->Goal==3&&PEDESTRIAN_MULTIPLE_state2->RobPos.Y==PEDESTRIAN_MULTIPLE_state1->RobPos.Y+delta)		//no uncertainty in robot motion
		{
			
			if(PEDESTRIAN_MULTIPLE_state2->PedPos.X==PEDESTRIAN_MULTIPLE_state1->PedPos.X&&PEDESTRIAN_MULTIPLE_state2->PedPos.Y==PEDESTRIAN_MULTIPLE_state1->PedPos.Y)
			{
				//ROS_INFO("enter");
				return 0.1;
			}
			if(PEDESTRIAN_MULTIPLE_state2->PedPos.X==PEDESTRIAN_MULTIPLE_state1->PedPos.X&&PEDESTRIAN_MULTIPLE_state2->PedPos.Y==PEDESTRIAN_MULTIPLE_state1->PedPos.Y+1)
			{
				if(PEDESTRIAN_MULTIPLE_state1->PedPos.X==4)	return 0.9;
				else	return 0.3;
			}
			if(PEDESTRIAN_MULTIPLE_state2->PedPos.X==PEDESTRIAN_MULTIPLE_state1->PedPos.X+1&&PEDESTRIAN_MULTIPLE_state2->PedPos.Y==PEDESTRIAN_MULTIPLE_state1->PedPos.Y)
			{
				if(PEDESTRIAN_MULTIPLE_state1->PedPos.Y==Size-1)	return 0.9;
				else	return 0.7;
			}
			
			/*noise case  go back*/
			if(PEDESTRIAN_MULTIPLE_state2->PedPos.X==PEDESTRIAN_MULTIPLE_state1->PedPos.X-1&&PEDESTRIAN_MULTIPLE_state2->PedPos.Y==PEDESTRIAN_MULTIPLE_state1->PedPos.Y)
			{
				return 0.06;
			}
		}
	}
	return 0.0;
}

double PEDESTRIAN_MULTIPLE::GetReward(int s)
{
	NToState(s,PEDESTRIAN_MULTIPLE_state1);
	
	if(PEDESTRIAN_MULTIPLE_state1->RobPos.Y==PEDESTRIAN_MULTIPLE_state1->PedPos.Y&&(2==PEDESTRIAN_MULTIPLE_state1->PedPos.X||1==PEDESTRIAN_MULTIPLE_state1->PedPos.X||3==PEDESTRIAN_MULTIPLE_state1->PedPos.X))
	{
		return CRUSH;
	}
	if(PEDESTRIAN_MULTIPLE_state1->RobPos.Y+1==PEDESTRIAN_MULTIPLE_state1->PedPos.Y&&(2==PEDESTRIAN_MULTIPLE_state1->PedPos.X||1==PEDESTRIAN_MULTIPLE_state1->PedPos.X||3==PEDESTRIAN_MULTIPLE_state1->PedPos.X)&&PEDESTRIAN_MULTIPLE_state1->Vel>0)
	{
		return CRUSH;
	}
	
	if(PEDESTRIAN_MULTIPLE_state1->RobPos.Y>=PEDESTRIAN_MULTIPLE_state1->PedPos.Y)
	{
		return 1000;
	}	

	return 0;
}

double PEDESTRIAN_MULTIPLE::MaxQ(int s) 
{
	int i;
	double m=-1000000;
	for(i=0;i<NumActions;i++)
	{
		if(qValue[s][i]>m)	m=qValue[s][i];
	}
	return m;
} 

void PEDESTRIAN_MULTIPLE::Init_QMDP()
{
	int i,j,k,ns1,ns2;
	//ROS_INFO("num of states %d",GetNumOfStates());
	
	for (i=1;i<20;i++)		//num of bellman-iterations
	{
		for(ns1=0;ns1<GetNumOfStates();ns1++)	//all numerations
		{
			for(j=0;j<NumActions;j++)
			{
				qValue[ns1][j]=0;
				double prob=0;
				for(ns2=0;ns2<GetNumOfStates();ns2++)
				{
					prob+=TransFn(ns1,j,ns2);
					qValue[ns1][j]+=TransFn(ns1,j,ns2)*Value[ns2];		
				}
					//ROS_INFO("%f",prob);
				if(prob!=0)
				{
					qValue[ns1][j]/=prob;
				}
				qValue[ns1][j]*=Discount;
			}
			Value[ns1]=MaxQ(ns1)+GetReward(ns1);
		}
		//ROS_INFO("initialization finished");
	}
	//ROS_INFO("initialization finished");
}


void PEDESTRIAN_MULTIPLE::writeQMDP()
{
	ofstream out("qmdp_values.txt");
	int i,j;
	for(i=0;i<GetNumOfStates();i++)
	{
		out<<Value[i]<<" ";
	}
	out<<endl;
	for(i=0;i<GetNumOfStates();i++)
	{
		for(j=0;j<3;j++)
		{
			out<<qValue[i][j]<<" ";
		}
	}
}

void PEDESTRIAN_MULTIPLE::loadQMDP()
{
	int i,j;
	ifstream in("qmdp_values.txt");
	
	for(i=0;i<GetNumOfStates();i++)
	{
		in>>Value[i];
	}
	for(i=0;i<GetNumOfStates();i++)
	{
		for(j=0;j<3;j++)
		{
			in>>qValue[i][j];
		}
	}
}

int PEDESTRIAN_MULTIPLE::QMDP_SelectAction(STATE*state)
{
	int best_a=0;
	double best_v=-10000;
	int n=StateToN(state);
	for(int i=0;i<3;i++)
	{
		if(qValue[n][i]>best_v)
		{
			best_v=qValue[n][i];
			best_a=i;
		}
	}
	return best_a;
}
double PEDESTRIAN_MULTIPLE::QMDP(STATE*state)
{
	//PEDESTRIAN_MULTIPLE_STATE* PEDESTRIAN_MULTIPLE_state=safe_cast<PEDESTRIAN_MULTIPLE_STATE*>(state);
	int n=StateToN(state);
	int j;
	/*for(j=0;j<NumActions;j++)
	{
		//ROS_INFO("action %d : %f",j,qValue[n][j]);
	}*/
	return MaxQ(n)+GetReward(n);
}
double PEDESTRIAN_MULTIPLE::QMDP(int state)
{
	//PEDESTRIAN_MULTIPLE_STATE* PEDESTRIAN_MULTIPLE_state=safe_cast<PEDESTRIAN_MULTIPLE_STATE*>(state);
	return MaxQ(state)+GetReward(state);
}

double PEDESTRIAN_MULTIPLE::Heuristic(STATE& state)
{
	PEDESTRIAN_MULTIPLE_STATE PEDESTRIAN_MULTIPLE_state;
        PEDESTRIAN_MULTIPLE_state=safe_cast<const PEDESTRIAN_MULTIPLE_STATE&>(state);
	return (1000-(PEDESTRIAN_MULTIPLE_state.PedPos.Y-PEDESTRIAN_MULTIPLE_state.RobPos.Y)*30);
}

void PEDESTRIAN_MULTIPLE::display_QMDP()
{
	//PEDESTRIAN_MULTIPLE_STATE* PEDESTRIAN_MULTIPLE_state=safe_cast<PEDESTRIAN_MULTIPLE_STATE*>(state);
	int i;
	PEDESTRIAN_MULTIPLE_STATE*PEDESTRIAN_MULTIPLE_state=MemoryPool.Allocate();
	double h_value;
	for(i=0;i<500;i++)
	{
		//ROS_INFO("value i %f",Value[i]);
		NToState(i,PEDESTRIAN_MULTIPLE_state);
		h_value=Heuristic(*PEDESTRIAN_MULTIPLE_state);
	//	if(PEDESTRIAN_MULTIPLE_state->Vel==0&&PEDESTRIAN_MULTIPLE_state->RobPos.Y==0&&PEDESTRIAN_MULTIPLE_state->PedPos.X==0&&PEDESTRIAN_MULTIPLE_state->PedPos.Y==4)
	//	{
			printf("goal %d vel %d ry %d px %d py %d value1 %f value2 %f value3 %f\n",PEDESTRIAN_MULTIPLE_state->Goal,PEDESTRIAN_MULTIPLE_state->Vel,PEDESTRIAN_MULTIPLE_state->RobPos.Y,PEDESTRIAN_MULTIPLE_state->PedPos.X,PEDESTRIAN_MULTIPLE_state->PedPos.Y,qValue[i][0],qValue[i][1],qValue[i][2]);
			//printf("goal %d vel %d ry %d px %d py %d value1 %f \n",PEDESTRIAN_MULTIPLE_state->Goal,PEDESTRIAN_MULTIPLE_state->Vel,PEDESTRIAN_MULTIPLE_state->RobPos.Y,PEDESTRIAN_MULTIPLE_state->PedPos.X,PEDESTRIAN_MULTIPLE_state->PedPos.Y,Value[i]);

	//	}
		
		/*if(Value[i]>1000)
		{
			NToState(i,PEDESTRIAN_MULTIPLE_state);
			ROS_INFO("goal %d ry %d px %d py %d value %f",PEDESTRIAN_MULTIPLE_state->Goal,PEDESTRIAN_MULTIPLE_state->RobPos.Y,PEDESTRIAN_MULTIPLE_state->PedPos.X,PEDESTRIAN_MULTIPLE_state->PedPos.Y,Value[i]);
		}*/
	}
}
