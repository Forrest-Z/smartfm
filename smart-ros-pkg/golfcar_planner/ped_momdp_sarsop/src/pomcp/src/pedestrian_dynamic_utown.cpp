#include "pedestrian_dynamic_utown.h"
#include "utils.h"
#include <string>
#include <math.h>
#include <fstream>
#include <iomanip>

using namespace std;
using namespace UTILS;



PEDESTRIAN_DYNAMIC_UTOWN_STATE*PEDESTRIAN_DYNAMIC_UTOWN_state1;
PEDESTRIAN_DYNAMIC_UTOWN_STATE*PEDESTRIAN_DYNAMIC_UTOWN_state2;
const int CRUSH=-5000;
PEDESTRIAN_DYNAMIC_UTOWN::PEDESTRIAN_DYNAMIC_UTOWN(int x_size=4,int y_size=9):X_SIZE(x_size),Y_SIZE(y_size)		
{
	//NumObservations=5*Size+1;
	Discount=0.95;
	NumObservations=MaxObs();  //Y_SIZE+5 is max length of rob_map
	RewardRange=-CRUSH;
	NumActions=3;
	//UpdateModel(0);
	//current_horizon=0;
	PEDESTRIAN_DYNAMIC_UTOWN_state1=MemoryPool.Allocate();
	PEDESTRIAN_DYNAMIC_UTOWN_state2=MemoryPool.Allocate();
	InitModel();
	//client_pt=&client;

}

STATE*PEDESTRIAN_DYNAMIC_UTOWN::Copy(const STATE &state) const
{
	const PEDESTRIAN_DYNAMIC_UTOWN_STATE& PEDESTRIAN_DYNAMIC_UTOWN_state=safe_cast<const PEDESTRIAN_DYNAMIC_UTOWN_STATE&>(state);
	PEDESTRIAN_DYNAMIC_UTOWN_STATE*newstate=MemoryPool.Allocate();
	*newstate=PEDESTRIAN_DYNAMIC_UTOWN_state;
	return newstate;
}

void PEDESTRIAN_DYNAMIC_UTOWN::Validate(const STATE& state) const
{
}

void PEDESTRIAN_DYNAMIC_UTOWN::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const PEDESTRIAN_DYNAMIC_UTOWN_STATE& PEDESTRIAN_DYNAMIC_UTOWN_state = safe_cast<const PEDESTRIAN_DYNAMIC_UTOWN_STATE&>(state);
    ostr<<"Rob Pos : "<<rob_map[PEDESTRIAN_DYNAMIC_UTOWN_state.RobPos.Y].first<<" "<<rob_map[PEDESTRIAN_DYNAMIC_UTOWN_state.RobPos.Y].second<<endl;
    ostr<<"Ped Pos : "<<PEDESTRIAN_DYNAMIC_UTOWN_state.PedPos.X<<" "<<PEDESTRIAN_DYNAMIC_UTOWN_state.PedPos.Y<<endl;
	ostr<<"Ped Vel : "<<PEDESTRIAN_DYNAMIC_UTOWN_state.Vel<<endl;
}



STATE*PEDESTRIAN_DYNAMIC_UTOWN::CreateStartState() const
{
	PEDESTRIAN_DYNAMIC_UTOWN_STATE*PEDESTRIAN_DYNAMIC_UTOWN_state=MemoryPool.Allocate();
	PEDESTRIAN_DYNAMIC_UTOWN_state->RobPos.X=1;
	PEDESTRIAN_DYNAMIC_UTOWN_state->RobPos.Y=0;

	PEDESTRIAN_DYNAMIC_UTOWN_state->Vel=1.0;


	int rand_pose=rand()%(X_SIZE*Y_SIZE);
	PEDESTRIAN_DYNAMIC_UTOWN_state->PedPos.X=rand_pose/Y_SIZE;
	PEDESTRIAN_DYNAMIC_UTOWN_state->PedPos.Y=rand_pose-PEDESTRIAN_DYNAMIC_UTOWN_state->PedPos.X*Y_SIZE;


//	PEDESTRIAN_DYNAMIC_UTOWN_state->PedPos.X=0;
//	PEDESTRIAN_DYNAMIC_UTOWN_state->PedPos.Y=rand()%10;

	int p=rand()%2;
	PEDESTRIAN_DYNAMIC_UTOWN_state->Goal=p;
	return PEDESTRIAN_DYNAMIC_UTOWN_state;
}

/*
STATE*PEDESTRIAN_DYNAMIC_UTOWN::CreateStartState() const
{
	PEDESTRIAN_DYNAMIC_UTOWN_STATE*PEDESTRIAN_DYNAMIC_UTOWN_state=MemoryPool.Allocate();
	PEDESTRIAN_DYNAMIC_UTOWN_state->RobPos.X=1;
	PEDESTRIAN_DYNAMIC_UTOWN_state->RobPos.Y=0;

	PEDESTRIAN_DYNAMIC_UTOWN_state->Vel=0.0;


	int rand_pose=rand()%40;
	//PEDESTRIAN_DYNAMIC_UTOWN_state->PedPos.X=rand_pose/10;
	//PEDESTRIAN_DYNAMIC_UTOWN_state->PedPos.Y=rand_pose-PEDESTRIAN_DYNAMIC_UTOWN_state->PedPos.X*10;
	PEDESTRIAN_DYNAMIC_UTOWN_state->PedPos.X=0;
	PEDESTRIAN_DYNAMIC_UTOWN_state->PedPos.Y=0;

	
	int p=rand()%4;
	PEDESTRIAN_DYNAMIC_UTOWN_state->Goal=p;
	PEDESTRIAN_DYNAMIC_UTOWN_state->Goal=3;
	return PEDESTRIAN_DYNAMIC_UTOWN_state;
}*/

void PEDESTRIAN_DYNAMIC_UTOWN::FreeState(STATE* state) const
{
    PEDESTRIAN_DYNAMIC_UTOWN_STATE* PEDESTRIAN_DYNAMIC_UTOWN_state = safe_cast<PEDESTRIAN_DYNAMIC_UTOWN_STATE*>(state);
    MemoryPool.Free(PEDESTRIAN_DYNAMIC_UTOWN_state);
}



void PEDESTRIAN_DYNAMIC_UTOWN::DisplayBeliefs(const BELIEF_STATE& beliefs, 
    std::ostream& ostr) const
{
    PEDESTRIAN_DYNAMIC_UTOWN_STATE PEDESTRIAN_DYNAMIC_UTOWN_state;
    int goal_count[4]={0};
    double ped_posx=0,ped_posy=0;
    double num=0;



	ostr<<"Number of Beliefs:"<<beliefs.GetNumSamples()<<endl;
    for (int i=0;i<beliefs.GetNumSamples();i++)
    {
    	const STATE* p_PEDESTRIAN_DYNAMIC_UTOWN_state=beliefs.GetSample(i);
    	PEDESTRIAN_DYNAMIC_UTOWN_state=safe_cast<const PEDESTRIAN_DYNAMIC_UTOWN_STATE&>(*p_PEDESTRIAN_DYNAMIC_UTOWN_state);
    	goal_count[PEDESTRIAN_DYNAMIC_UTOWN_state.Goal]++;
    	ped_posx+=PEDESTRIAN_DYNAMIC_UTOWN_state.PedPos.X;
    	ped_posy+=PEDESTRIAN_DYNAMIC_UTOWN_state.PedPos.Y;
    	num=num+1;
    }
    if(num!=0)
    {
			ostr<<"***********current belief***************************"<<endl;
			const PEDESTRIAN_DYNAMIC_UTOWN_STATE*ts;
			ts=safe_cast<const PEDESTRIAN_DYNAMIC_UTOWN_STATE*>(beliefs.GetSample(0));
			ostr<<"Rob Pos :"<<rob_map[ts->RobPos.Y].first<<" "<<rob_map[ts->RobPos.Y].second<<endl;
			ostr<<"Ped Pos :"<<ts->PedPos.X<<" "<<ts->PedPos.Y<<endl;
			ostr<<"Beliefs are "<<goal_count[0]/num<<" "<<goal_count[1]/num<<" "<<goal_count[2]/num<<" "<<goal_count[3]/num<<endl;
			ostr<<"****************************************************"<<endl;
			//ostr<<"Ped Pos X,Y "<<ped_posx/beliefs.GetNumSamples()<<ped_posy/beliefs.GetNumSamples()<<endl;
			//ostr<<"Ped Pos X,Y "<<ped_posx/beliefs.GetNumSamples()<<ped_posy/beliefs.GetNumSamples()<<endl;
    }
    else
    	    printf("Belief has no particle");
}

bool PEDESTRIAN_DYNAMIC_UTOWN::Same(const BELIEF_STATE& beliefState1,const BELIEF_STATE& beliefState2,double & distance)
{
	const PEDESTRIAN_DYNAMIC_UTOWN_STATE*PEDESTRIAN_DYNAMIC_UTOWN_state1;
	const PEDESTRIAN_DYNAMIC_UTOWN_STATE*PEDESTRIAN_DYNAMIC_UTOWN_state2;
	PEDESTRIAN_DYNAMIC_UTOWN_state1=safe_cast<const PEDESTRIAN_DYNAMIC_UTOWN_STATE*>(beliefState1.GetSample(0));
	PEDESTRIAN_DYNAMIC_UTOWN_state2=safe_cast<const PEDESTRIAN_DYNAMIC_UTOWN_STATE*>(beliefState2.GetSample(0));
	int dx1=PEDESTRIAN_DYNAMIC_UTOWN_state1->PedPos.X-PEDESTRIAN_DYNAMIC_UTOWN_state1->RobPos.X;
	int dx2=PEDESTRIAN_DYNAMIC_UTOWN_state2->PedPos.X-PEDESTRIAN_DYNAMIC_UTOWN_state2->RobPos.X;
	int dy1=PEDESTRIAN_DYNAMIC_UTOWN_state1->PedPos.Y-PEDESTRIAN_DYNAMIC_UTOWN_state1->RobPos.Y;
	int dy2=PEDESTRIAN_DYNAMIC_UTOWN_state2->PedPos.Y-PEDESTRIAN_DYNAMIC_UTOWN_state2->RobPos.Y;
	
	//if((dx1!=dx2)||(dy1!=dy2)) 	return false;
	if((PEDESTRIAN_DYNAMIC_UTOWN_state1->PedPos.X!=PEDESTRIAN_DYNAMIC_UTOWN_state2->PedPos.X)||(PEDESTRIAN_DYNAMIC_UTOWN_state1->PedPos.Y!=PEDESTRIAN_DYNAMIC_UTOWN_state2->PedPos.Y)) return false;
	if((PEDESTRIAN_DYNAMIC_UTOWN_state1->RobPos.X!=PEDESTRIAN_DYNAMIC_UTOWN_state2->RobPos.X)||(PEDESTRIAN_DYNAMIC_UTOWN_state1->RobPos.Y!=PEDESTRIAN_DYNAMIC_UTOWN_state2->RobPos.Y)) return false;
	if(PEDESTRIAN_DYNAMIC_UTOWN_state1->Vel!=PEDESTRIAN_DYNAMIC_UTOWN_state2->Vel) 	return false;

	int g1=0,g2=0;
	for(int i=0;i<beliefState1.GetNumSamples();i++)
	{
		PEDESTRIAN_DYNAMIC_UTOWN_state1=safe_cast<const PEDESTRIAN_DYNAMIC_UTOWN_STATE*>(beliefState1.GetSample(i));
		if(PEDESTRIAN_DYNAMIC_UTOWN_state1->Goal==1) g1++;
	}
	for(int i=0;i<beliefState2.GetNumSamples();i++)
	{
		PEDESTRIAN_DYNAMIC_UTOWN_state2=safe_cast<const PEDESTRIAN_DYNAMIC_UTOWN_STATE*>(beliefState2.GetSample(i));
		if(PEDESTRIAN_DYNAMIC_UTOWN_state2->Goal==1) g2++;
	}

	double p1=(g1+0.0)/beliefState1.GetNumSamples();
	double p2=(g2+0.0)/beliefState2.GetNumSamples();
	distance=fabs(p1-p2);
	if(distance>0.05) return false;
	return true;
}


VNODE* PEDESTRIAN_DYNAMIC_UTOWN::Find_old(VNODE*v1,const HISTORY &h)
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
VNODE* PEDESTRIAN_DYNAMIC_UTOWN::Find(VNODE*v1,const HISTORY &h)
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

VNODE* PEDESTRIAN_DYNAMIC_UTOWN::MapBelief(VNODE*vn,VNODE*r,const HISTORY &h,int historyDepth)
{

	//cout<<"Map Belief"<<endl;
	return vn;
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


void PEDESTRIAN_DYNAMIC_UTOWN::RobStep(STATE& state, int action) const 
{
	PEDESTRIAN_DYNAMIC_UTOWN_STATE& pedestrian_state=safe_cast<PEDESTRIAN_DYNAMIC_UTOWN_STATE&>(state);
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

int dir[8][2];
void PEDESTRIAN_DYNAMIC_UTOWN::PedStep(STATE& state) const
{
	PEDESTRIAN_DYNAMIC_UTOWN_STATE& pedestrian_state=safe_cast<PEDESTRIAN_DYNAMIC_UTOWN_STATE&>(state);
	int &pedX=pedestrian_state.PedPos.X;
	int &pedY=pedestrian_state.PedPos.Y;
	int goal=pedestrian_state.Goal;
	double p=(rand()+0.0)/RAND_MAX;
	double prob=0;
	bool inner=false;
	
	int tempx=pedX,tempy=pedY;
	for(int i=0;i<X_SIZE,inner==false;i++)
		for(int j=0;j<Y_SIZE;j++)
		{
			prob+=model[pedX][pedY][goal][i][j];
			if(p<prob)
			{
				//cout<<model[pedX][pedY][goal][i][j]<<endl;
				//cout<<pedX<<" "<<pedY<<endl;
				//cout<<i<<" "<<j<<endl;
				pedX=i;
				pedY=j;
				inner=true;
				break;
			}
		}
	
	/*
	if(tempx==0&&tempy==3)
	{
		cout<<p<<endl;
		cout<<prob<<endl;
		cout<<pedX<<" "<<pedY<<endl;
		cout<<model[tempx][tempy][0][pedX][pedY]<<endl;
		cout<<endl;
	}
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<10;j++)
		{
			cout<<setprecision(3)<<setw(5)<<model[0][3][0][i][j]<<" ";
		}
		cout<<endl;
	}*/
	
	/*
	for(int p=0;p<8;p++)
	{
		if(p<prob+model[pedX][pedY][goal][p])
		{
			pedX=pedX+dir[p][0];
			pedY=pedY+dir[p][1];
			break;
		}
	}*/
}

bool PEDESTRIAN_DYNAMIC_UTOWN::Step(STATE& state,int action, OBS_TYPE & observation,double&reward) const
{
	PEDESTRIAN_DYNAMIC_UTOWN_STATE& pedestrian_state=safe_cast<PEDESTRIAN_DYNAMIC_UTOWN_STATE&>(state);
	reward=0;
	observation=MaxObs()-1;
	int &pedX=pedestrian_state.PedPos.X;
	int &pedY=pedestrian_state.PedPos.Y;
	int pedX_old=pedX;
	int pedY_old=pedY;

	int &robX=pedestrian_state.RobPos.X;
	int &robY=pedestrian_state.RobPos.Y;
	int &rob_vel=pedestrian_state.Vel;

	if(robY>=rob_map.size()-1)
	{
		reward=1000;
		return true;
	}

	
	if(pedX==rob_map[robY].first&&rob_map[robY].second==pedY)
	{
		reward=CRUSH;
		return true;
	}
	
/*	
	if(pedX==rob_map[robY].first&&pedY==rob_map[robY].second+1)
	{
		if(rob_vel==0&&action==1)  {reward=CRUSH;return true;}
		if(rob_vel==1&&action<2)   {reward=CRUSH;return true;}
		if(rob_vel==2)     			{reward=CRUSH;return true;}
	}
	if(pedX==rob_map[robY].first&&pedY==rob_map[robY].second+2)
	{
		if(rob_vel==1&&action==1)  {reward=CRUSH;return true;}
		if(rob_vel==2&&action<2)  {reward=CRUSH;return true;}
	}*/



	RobStep(state,action);
	PedStep(state);
	reward=-50;
	observation=rob_vel*(X_SIZE*Y_SIZE*rob_map.size())+robY*(X_SIZE*Y_SIZE)+pedX*Y_SIZE+pedY;

	//observation=rob_vel*3000+map[robY][0]*500+map[robY][1]*41+pedX*11+pedY;
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




