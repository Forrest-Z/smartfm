#include "battleship.h"
#include "utils.h"
#include <string>
#include <math.h>
#include <fstream>

using namespace std;
using namespace UTILS;



TESTPROBLEM_STATE*teststate1;
TESTPROBLEM_STATE*teststate2;
const int CRUSH=-3000;
TESTPROBLEM::TESTPROBLEM(int size=9):Size(size)
{
	Discount=0.95;
	//NumObservations=5*Size+1;
	NumObservations=400;
	RewardRange=4000;
	NumActions=3;
	//UpdateModel(0);
	//current_horizon=0;
	teststate1=MemoryPool.Allocate();
	teststate2=MemoryPool.Allocate();
	InitModel();
	//client_pt=&client;

}

STATE*TESTPROBLEM::Copy(const STATE &state) const
{
	const TESTPROBLEM_STATE& teststate=safe_cast<const TESTPROBLEM_STATE&>(state);
	TESTPROBLEM_STATE*newstate=MemoryPool.Allocate();
	*newstate=teststate;
	return newstate;
}

void TESTPROBLEM::Validate(const STATE& state) const
{
}

void TESTPROBLEM::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const TESTPROBLEM_STATE& teststate = safe_cast<const TESTPROBLEM_STATE&>(state);
    ostr<<"Rob Pos : "<<teststate.RobPos.X<<" "<<teststate.RobPos.Y<<endl;
    ostr<<"Ped Pos : "<<teststate.PedPos.X<<" "<<teststate.PedPos.Y<<endl;
}
STATE*TESTPROBLEM::CreateStartState() const
{
	//ROS_INFO("fucking creating start state");
	TESTPROBLEM_STATE*teststate=MemoryPool.Allocate();
	teststate->RobPos.X=2;
	teststate->RobPos.Y=0;
	teststate->PedPos.X=0;
	teststate->PedPos.Y=4;
	teststate->Vel=0;
	int p=rand();
	//p=1;
	if(p%2==0)
	{
		teststate->Goal=1;
	}
	else
	{
		teststate->Goal=3;
	}
	//teststate->Goal=p%4;
	//teststate->Goal=3;
	//ROS_INFO("start goal is %d",teststate->Goal);
	//teststate->Goal=3;
	//cout<<"start goal="<<teststate->Goal<<endl;
	return teststate;
}

void TESTPROBLEM::FreeState(STATE* state) const
{
    TESTPROBLEM_STATE* teststate = safe_cast<TESTPROBLEM_STATE*>(state);
    MemoryPool.Free(teststate);
}


void TESTPROBLEM::InitModel()
{
	int i1,i2,j1,j2;
	for(i1=0;i1<5;i1++)
	{
		for(j1=0;j1<Size;j1++)
		{
			walk_dirs[i1][j1][0]=walk_dirs[i1][j1][1]=4;
		}
	}
	/*
	int b1s=100,b1t=-1,b4s=100,b4t=-1;
	
	//1,2,3,4 left right up free
	for(j1=0;j1<Size;j1++)
	{
		if(j1>=b1s&&j1<=b1t)
		{
			walk_dirs[0][j1][0]=walk_dirs[0][j1][1]=2;
			walk_dirs[1][j1][1]=3;
		}
		if(j1<=b1t)
		{
			walk_dirs[1][j1][0]=3;
		}
		if(j1>=b4s&&j1<=b4t)
		{
			walk_dirs[4][j1][1]=walk_dirs[4][j1][0]=1;
			walk_dirs[3][j1][0]=3;
		}
		if(j1<=b4t)
		{
			walk_dirs[3][j1][1]=3;
		}
	}
	char buf[20];
	string res="\n";c
	for(i1=0;i1<5;i1++)
	{
		for(j1=0;j1<Size;j1++)
		{
			sprintf(buf,"%d",walk_dirs[i1][j1][0]);
			res+=buf;
		}
		res+="\n";
	}
	ROS_INFO("%s",res.c_str());
	res="\n";
	for(i1=0;i1<5;i1++)
	{
		for(j1=0;j1<Size;j1++)
		{
			sprintf(buf,"%d",walk_dirs[i1][j1][1]);
			res+=buf;
		}
		res+="\n";
	}
	ROS_INFO("%s",res.c_str());*/
	//UpdateModel(0);
}
void TESTPROBLEM::UpdateModel(int s)
{
	/*
	ros::NodeHandle n;
	ros::ServiceClient  client=n.serviceClient<pomcp_map_server::short_path>("short_path");
	//ROS_INFO("Model updated!!!!!!!!!!!!!!!!!!!!!!!!!!!");
	pomcp_map_server::short_path srv;
	srv.request.start=s;
	client.call(srv);
	int count=0;
	int i1,j1,i2,j2;
	char buf[20];
	string res="\n";
	for(i1=0;i1<5;i1++)
	{
		for(j1=0;j1<Size;j1++)
		{
			for(i2=0;i2<5;i2++)
				for(j2=0;j2<Size;j2++)
				{
					trans[i1][j1][i2][j2]=srv.response.walk_dirs[count++];	
				}
			sprintf(buf,"%d",trans[i1][j1][0][Size-1]);
			res+=buf;
		}
		res+="\n";
	}
	ROS_INFO("%s",res.c_str());*/
}

void TESTPROBLEM::DisplayBeliefs(const BELIEF_STATE& beliefs, 
    std::ostream& ostr) const
{
    TESTPROBLEM_STATE teststate;
    int goal_count[4]={0};
    double ped_posx=0,ped_posy=0;
    double num=0;




    for (int i=0;i<beliefs.GetNumSamples();i++)
    {
    	const STATE* p_teststate=beliefs.GetSample(i);
    	teststate=safe_cast<const TESTPROBLEM_STATE&>(*p_teststate);
    	goal_count[teststate.Goal]++;
    	ped_posx+=teststate.PedPos.X;
    	ped_posy+=teststate.PedPos.Y;
    	num=num+1;
    }
    if(num!=0)
    {
			ostr<<"***********current belief***************************"<<endl;
			const TESTPROBLEM_STATE*ts;
			ts=safe_cast<const TESTPROBLEM_STATE*>(beliefs.GetSample(0));
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

bool TESTPROBLEM::Same(const BELIEF_STATE& beliefState1,const BELIEF_STATE& beliefState2,double & distance)
{
	const TESTPROBLEM_STATE*teststate1;
	const TESTPROBLEM_STATE*teststate2;
	teststate1=safe_cast<const TESTPROBLEM_STATE*>(beliefState1.GetSample(0));
	teststate2=safe_cast<const TESTPROBLEM_STATE*>(beliefState2.GetSample(0));
	int dx1=teststate1->PedPos.X-teststate1->RobPos.X;
	int dx2=teststate2->PedPos.X-teststate2->RobPos.X;
	int dy1=teststate1->PedPos.Y-teststate1->RobPos.Y;
	int dy2=teststate2->PedPos.Y-teststate2->RobPos.Y;
	
	//if((dx1!=dx2)||(dy1!=dy2)) 	return false;
	if((teststate1->PedPos.X!=teststate2->PedPos.X)||(teststate1->PedPos.Y!=teststate2->PedPos.Y)) return false;
	if((teststate1->RobPos.X!=teststate2->RobPos.X)||(teststate1->RobPos.Y!=teststate2->RobPos.Y)) return false;
	if(teststate1->Vel!=teststate2->Vel) 	return false;

	int g1=0,g2=0;
	for(int i=0;i<beliefState1.GetNumSamples();i++)
	{
		teststate1=safe_cast<const TESTPROBLEM_STATE*>(beliefState1.GetSample(i));
		if(teststate1->Goal==1) g1++;
	}
	for(int i=0;i<beliefState2.GetNumSamples();i++)
	{
		teststate2=safe_cast<const TESTPROBLEM_STATE*>(beliefState2.GetSample(i));
		if(teststate2->Goal==1) g2++;
	}

	double p1=(g1+0.0)/beliefState1.GetNumSamples();
	double p2=(g2+0.0)/beliefState2.GetNumSamples();
	distance=fabs(p1-p2);
	if(distance>0.05) return false;
	return true;
}

int foc=0;
VNODE* TESTPROBLEM::Find_old(VNODE*v1,const HISTORY &h)
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
		foc++;
		cout<<foc<<endl;
		return v1; 
	}
	else
	{
		return 0;
	}
}
VNODE* TESTPROBLEM::Find(VNODE*v1,const HISTORY &h)
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

VNODE* TESTPROBLEM::MapBelief(VNODE*vn,VNODE*r,const HISTORY &h,int historyDepth)
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


bool TESTPROBLEM::Step(STATE& state,int action, int & observation,double&reward) const
{
	TESTPROBLEM_STATE& teststate=safe_cast<TESTPROBLEM_STATE&>(state);
	reward=0;
	observation=150;
	
	if (action==ACT_ACC)	//accelerate
	{	
		if(teststate.Vel!=2)	teststate.Vel++;	//speed limit
	}
	else if(action==ACT_DEC)
	{
		if(teststate.Vel!=0)	teststate.Vel--;
		//if(teststate.Vel>0)	teststate.Vel=0;
	}

	//crush detect	
	if(abs(teststate.RobPos.X-teststate.PedPos.X)<=1&&teststate.RobPos.Y==teststate.PedPos.Y)	
	{
		reward=CRUSH;
		return true;
	}
	if(abs(teststate.RobPos.X-teststate.PedPos.X)<=1&&teststate.RobPos.Y==teststate.PedPos.Y-1&&teststate.Vel>0)
	{
		reward=CRUSH;
		return true;
	}
	
	/*
	int rel_ry,rel_py,rel_px;
	rel_ry=teststate.RobPos.Y-current_horizon;			//relative poses inside the current window
	rel_px=teststate.PedPos.X;
	rel_py=teststate.PedPos.Y-current_horizon;*/
	//pos transition
	if(teststate.RobPos.Y>=teststate.PedPos.Y)	//overtake the pedestrian,end
	{
		reward=1000;
		return true;
	}
	else if(teststate.RobPos.Y>=Size||teststate.PedPos.Y>=Size)
	{
		reward=0;
		return true;
	}
	else
	{
		teststate.RobPos.Y=teststate.RobPos.Y+teststate.Vel;
	}
	
	int &pedX=teststate.PedPos.X;
	int &pedY=teststate.PedPos.Y;
	
	
	//if((rel_py== Size-1 || rel_py==0)&&(pedX==0||pedX==4))		//pedestrain reach its goal
	/*if(rel_py==Size-1 && pedX==4)	//reach the goal
	{
		//no move
	}*/
	//double p=(rand()+0.0)/RAND_MAX;
	//double sum=1.03;
	//p=p*sum;
	if(Bernoulli(0.1)==true)	
	{
		//no move with prob 1/3
	}
	else
	{	
		int walk_dir;
		//double rate=5.0/(5+Size);
		//double rate=0.3;
		double rate=0.7;
		double noise=0.06;
		if(teststate.Goal==0)
		{
			walk_dir=trans[teststate.PedPos.X][teststate.PedPos.Y][0][0];
			if(walk_dir==3)		//two directions the same
			{
				if(Bernoulli(rate)==true && pedX>0)
				{
					pedX--;	
				}
				else
				{
					pedY--;
				}
			}
			else if(walk_dir==1&&pedX>0)	//left-right
			{
				pedX--;
			}			
			else	if(walk_dir==2&&pedY>0)		//up down
			{
				pedY--;
			}
			else			//exception
			{
				
			}
		}
		else if(teststate.Goal==1)
		{
			walk_dir=walk_dirs[teststate.PedPos.X][teststate.PedPos.Y][0];
			//if(pedY==7)
			//ROS_INFO("noise!!! %d %d %d",action,pedX,pedY);
			if(walk_dir==4)		//two directions the same
			{
				if(Bernoulli(rate)==true&&pedX>0)
				{
					pedX--;	
				}
				else if(Bernoulli(noise)==true&&pedX<4)
				{
					//ROS_INFO("noise!!!!!!!!!");
					pedX++;
				}
				else if(pedY<Size-1)
				{
					pedY++;
				}
			}
			else if(walk_dir==1&&pedX>0)	//left
			{
				pedX--;
			}			
			else	if(walk_dir==2&&pedX<4)		//right
			{
				pedX++;
			}
			else 	if(walk_dir==3&&pedY<Size-1)
			{
				pedY++;
			}
			else			//exception
			{
				
			}
		}
		else if(teststate.Goal==2)
		{
			walk_dir=trans[teststate.PedPos.X][teststate.PedPos.Y][4][0];
			if(walk_dir==3)		//two directions the same
			{
				if(Bernoulli(rate)==true)
				{
					pedX++;	
				}
				else
				{
					pedY--;
				}
			}
			else if(walk_dir==1)	//left-right
			{
				pedX++;
			}			
			else	if(walk_dir==2)		//up down
			{
				pedY--;
			}
			else			//exception
			{
				
			}
		}
		else
		{
			walk_dir=walk_dirs[teststate.PedPos.X][teststate.PedPos.Y][1];
			if(walk_dir==4)		//two directions the same
			{
				if(Bernoulli(rate)==true &&pedX<4)
				{
					pedX++;	
				}
				else if(Bernoulli(noise)==true&&pedX>0)
				{
					pedX--;
				}
				else if(pedY<Size-1)
				{
					pedY++;
				}
			}
			else if(walk_dir==1&&pedX>0)	//left
			{
				pedX--;
			}		
	
			else	if(walk_dir==2&&pedX<4)		//right
			{
				pedX++;
			}
			else 	if(walk_dir==3&&pedY<Size-1)		//up
			{
				pedY++;
			}
			else			//exception
			{
				
			}
		}
	}
	
	double obs_noise=0.1;
	double p=(rand()+0.0)/RAND_MAX;
	observation=teststate.PedPos.X*30+teststate.PedPos.Y;

	/*	
	if(p<0.1)
	{
		if(teststate.PedPos.X+1<4)
			observation=(teststate.PedPos.X+1)*30+teststate.PedPos.Y;
	}
	else if(p<0.2)
	{
		if(teststate.PedPos.X-1>0)
			observation=(teststate.PedPos.X-1)*30+teststate.PedPos.Y;
	}
	else if(p<0.3)
	{
		if(teststate.PedPos.Y+1<Size-1)
			observation=teststate.PedPos.X*30+teststate.PedPos.Y+1;
	}
	else if(p<0.4)
	{
		if(teststate.PedPos.Y-1>0)
			observation=teststate.PedPos.X*30+teststate.PedPos.Y-1;
	}
	else
	{
		observation=teststate.PedPos.X*30+teststate.PedPos.Y;
	}*/
	reward=0;

	return false;
}

int TESTPROBLEM::GetNumOfStates()
{
	return 5*Size*Size*3*2;	
}
int TESTPROBLEM::StateToN(STATE* state)
{
	TESTPROBLEM_STATE* teststate=safe_cast<TESTPROBLEM_STATE*>(state);	
	int add_goal=(5*Size*Size*3)*(teststate->Goal==3);
	int add_vel=(5*Size*Size)*teststate->Vel;
	int add_Y1=(5*Size)*teststate->RobPos.Y;
	int add_Y2=5*teststate->PedPos.Y;
	int add_X=teststate->PedPos.X;
	int N=add_X+add_Y2+add_Y1+add_vel+add_goal;
	return N;
	//teststate.RobPos.Y
}

void TESTPROBLEM::NToState(int n,TESTPROBLEM_STATE*teststate)
{
	//5 lanes
	int N=n;
	//TESTPROBLEM_STATE*teststate=MemoryPool.Allocate();
	teststate->Goal=(N/(5*Size*Size*3)==0?1:3);
	N=N%(5*Size*Size*3);
	teststate->Vel=N/(5*Size*Size);
	N=N%(5*Size*Size);
	teststate->RobPos.Y=N/(5*Size);
	N=N%(5*Size);
	teststate->PedPos.Y=N/5;
	N=N%5;
	teststate->PedPos.X=N;
	//return teststate;
}


double TESTPROBLEM::TransFn(int s1,int a,int s2)
{
	
	NToState(s1,teststate1);
	NToState(s2,teststate2);
	//return 0.0;
	
	//3 terminating states
	
	
	/*strict condition*/
	
	if(teststate1->RobPos.Y==teststate1->PedPos.Y&&(2==teststate1->PedPos.X||1==teststate1->PedPos.X||3==teststate1->PedPos.X))
	{
		return 0.0;
	}
	
	if(teststate1->RobPos.Y+1==teststate1->PedPos.Y&&(2==teststate1->PedPos.X||1==teststate1->PedPos.X||3==teststate1->PedPos.X)&&teststate1->Vel>0)
	{
		return 0.0;
	}
	
	/*loose constraint*/
	/*
	if(teststate1->RobPos.Y==teststate1->PedPos.Y&&(2==teststate1->PedPos.X))
	{
		return 0.0;
	}
	
	if(teststate1->RobPos.Y+1==teststate1->PedPos.Y&&(2==teststate1->PedPos.X)&&teststate1->Vel>0)
	{
		return 0.0;
	}*/
	
	
	
	if(teststate1->RobPos.Y>=teststate1->PedPos.Y)	//overtake,finish
	{
		//ROS_INFO("enter");
		//ROS_INFO("goal %d ry %d px %d py %d",teststate1->Goal,teststate1->RobPos.Y,teststate1->PedPos.X,teststate1->PedPos.Y);
		return 0.0;
	}
	
	if(teststate1->RobPos.Y>=Size-1||teststate1->PedPos.Y>=Size-1)
	{
		return 0.0;
	}
	
	int delta;
	int action=a;
	if (action==ACT_ACC)
	{
		delta=teststate1->Vel+1;
		if(delta>2)	delta=2;
	}
	else if (action==ACT_DEC)
	{
		delta=teststate1->Vel-1;
		if(delta<0)	delta=0;
	}
	else
	{
		delta=teststate1->Vel;
	}
	
	if(teststate1->Goal==1&&delta==teststate2->Vel)
	{
		if(teststate2->Goal==1&&teststate2->RobPos.Y==teststate1->RobPos.Y+delta)		//no uncertainty in robot motion
		{	
			if(teststate2->PedPos.X==teststate1->PedPos.X&&teststate2->PedPos.Y==teststate1->PedPos.Y)
			{
				//ROS_INFO("enter");
				return 0.1;
			}
			if(teststate2->PedPos.X==teststate1->PedPos.X&&teststate2->PedPos.Y==teststate1->PedPos.Y+1)
			{
				if(teststate1->PedPos.X==0)	return 0.9;
				else	return 0.3;
			}
			if(teststate2->PedPos.X==teststate1->PedPos.X-1&&teststate2->PedPos.Y==teststate1->PedPos.Y)
			{
				if(teststate1->PedPos.Y==Size-1)	return 0.9;
				else	return 0.7;
			}
			
			/*noise case  go back*/
			if(teststate2->PedPos.X==teststate1->PedPos.X+1&&teststate2->PedPos.Y==teststate1->PedPos.Y)
			{
				return 0.06;
			}
		}
	}
	if(teststate1->Goal==3&&delta==teststate2->Vel)
	{
		if(teststate2->Goal==3&&teststate2->RobPos.Y==teststate1->RobPos.Y+delta)		//no uncertainty in robot motion
		{
			
			if(teststate2->PedPos.X==teststate1->PedPos.X&&teststate2->PedPos.Y==teststate1->PedPos.Y)
			{
				//ROS_INFO("enter");
				return 0.1;
			}
			if(teststate2->PedPos.X==teststate1->PedPos.X&&teststate2->PedPos.Y==teststate1->PedPos.Y+1)
			{
				if(teststate1->PedPos.X==4)	return 0.9;
				else	return 0.3;
			}
			if(teststate2->PedPos.X==teststate1->PedPos.X+1&&teststate2->PedPos.Y==teststate1->PedPos.Y)
			{
				if(teststate1->PedPos.Y==Size-1)	return 0.9;
				else	return 0.7;
			}
			
			/*noise case  go back*/
			if(teststate2->PedPos.X==teststate1->PedPos.X-1&&teststate2->PedPos.Y==teststate1->PedPos.Y)
			{
				return 0.06;
			}
		}
	}
	return 0.0;
}

double TESTPROBLEM::GetReward(int s)
{
	NToState(s,teststate1);
	
	if(teststate1->RobPos.Y==teststate1->PedPos.Y&&(2==teststate1->PedPos.X||1==teststate1->PedPos.X||3==teststate1->PedPos.X))
	{
		return CRUSH;
	}
	if(teststate1->RobPos.Y+1==teststate1->PedPos.Y&&(2==teststate1->PedPos.X||1==teststate1->PedPos.X||3==teststate1->PedPos.X)&&teststate1->Vel>0)
	{
		return CRUSH;
	}
	
	if(teststate1->RobPos.Y>=teststate1->PedPos.Y)
	{
		return 1000;
	}	

	return 0;
}

double TESTPROBLEM::MaxQ(int s) 
{
	int i;
	double m=-1000000;
	for(i=0;i<NumActions;i++)
	{
		if(qValue[s][i]>m)	m=qValue[s][i];
	}
	return m;
} 

void TESTPROBLEM::Init_QMDP()
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


void TESTPROBLEM::writeQMDP()
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

void TESTPROBLEM::loadQMDP()
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

int TESTPROBLEM::QMDP_SelectAction(STATE*state)
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
double TESTPROBLEM::QMDP(STATE*state)
{
	//TESTPROBLEM_STATE* teststate=safe_cast<TESTPROBLEM_STATE*>(state);
	int n=StateToN(state);
	int j;
	/*for(j=0;j<NumActions;j++)
	{
		//ROS_INFO("action %d : %f",j,qValue[n][j]);
	}*/
	return MaxQ(n)+GetReward(n);
}
double TESTPROBLEM::QMDP(int state)
{
	//TESTPROBLEM_STATE* teststate=safe_cast<TESTPROBLEM_STATE*>(state);
	return MaxQ(state)+GetReward(state);
}

double TESTPROBLEM::Heuristic(STATE& state)
{
	TESTPROBLEM_STATE teststate;
        teststate=safe_cast<const TESTPROBLEM_STATE&>(state);
	return (1000-(teststate.PedPos.Y-teststate.RobPos.Y)*30);
}

void TESTPROBLEM::display_QMDP()
{
	//TESTPROBLEM_STATE* teststate=safe_cast<TESTPROBLEM_STATE*>(state);
	int i;
	TESTPROBLEM_STATE*teststate=MemoryPool.Allocate();
	double h_value;
	for(i=0;i<500;i++)
	{
		//ROS_INFO("value i %f",Value[i]);
		NToState(i,teststate);
		h_value=Heuristic(*teststate);
	//	if(teststate->Vel==0&&teststate->RobPos.Y==0&&teststate->PedPos.X==0&&teststate->PedPos.Y==4)
	//	{
			printf("goal %d vel %d ry %d px %d py %d value1 %f value2 %f value3 %f\n",teststate->Goal,teststate->Vel,teststate->RobPos.Y,teststate->PedPos.X,teststate->PedPos.Y,qValue[i][0],qValue[i][1],qValue[i][2]);
			//printf("goal %d vel %d ry %d px %d py %d value1 %f \n",teststate->Goal,teststate->Vel,teststate->RobPos.Y,teststate->PedPos.X,teststate->PedPos.Y,Value[i]);

	//	}
		
		/*if(Value[i]>1000)
		{
			NToState(i,teststate);
			ROS_INFO("goal %d ry %d px %d py %d value %f",teststate->Goal,teststate->RobPos.Y,teststate->PedPos.X,teststate->PedPos.Y,Value[i]);
		}*/
	}
}
