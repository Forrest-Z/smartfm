#include<iostream>
#include "executer.h"


Executer::Executer()
{
	sim=new PEDESTRIAN_DYNAMIC(10);
	MCTS::PARAMS searchParams;
	searchParams.NumStartStates=50000; 
	searchParams.NumSimulations=10000; 
	searchParams.UseTime=true;
	mcts=new MCTS(*sim, searchParams);
	PEDESTRIAN_DYNAMIC*pedproblem_d=new PEDESTRIAN_DYNAMIC(10);
	mcts->pedproblem_d=pedproblem_d;
}

void Executer::Step(int action,int observation)
{
	bool outOfParticles;
	int reward;
	outOfParticles = !mcts->Update(action, observation, reward);
	if(outOfParticles)
	{
		std::cout<<"outOfParticles!!Stop!!!"<<std::endl;
	}

}

int  Executer::GetAction()
{
	return mcts->SelectAction();
}

void Executer::Init()
{
	
}


using namespace std;
int seq[20][2]={{0,0},{0,1},{0,2},{0,3},{0,3},{0,4},{0,5},{0,5},{0,5},{0,5},{0,5},{0,5},{0,6},{0,6},{0,7},{0,7},{0,7},{0,7},{0,8},{0,8}};  //px,py



int main()
{
	Executer exec;
	int rob_vel,robY,pedX,pedY;
	
	robY=0;	
	rob_vel=1;
	for(int i=0;i<20;i++)
	{
		int act=exec.GetAction();
		robY=robY+rob_vel;
		if(act==1)  rob_vel++;
		else if(act==2) rob_vel--;
		
		if(rob_vel<0) rob_vel=0;
		if(rob_vel>2) rob_vel=2;
		 

		pedX=seq[i][0];
		pedY=seq[i][1];
		
		int observation=rob_vel*500+robY*41+pedX*11+pedY;
		
		cout<<pedX<<" "<<pedY<<endl;
		cout<<"act:"<<act<<endl;
		cout<<"robY:"<<robY<<endl;
		cout<<"vel:"<<rob_vel<<endl;
		exec.Step(act,observation);
	}
	return 0;
}
