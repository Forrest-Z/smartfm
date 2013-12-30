#include<iostream>
#include<fstream>
#include "executer.h"


Executer::Executer()
{
	std::cout<<"New Executer added!"<<std::endl;
	sim=new PEDESTRIAN_DYNAMIC_REAL(9);
	searchParams.ExplorationConstant = sim->GetRewardRange();
	MCTS::InitFastUCB(searchParams.ExplorationConstant);	
	searchParams.NumStartStates=50000; 
	searchParams.NumSimulations=30000; 
	mcts=new MCTS(*sim, searchParams);
	PEDESTRIAN_DYNAMIC_REAL*pedproblem_d=new PEDESTRIAN_DYNAMIC_REAL(9);
	mcts->pedproblem_d=pedproblem_d;
	iter=0;
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
	iter++;

}
/*
bool Executer::OneStep(STATE*state,int action,int&obs,double &rwd)
{	
	return sim->Step(state,action,obs,rwd); 
}*/

int  Executer::GetAction()
{
	return mcts->SelectAction();
}

void Executer::Init()
{
	
}

void Executer::Display()
{
	char buf[20];
	sprintf(buf,"final tree%d.txt",iter);
	std::ofstream out(buf);
	mcts->DisplayValue(20,out);
}

void Executer::Reset()
{
	delete mcts;
	mcts=new MCTS(*sim, searchParams);
	PEDESTRIAN_DYNAMIC_REAL*pedproblem_d=new PEDESTRIAN_DYNAMIC_REAL(9);
	mcts->pedproblem_d=pedproblem_d;
	iter=0;
}
/*
using namespace std;
int seq[20][2]={{0,0},{0,1},{0,2},{0,3},{0,3},{0,4},{0,5},{0,5},{0,5},{0,5},{0,5},{0,5},{0,6},{0,6},{0,7},{0,7},{0,7},{0,7},{0,8},{0,8}};  //px,py

int rob[20]={0,0,0,1,1,1,2,2,2,3,3,3,4,4,4,5,5,5,6,6};

int main()
{
	cout<<"executer main function!"<<endl;
	Executer exec;
	int rob_vel,robY,pedX,pedY;
	
	robY=0;	
	rob_vel=1;
	for(int i=0;i<20;i++)
	{
		int act=exec.GetAction();
		
		char buf[20];
		sprintf(buf,"final tree%d.txt",i);
		ofstream out(buf);
		//out<<"hello world";
		exec.mcts->DisplayValue(20,out);
		out.close();
		cout<<"Tree generated!"<<endl;
		
		
		
		//robY=robY+rob_vel;
		robY=rob[i];
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
}*/
