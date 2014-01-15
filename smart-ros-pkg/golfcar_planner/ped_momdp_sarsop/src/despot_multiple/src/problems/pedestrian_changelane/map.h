#ifndef MYMAP_H
#define MYMAP_H
#include "math_utils.h"
#include <vector>
#include <fstream>
#include <iostream>
#include "param.h"


class MyMap {
public:


	int width;
	int height;
	bool full_map[1000][1000];
	bool map_obs[1000][1000];
	int global_plan[10000][2];   //maximum 10000 points
	int goal_pos[ModelParams::NGOAL][2];
	int pathLength;

	MyMap()
	{
		//InitGoalsBlank();
		//InitGoalsRound();
		InitGoalsUTown();
	}
	void InitGoalsUTown()
	{
		//LoadMap("utown_map2");
		LoadPath("real_path");
/*
		goal_pos[3][0]=131*ModelParams::map_rln;
		goal_pos[3][1]=136*ModelParams::map_rln;
		goal_pos[2][0]=128*ModelParams::map_rln;
		goal_pos[2][1]=118*ModelParams::map_rln;
		goal_pos[0][0]=100*ModelParams::map_rln;
		goal_pos[0][1]=136*ModelParams::map_rln;
		goal_pos[1][0]=100*ModelParams::map_rln;
		goal_pos[1][1]=112*ModelParams::map_rln;

*/


		
		goal_pos[3][0]=93*ModelParams::map_rln;
		goal_pos[3][1]=177*ModelParams::map_rln;
		goal_pos[2][0]=96*ModelParams::map_rln;
		goal_pos[2][1]=195*ModelParams::map_rln;
		goal_pos[0][0]=107*ModelParams::map_rln;
		goal_pos[0][1]=166*ModelParams::map_rln;
		goal_pos[1][0]=116*ModelParams::map_rln;
		goal_pos[1][1]=183*ModelParams::map_rln;
			
		
	/*	
		goal_pos[3][0]=106*ModelParams::map_rln;
		goal_pos[3][1]=117*ModelParams::map_rln;
		goal_pos[2][0]=110*ModelParams::map_rln;
		goal_pos[2][1]=134*ModelParams::map_rln;
		goal_pos[0][0]=126*ModelParams::map_rln;
		goal_pos[0][1]=112*ModelParams::map_rln;
		goal_pos[1][0]=129*ModelParams::map_rln;
		goal_pos[1][1]=128*ModelParams::map_rln;
	*/	

		/*
		goal_pos[0][0]=40*ModelParams::rln;  //jing wu zhu guan
		goal_pos[0][1]=21*ModelParams::rln;
		goal_pos[1][0]=36*ModelParams::rln;  //workshop
		goal_pos[1][1]=1.5*ModelParams::rln;
		goal_pos[2][0]=32*ModelParams::rln;  //bus entrance up
		goal_pos[2][1]=26*ModelParams::rln;
		goal_pos[3][0]=27*ModelParams::rln;  //bus entrance down
		goal_pos[3][1]=3*ModelParams::rln;
		goal_pos[4][0]=54*ModelParams::rln;  //jing wu zhu guan entrance 2
		goal_pos[4][1]=27*ModelParams::rln;
		goal_pos[5][0]=41*ModelParams::rln;  //to the bridge
		goal_pos[5][1]=39*ModelParams::rln;
		goal_pos[6][0]=61*ModelParams::rln;  //Create
		goal_pos[6][1]=56*ModelParams::rln;
		goal_pos[7][0]=67*ModelParams::rln;  //enterprise
		goal_pos[7][1]=39*ModelParams::rln;
		goal_pos[8][0]=80*ModelParams::rln;  //KouFu
		goal_pos[8][1]=68*ModelParams::rln;
		goal_pos[9][0]=82*ModelParams::rln; //TO the residence
		goal_pos[9][1]=52*ModelParams::rln;
		goal_pos[10][0]=90*ModelParams::rln; //Steve Riady Center
		goal_pos[10][1]=63*ModelParams::rln;
		goal_pos[11][0]=47*ModelParams::rln;//Wendy's
		goal_pos[11][1]=52*ModelParams::rln;
		goal_pos[12][0]=74*ModelParams::rln;
		goal_pos[12][1]=70*ModelParams::rln;*/
		/*
		goal_pos[1][0]=122;     //create
		goal_pos[1][1]=height-385;
		goal_pos[2][0]=734;    //workshop
		goal_pos[2][1]=height-1519;
		goal_pos[3][0]=780;    //restaurant
		goal_pos[3][1]=height-1150;
		goal_pos[4][0]=885;//Wendy's  
		goal_pos[4][1]=height-371;  
		goal_pos[5][0]=1633;  //KouFu
		goal_pos[5][1]=height-152;
		goal_pos[6][0]=610;   //bus stop
		goal_pos[6][1]=height-997;	
		goal_pos[7][0]=594;  //to the bridge
		goal_pos[7][1]=height-606;  
		goal_pos[8][0]=1282;  //enterprise
		goal_pos[8][1]=height-910;

		goal_pos[9][0]=1366;  //
		goal_pos[9][1]=height-768;
		goal_pos[10][0]=
		goal_pos[10][1]=
		*/
		/*
		goal_pos[9][0]=1646;  //italiano
		goal_pos[9][1]=height-762;
		goal_pos[10][0]=
		goal_pos[10][1]=*/

	}
	void InitGoalsRound()
	{
		
		goal_pos[0][0]=11*ModelParams::rln;
		goal_pos[0][1]=21*ModelParams::rln;
		goal_pos[1][0]=0.5*ModelParams::rln;
		goal_pos[1][1]=20*ModelParams::rln;
		goal_pos[2][0]=0.5*ModelParams::rln;
		goal_pos[2][1]=12*ModelParams::rln;
		goal_pos[3][0]=0.5*ModelParams::rln;
		goal_pos[3][1]=3*ModelParams::rln;
		goal_pos[4][0]=12*ModelParams::rln;
		goal_pos[4][1]=0.5*ModelParams::rln;

		LoadMap("round_map");
		LoadPath("round_path");
	}
	void InitGoalsBlank()
	{
		/*	
		goal_pos[0][0]=0*ModelParams::rln;
		goal_pos[0][1]=1*ModelParams::rln;
		goal_pos[1][0]=8*ModelParams::rln;
		goal_pos[1][1]=5*ModelParams::rln;
		goal_pos[2][0]=8*ModelParams::rln;
		goal_pos[2][1]=15*ModelParams::rln;
		goal_pos[3][0]=4*ModelParams::rln;
		goal_pos[3][1]=15*ModelParams::rln;
		goal_pos[4][0]=0*ModelParams::rln;
		goal_pos[4][1]=15*ModelParams::rln;
		goal_pos[5][0]=0*ModelParams::rln;
		goal_pos[5][1]=5*ModelParams::rln;
		goal_pos[6][0]=8*ModelParams::rln;
		goal_pos[6][1]=1*ModelParams::rln;
		goal_pos[7][0]=8*ModelParams::rln;
		goal_pos[7][1]=10*ModelParams::rln;
		goal_pos[8][0]=0*ModelParams::rln;
		goal_pos[8][1]=10*ModelParams::rln;*/
		/*	
		goal_pos[0][0]=1*ModelParams::rln;
		goal_pos[0][1]=1*ModelParams::rln;
		goal_pos[1][0]=7*ModelParams::rln;
		goal_pos[1][1]=1*ModelParams::rln;
		goal_pos[2][0]=7*ModelParams::rln;
		goal_pos[2][1]=10*ModelParams::rln;
		goal_pos[3][0]=1*ModelParams::rln;
		goal_pos[3][1]=10*ModelParams::rln;	*/
	
		/*	
		goal_pos[0][0]=1*ModelParams::rln;
		goal_pos[0][1]=5*ModelParams::rln;
		
		goal_pos[1][0]=7*ModelParams::rln;
		goal_pos[1][1]=5*ModelParams::rln;
		
		goal_pos[2][0]=4*ModelParams::rln;
		goal_pos[2][1]=10*ModelParams::rln;*/

		/*	
		goal_pos[0][0]=4*ModelParams::rln;
		goal_pos[0][1]=10*ModelParams::rln;*/
		
		int delta=ModelParams::rln*7;
		int ct=0;
		int x1=ModelParams::rln,x2=ModelParams::rln*10;
		/*	
		for(int y=ModelParams::rln;y<463&&ct<ModelParams::NGOAL;y+=delta)
		{
			goal_pos[ct][0]=x1;
			goal_pos[ct][1]=y;
			ct++;
			
			std::cout<<goal_pos[ct-1][0]<<" "<<goal_pos[ct-1][1]<<std::endl;
			
			goal_pos[ct][0]=x2;
			goal_pos[ct][1]=y;
			ct++;
			std::cout<<goal_pos[ct-1][0]<<" "<<goal_pos[ct-1][1]<<std::endl;
			
		}*/
		goal_pos[0][0]=10;
		goal_pos[0][1]=100;
		goal_pos[1][0]=150;
		goal_pos[1][1]=100;
		goal_pos[2][0]=10;
		goal_pos[2][1]=200;
		goal_pos[3][0]=150;
		goal_pos[3][1]=200;
		goal_pos[4][0]=10;
		goal_pos[4][1]=300;
		goal_pos[5][0]=150;
		goal_pos[5][1]=300;
		
		//LoadMap("blank_map");
		LoadPath("blank_path");
	}
	void LoadMap(const char*);
	void LoadPath(const char*);
	bool Free(int w,int h)
	{
		return true;
		return (map_obs[w][h]==false); 

	}
	void PreComp(int w,int h)
	{
		double delta=ModelParams::rln;
		int sum=0,obs_sum=0;
		for(int i=w-delta/2;i<w+delta/2;i++)
			for(int j=h-delta/2;j<h+delta/2;j++)
			{
				if(i>0&&i<width&&j>0&&j<height)
				{
					sum++;
					if(full_map[i][j]==1) obs_sum++;
				}
			}
		if(obs_sum>sum*0.3) map_obs[w][h]=true;  //more than half grids are filled
		else 	map_obs[w][h]=false;
	}

	bool InMap(int w,int h)
	{
	     if(w>0&&w<width&&h>0&&h<height&&Free(w,h)) return true; 
		 else return false;
	}

};



#endif

