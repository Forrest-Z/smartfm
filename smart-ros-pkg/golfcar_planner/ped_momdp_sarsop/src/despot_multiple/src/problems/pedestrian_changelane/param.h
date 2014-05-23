
#ifndef MODELPARAMS_H
#define MODELPARAMS_H


namespace ModelParams {
	const double rln=10;
	const double ped_rln=10;
	const double path_rln=10;
	const double map_rln=10;
	const double pi=3.1415926;
	const int UCT_CONST=50000;
	const int XSIZE=7;
	const int YSIZE=15;
	const int RMMax=30;   //maximum length of the rob_map structure
	const int N_PED_IN=10;

	//const int NGOAL=13;
	//const int GOAL_DIST=22;  //the distance between ped and its goal
	
	//const int NGOAL=5+1;  //plus one stop intention
	const int NGOAL=5;


	const int GOAL_DIST=100;
	
	const int goodrob=0;
	const int goodped=1;
	const bool debug=false;
	const bool FixedPath=false;
	const bool SocialForceModel=false;
	const bool SocialForceWorld=false;

	const double vel_levels [10] = { 0.0, 1.0, 2.0 };
	//const int  VEL_N=7;
	//const double VEL_MAX=2.0;
	
	const int  VEL_N=3;
	const double VEL_MAX=2;
	const double control_freq=1;
	const double AccSpeed=1.0;
	const char rosns[100]="";
	const char laser_frame[100]="/base_laser_link";
	const int NPATH=2;
};

/*
namespace ModelParams {
	const double rln=5;
	const double ped_rln=5;
	const double path_rln=10;
	const double map_rln=10;
	const double pi=3.1415926;
	const int UCT_CONST=50000;
	const int XSIZE=10;
	const int YSIZE=20;
	const int RMMax=26;   //maximum length of the rob_map structure
	const int N_PED_IN=10;

	
	//const int NGOAL=13;
	//const int GOAL_DIST=22;  //the distance between ped and its goal
	
	//const int NGOAL=5+1;  //plus one stop intention
	const int NGOAL=6;


	const int GOAL_DIST=100;
	
	const int goodrob=0;
	const int goodped=1;
	const bool debug=false;
	const bool FixedPath=false;
	const bool SocialForceModel=false;
	const bool SocialForceWorld=false;

	const double vel_levels [10] = { 0.0, 1.0, 2.0 };
	//const int  VEL_N=7;
	//const double VEL_MAX=2.0;
	
	const int  VEL_N=3;
	const double VEL_MAX=2;
	const double control_freq=1.0;
	const double AccSpeed=1.0;
};*/

#endif
