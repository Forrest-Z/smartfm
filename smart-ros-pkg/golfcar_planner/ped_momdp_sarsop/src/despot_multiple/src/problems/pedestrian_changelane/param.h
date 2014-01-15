
#ifndef MODELPARAMS_H
#define MODELPARAMS_H

namespace ModelParams {
	const double rln=10;
	const double path_rln=10;
	const double map_rln=10;
	const double pi=3.1415926;
	const int UCT_CONST=50000;
	const int XSIZE=7;
	const int YSIZE=15;
	const int RMMax=20;   //maximum length of the rob_map structure
	const int N_PED_IN=7;

	
	/*!!!!!!!!!!Need to modify for different experiment map!!!!!!!*/
	//const int NGOAL=13;
	//const int GOAL_DIST=22;  /*the distance between ped and its goal*/
	
	const int NGOAL=4;
	const int GOAL_DIST=100;
	
	const int goodrob=0;
	const int goodped=1;
	const bool debug=true;
	const bool FixedPath=false;
	const bool SocialForceModel=false;
	const bool SocialForceWorld=false;

	const double vel_levels [10] = { 0.0, 1.0, 2.0 };
	const int  VEL_N=3;
};

#endif
