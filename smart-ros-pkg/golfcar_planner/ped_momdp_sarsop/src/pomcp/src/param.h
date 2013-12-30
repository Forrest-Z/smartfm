
#ifndef MODELPARAMS_H
#define MODELPARAMS_H

namespace ModelParams {
	const double rln=20;
	const double path_rln=40;
	const double pi=3.1415926;
	const int UCT_CONST=50000;
	const int XSIZE=5;
	const int YSIZE=10;
	const int RMMax=20;   //maximum length of the rob_map structure
	const int N_PED_IN=7;

	
	/*!!!!!!!!!!Need to modify for different experiment map!!!!!!!*/
	//const int NGOAL=13;
	//const int GOAL_DIST=22;  /*the distance between ped and its goal*/
	
	const int NGOAL=6;
	const int GOAL_DIST=12;
	
	const int goodrob=1;
	const int goodped=1;
	const bool debug=false;
	const bool FixedPath=false;
	const bool SocialForceModel=true;
	const bool SocialForceWorld=false;
};

#endif
