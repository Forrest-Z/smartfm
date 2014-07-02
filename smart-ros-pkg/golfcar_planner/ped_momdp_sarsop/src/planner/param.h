
#ifndef MODELPARAMS_H
#define MODELPARAMS_H
#include<string>


namespace ModelParams {
	const double pos_rln=0.5;
	const double vel_rln=0.3;
	const double pi=3.1415926;
	const int UCT_CONST=50000;
	const int XSIZE=7;
	const int YSIZE=15;
	const int RMMax=30;   //maximum length of the rob_map structure
	const int N_PED_IN=15;
	const int GOAL_TRAVELLED=10;

    const double CRASH_PENALTY = -10000;
	const double GOAL_REWARD = 500;

    //TODO dynamically calculate this
    const double PATH_STEP = 0.05;

	//const int NGOAL=13;
	//const int GOAL_DIST=22;  //the distance between ped and its goal
	
	//const int NGOAL=5+1;  //plus one stop intention
	const int NGOAL=5;


	const int GOAL_DIST=100;
    const double GOAL_TOLERANCE = 3;
	
	const int goodrob=0;
	const int goodped=1;
	const bool debug=false;
	const bool FixedPath=false;
	const bool SocialForceModel=false;
	const bool SocialForceWorld=false;

    const double NOISE_GOAL_ANGLE = 3.14 * 0.6;
    const double PED_SPEED = 1.2;

	const double vel_levels [10] = { 0.0, 1.0, 2.0 };
	//const int  VEL_N=7;
	//const double VEL_MAX=2.0;
	
	const int  VEL_N=3;
	const double VEL_MAX=2;
	//const double VEL_MAX=1;
	const double control_freq=3;
	const double AccSpeed=1.0;
//  const char rosns[100]="/golfcart";
//	const char laser_frame[100]="/base_laser_link";

//	const char laser_frame[100]="/front_bottom_lidar";

	const std::string rosns="/golfcart";
	const std::string laser_frame="/front_bottom_lidar";
	
	//const std::string rosns="";
	
	//const std::string laser_frame="/base_link";
	//const std::string laser_frame="/base_laser_link";
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
