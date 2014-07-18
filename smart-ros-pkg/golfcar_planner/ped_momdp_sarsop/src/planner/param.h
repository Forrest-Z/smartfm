
#ifndef MODELPARAMS_H
#define MODELPARAMS_H
#include<string>


namespace ModelParams {

	const double GOAL_TRAVELLED=5.0;
	const int N_PED_IN=4;

	extern double VEL_MAX;
    extern double NOISE_GOAL_ANGLE;
    extern double CRASH_PENALTY;
    extern double REWARD_FACTOR_VEL;
    extern double REWARD_BASE_CRASH_VEL;
    extern double BELIEF_SMOOTHING;
    extern double NOISE_ROBVEL;
    extern double COLLISION_DISTANCE;

    extern double IN_FRONT_ANGLE_DEG;

	const double pos_rln=0.5; // position resolution
	const double vel_rln=0.03; // velocity resolution

    const double PATH_STEP = 0.05;

    const double GOAL_TOLERANCE = 2;

    const double PED_SPEED = 1.2;

	const bool debug=false;

	const double control_freq=3;
	const double AccSpeed=0.5;

	extern std::string rosns;
	extern std::string laser_frame;

    inline void init_params(bool in_simulation) {
        if(in_simulation) {
            rosns="";
            laser_frame="/base_laser_link";
        } else {
            rosns="/golfcart";
            laser_frame="/front_bottom_lidar";
        }
    }

    // deprecated params
	const double GOAL_REWARD = 0.0;
};

#endif

