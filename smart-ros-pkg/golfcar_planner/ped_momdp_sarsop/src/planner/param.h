
#ifndef MODELPARAMS_H
#define MODELPARAMS_H
#include<string>


namespace ModelParams {
	const double pos_rln=0.5;
	const double vel_rln=0.3;
	const int N_PED_IN=15;
	const int GOAL_TRAVELLED=10;

    const double CRASH_PENALTY = -10000;
	const double GOAL_REWARD = 500;

    const double PATH_STEP = 0.05;

    const double GOAL_TOLERANCE = 3;

	const bool debug=false;

    const double NOISE_GOAL_ANGLE = 3.14 * 0.6;
    const double PED_SPEED = 1.2;

	const double VEL_MAX=2;
	//const double VEL_MAX=1;
	const double control_freq=3;
	const double AccSpeed=1.0;

	static std::string rosns="/golfcart";
	static std::string laser_frame="/front_bottom_lidar";

	//const std::string rosns="";

	//const std::string laser_frame="/base_link";
	//const std::string laser_frame="/base_laser_link";

    inline void init_params(bool in_simulation) {
        if(in_simulation) {
            rosns="";
            laser_frame="/base_laser_link";
        } else {
            rosns="/golfcart";
            laser_frame="/front_bottom_lidar";
        }
    }
};

#endif

