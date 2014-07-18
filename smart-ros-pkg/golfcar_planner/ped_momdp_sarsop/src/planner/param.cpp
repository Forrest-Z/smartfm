#include"param.h"

namespace ModelParams {
    double CRASH_PENALTY = -10000;
    double REWARD_FACTOR_VEL = 1.0;
    double REWARD_BASE_CRASH_VEL=0.8;
    double BELIEF_SMOOTHING = 0.05;
    double NOISE_ROBVEL = 0.2;
    double NOISE_GOAL_ANGLE = 3.14 * 0.25;
    double COLLISION_DISTANCE = 1.0;
    double IN_FRONT_ANGLE_DEG = 90;

	double VEL_MAX=2;

	std::string rosns="/golfcart";
	std::string laser_frame="/front_bottom_lidar";
}

