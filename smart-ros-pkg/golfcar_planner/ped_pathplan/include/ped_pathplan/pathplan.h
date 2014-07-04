#ifndef _PATHPLAN_H
#define _PATHPLAN_H

#include <cmath>
#include <string>
#include <vector>

// cost defs
#define COST_UNKNOWN_ROS 255		// 255 is unknown cost
#define COST_OBS 254		// 254 for forbidden regions
#define COST_OBS_ROS 253	// ROS values of 253 are obstacles
#define COST_NEUTRAL 50		// Set this to "open space" value
#define COST_FACTOR 2		// Used for translating costs in NavFn::setCostmap()
//const float COST_STEERING = (10 / M_PI * 180 );
const float COST_STEERING = (100 / M_PI * 180 );

#define COSTTYPE unsigned char	// Whatever is used...

namespace ped_pathplan {
    //const float STEERING_LIMIT = M_PI / 180 * 15;
    const float STEERING_LIMIT = M_PI / 180 * 5;
    const int N_YAWS = 1+72*2;
    const float TOLERANCE = 10;
    const float D_YAW = M_PI * 2 / N_YAWS;

    typedef std::vector<float> State;
    typedef std::vector<int> DiscreteState;

    struct PathItem {
        State state;
        float g, h;
        int index;
        int prev_index;
    };

    typedef std::pair<float, int> QPair;


    class PathPlan {
        public:
            PathPlan(int nx, int ny, float steering_limit_deg, float yaw_res_deg);	// size of map


            float resolution;
            float step;
            int nx, ny, ns;		/**< size of grid, in pixels */
            COSTTYPE *costarr;		/**< cost array in 2D configuration space */
            std::vector<float> steerings;
            State start, goal;

            void setCostmap(const COSTTYPE *cmap, bool isROS=true, bool allow_unknown = true); /**< sets up the cost map */

            void setGoal(const State& goal);
            void setStart(const State& start);
            std::vector<State> calcPath();

        protected:
            PathItem next(const PathItem& p, float t, bool& success);
            float heuristic(const State& s);
            DiscreteState discretize(const State& s);
            float distToGoal(const State& s);
            bool isGoalReached(const State& s);
    };

}

#endif
