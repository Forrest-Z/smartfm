/*
 * rrtstar.h
 *
 *  Created on: Jan 24, 2013
 *      Author: liuwlz
 */

#ifndef RRTSTAR_H_
#define RRTSTAR_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/contrib/rrt_star/RRTstar.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/geometric/SimpleSetup.h>
#include <fstream>
#include <stdlib.h>

#include <rrts_ompl/rrts_edge.h>

using namespace ompl;
using namespace std;

//Define commonly used planning parameter
typedef base::ScopedState<base::DubinsStateSpace> state_;
typedef base::StateSpacePtr space_;
typedef base::SpaceInformationPtr spaceinfo_;
typedef base::ProblemDefinitionPtr prodefine_;
typedef base::PlannerPtr planner_;
typedef base::RealVectorBounds bound_;
typedef base::ValidStateSamplerPtr sampler_;
typedef geometric::SimpleSetup simplesetup_;
typedef geometric::PathGeometric path_;
typedef base::PlannerData planner_date_;



//Class rrts: Initialise planner and generate the path
class rrts {

	simplesetup_ *simplesetup;
	spaceinfo_ *spaceinfo;
	state_ *goal;
	state_ *root;
	space_ *space;
	planner_date_ *plan_data;

public:
	rrts();
	~rrts();

	bool first_root;
	bool first_goal;

	//Params for valid sampler.
	int sample_width; int sample_height;
	vector<int> sample_free_cell;

	int initplanner(int width, int height);
	int setBound(int width, int height);
	int setRoot(double _root[]);
	int setGoal(double _goal[]);
	int setValidChecker();
	int planning(vector<double> &path_reals);
	int visualize(vector<rrts_ompl::rrts_edge> &rrts_tree);

	bool sampler_type;
};

//TODO: Finish valid sampler: Figure out the constructor of heritagence problem.
class configuredStateSampler : public base::ValidStateSampler, public rrts
{
public:
	configuredStateSampler(const base::SpaceInformation *si) : ValidStateSampler(si){
        name_ = "valid sampler";
    }

    // Generate a sample in the valid part of the SE2 state space considering the free cell information given by local_map_ext node.
    virtual bool sample(base::State *state){
    	double x = static_cast<base::DubinsStateSpace::StateType*>(state)->getX();
    	double y = static_cast<base::DubinsStateSpace::StateType*>(state)->getY();

    	x = rng_.uniformInt(0, sample_width);
    	y = rng_.uniformInt(0, sample_height);

        assert(si_->isValid(state));

        int map_index = y * sample_width + x;

        if (sample_free_cell[map_index] > 0)
        	return true;
        else
        	return false;
    }

    virtual bool sampleNear(base::State *state, const base::State *near, const double distance){
    	throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
    	return false;
	}
protected:
    ompl::RNG rng_;
};

#endif /* RRTSTAR_H_ */
