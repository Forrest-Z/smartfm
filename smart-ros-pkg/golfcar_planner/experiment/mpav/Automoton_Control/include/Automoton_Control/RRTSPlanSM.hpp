/*
 * RRTSPlan.cpp
 *
 *  Created on: Nov 18, 2013
 *      Author: liuwlz
 */

#ifndef RRTSPLANSM_HPP_
#define RRTSPLANSM_HPP_

#include <Automoton_Control/ObstAvoidSM.hpp>

namespace sc = boost::statechart;

namespace MPAV{
	struct RRTSPlan:sc::simple_state<RRTSPlan, Moving>{
	private:
	public:
		typedef mpl::list<sc::custom_reaction<Ev_Reach_Temp_Goal>, sc::custom_reaction<Ev_Reach_Dest> > reactions;
		RRTSPlan();
		virtual ~RRTSPlan();
		sc::result react(const Ev_Reach_Temp_Goal &);
		sc::result react(const Ev_Reach_Dest &);
	};

	RRTSPlan::RRTSPlan(){
		RRTStar->planner_timer.start();
	}

	RRTSPlan::~RRTSPlan(){
		RRTStar->planner_timer.stop();
	}

	sc::result RRTSPlan::react(const Ev_Reach_Temp_Goal &){
		return transit<NormalPlan>();
	}

	sc::result RRTSPlan::react(const Ev_Reach_Dest &){
		return forward_event();
	}

}

#endif /* RRTSPLANSM_CPP_ */
