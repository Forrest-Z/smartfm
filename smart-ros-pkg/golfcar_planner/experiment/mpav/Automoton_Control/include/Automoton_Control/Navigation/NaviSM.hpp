/*
 * NaviSM.hpp
 *
 *  Created on: Nov 29, 2013
 *      Author: liuwlz
 */

#ifndef NAVISM_HPP_
#define NAVISM_HPP_

#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/event.hpp>
#include <boost/mpl/list.hpp>

#include <Automoton_Control/Navigation/NaviEvents.hpp>

#include <rrts_node_exp.h>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

namespace MPAV{

	/**Main state machine*/
	struct NaviSM;
	/**
	 * Forward definition for states and state machine
	 */
	struct	UTurnSM;
	struct IdleCheckSM;
	struct RoundaboutSM;
	struct	TJunctionSM;
	struct ObstAvoidSM;
	struct	ParkingSM;
	struct MovingSM;
	struct NormPlanSM;

	struct NaviSM:sc::state_machine<NaviSM, IdleCheckSM>{
		/**
		 * Definition of some common shared objects, like RRTStar, etc.
		 */
	};
}

#endif /* NAVISM_HPP_ */
