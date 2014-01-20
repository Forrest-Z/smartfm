/*
 * NormPlan.hpp
 *
 *  Created on: Nov 29, 2013
 *      Author: liuwlz
 */

#ifndef NORMPLAN_HPP_
#define NORMPLAN_HPP_

#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>

#include <Automoton_Control/Navigation/NaviSM.hpp>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

namespace MPAV{

	struct NormPlanSM : sc::state<NormPlanSM,NaviSM>{
		typedef mpl::list<
				sc::custom_reaction<Ev_ObstAvoid>,
				sc::custom_reaction<Ev_ReachDest>,
				sc::custom_reaction<Ev_Parking>,
				sc::custom_reaction<Ev_TJunc>,
				sc::custom_reaction<Ev_Roundabout>
				> reactions;
		typedef sc::state<NormPlanSM, NaviSM> Base;
		NormPlanSM(my_context ctx) : Base(ctx){

		}

		virtual ~NormPlanSM(){

		}

		sc::result react(const Ev_ObstAvoid &){
			return transit<ObstAvoidSM>();
		}

		sc::result react(const Ev_ReachDest& ){
			return forward_event();
		}

		sc::result react(const Ev_Parking &){
			return transit<ParkingSM>();
		}
	};
}


#endif /* NORMPLAN_HPP_ */
