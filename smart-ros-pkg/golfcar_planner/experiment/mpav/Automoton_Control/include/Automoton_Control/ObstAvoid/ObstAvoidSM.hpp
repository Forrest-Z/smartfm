/*
 * ObstAvoidSM.hpp
 *
 *  Created on: Nov 29, 2013
 *      Author: liuwlz
 */

#ifndef OBSTAVOIDSM_HPP_
#define OBSTAVOIDSM_HPP_

#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>

#include <Automoton_Control/Navigation/NaviSM.hpp>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

namespace MPAV{
	struct HeadGoal;
	struct TempStop;

	struct ObstAvoidSM:sc::state<ObstAvoidSM, MovingSM, TempStop>{
		typedef mpl::list<
				sc::custom_reaction<Ev_ReachSubGoal>,
				sc::custom_reaction<Ev_ReachDest>
				> reactions;
		typedef sc::state<ObstAvoidSM, MovingSM, TempStop> Base;
		ObstAvoidSM(my_context ctx):Base(ctx){

		}

		virtual ~ObstAvoidSM(){

		}

		sc::result react(const Ev_ReachDest){
			return forward_event();
		}

		sc::result react(const Ev_ReachSubGoal){
			return	transit<NormPlanSM>();
		}
	};

	struct TempStop:sc::state<TempStop, ObstAvoidSM>{
		typedef mpl::list<
				sc::custom_reaction<Ev_ReplanPathFound>,
				sc::custom_reaction<Ev_ObstAvoid>
				> reactions;

		typedef sc::state<TempStop, ObstAvoidSM> Base;

		TempStop(my_context ctx):Base(ctx){

		}
		virtual ~TempStop(){

		}

		sc::result react(const Ev_ReplanPathFound&){
			return transit<HeadGoal>();
		}
	};

	struct HeadGoal:sc::state<HeadGoal, ObstAvoidSM>{
		typedef mpl::list<
				sc::custom_reaction <Ev_ReplanPathDanger>,
				sc::custom_reaction <Ev_ReachSubGoal>
			> reactions;

		typedef sc::state<HeadGoal, ObstAvoidSM> Base;
		HeadGoal(my_context ctx):Base(ctx){

		}
		virtual ~HeadGoal(){

		}

		sc::result react(const Ev_ReachSubGoal &){
			return forward_event();
		}

		sc::result react(const Ev_ReplanPathDanger &){
			return transit<TempStop>();
		}
	};
}

#endif /* OBSTAVOIDSM_HPP_ */
