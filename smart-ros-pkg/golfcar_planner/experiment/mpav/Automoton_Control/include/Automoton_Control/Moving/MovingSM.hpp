/*
 * MovingSM.hpp
 *
 *  Created on: Nov 29, 2013
 *      Author: liuwlz
 */

#ifndef MOVINGSM_HPP_
#define MOVINGSM_HPP_

#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>

#include <Automoton_Control/Navigation/NaviSM.hpp>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

namespace MPAV{

	struct MovingSM : sc::state<MovingSM, NaviSM, NormPlanSM>{
		typedef mpl::list<
				sc::custom_reaction<Ev_ReachDest>
				> reactions;
		typedef sc::state<MovingSM, NaviSM, NormPlanSM> Base;
		MovingSM(my_context ctx) : Base(ctx){

		}

		virtual ~MovingSM(){

		}

		sc::result react(const Ev_ReachDest &){
			return transit<IdleCheckSM>();
		}
	};
}

#endif /* MOVINGSM_HPP_ */
