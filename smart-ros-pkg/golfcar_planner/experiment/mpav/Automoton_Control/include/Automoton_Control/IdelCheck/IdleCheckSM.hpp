/*
 * IdleCheckSM.hpp
 *
 *  Created on: Nov 29, 2013
 *      Author: liuwlz
 */

#ifndef IDLECHECKSM_HPP_
#define IDLECHECKSM_HPP_

#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>

#include <Automoton_Control/Navigation/NaviSM.hpp>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

namespace MPAV{

	struct IdelCheckSM : sc::state<IdelCheckSM,NaviSM>{
		typedef mpl::list<
				sc::custom_reaction<Ev_VehicleReady>
				> reactions;
		typedef sc::state<IdelCheckSM, NaviSM> Base;
		IdelCheckSM(my_context ctx) : Base(ctx){

		}
		virtual ~IdelCheckSM(){

		}

		sc::result react(const Ev_VehicleReady &){
			return transit<MovingSM>();
		}
	};
}


#endif /* IDLECHECKSM_HPP_ */
