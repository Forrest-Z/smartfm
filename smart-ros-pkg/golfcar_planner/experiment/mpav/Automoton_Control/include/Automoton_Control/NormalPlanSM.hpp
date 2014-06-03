/*
 * NormalPlan.hpp
 *
 *  Created on: Nov 18, 2013
 *      Author: liuwlz
 */

#ifndef NORMALPLANSM_HPP_
#define NORMALPLANSM_HPP_

#include <Automoton_Control/ObstAvoidSM.hpp>

namespace MPAV{

	struct NormalPlan: sc::simple_state<NormalPlan,Moving>{
	public:
		typedef mpl::list<sc::custom_reaction<Ev_Obst_Detected>, sc::custom_reaction<Ev_Reach_Dest> > reactions;
		NormalPlan(){

		}
		virtual ~NormalPlan(){

		}
		sc::result react(const Ev_Obst_Detected&);
		sc::result react(const Ev_Reach_Dest&);
	};

	sc::result NormalPlan::react(const Ev_Obst_Detected&){
		return transit<RRTSPlan>();
	}

	sc::result NormalPlan::react(const Ev_Reach_Dest&){
		return forward_event();
	}
}
#endif /* NORMALPLANSM_HPP_ */
