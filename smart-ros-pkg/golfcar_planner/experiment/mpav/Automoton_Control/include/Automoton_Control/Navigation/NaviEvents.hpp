/*
 * NaviEvents.hpp
 *
 *  Created on: Nov 29, 2013
 *      Author: liuwlz
 */

#ifndef NAVIEVENTS_HPP_
#define NAVIEVENTS_HPP_

#include <ros/ros.h>
#include <ros/console.h>
#include <boost/statechart/event.hpp>

namespace sc = boost::statechart;

namespace MPAV{

	/**
	 *	Navi level events
	 */
	struct Ev_VehicleReady:sc::event<Ev_VehicleReady>{

	};

	struct Ev_ObstAvoid:sc::event<Ev_ObstAvoid>{

	};

	struct Ev_TJunc:sc::event<Ev_TJunc>{

	};

	struct Ev_Roundabout:sc::event<Ev_Roundabout>{

	};

	struct Ev_Parking:sc::event<Ev_Parking>{

	};

	struct Ev_ReachDest:sc::event<Ev_ReachDest>{

	};

	/**
	 * ObstAvoid level events
	 */
	struct Ev_ReplanPathFound:sc::event<Ev_ReplanPathFound>{

	};

	struct Ev_ReplanPathDanger:sc::event<Ev_ReplanPathDanger>{

	};

	struct Ev_ReachSubGoal:sc::event<Ev_ReachSubGoal>{

	};

	/**
	 * T-Junc level events
	 */
}



#endif /* NAVIEVENTS_HPP_ */
