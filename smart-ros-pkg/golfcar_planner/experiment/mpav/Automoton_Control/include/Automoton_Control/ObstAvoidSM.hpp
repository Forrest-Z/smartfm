/*
 * PlannerShiftAuto.hpp
 *
 *  Created on: Nov 16, 2013
 *      Author: liuwlz
 */

#ifndef PLANNERSHIFTAUTO_HPP_
#define PLANNERSHIFTAUTO_HPP_

#include <ros/ros.h>

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>

#include <rrts_node_exp.h>
#include <Goal_Generator/ReplanGoal.hpp>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

namespace MPAV{
	/**
	 * Define planner and goal object as global variables
	 */
	PlannerExp* RRTStar;
	ReplanGoal* Subgoal;

	/**
	 * Pre-definitions of states and state machine
	 */
	struct ObstAvoidSM;

	struct	Idle;
	struct Moving;

	struct EmergeStop;
	struct RRTSPlan;
	struct EmerStop;
	struct RePlan;
	struct NormalPlan;

	/**
	 * Events
	 */
	struct Ev_PrePath_Got:sc::event<Ev_PrePath_Got>{
		Ev_PrePath_Got(){ROS_INFO("Reference Path Got");}
	};

	struct Ev_Obst_Detected :sc::event<Ev_Obst_Detected>{
		Ev_Obst_Detected(){ROS_INFO("Obstacle Detected");}
	};

	struct	Ev_Reach_Temp_Goal:sc::event<Ev_Reach_Temp_Goal>{
		Ev_Reach_Temp_Goal(){ROS_INFO("Reach Temporal Goal");}
	};

	struct Ev_Reach_Dest:sc::event<Ev_Reach_Dest>{
		Ev_Reach_Dest(){ROS_INFO("Reach Destination");}
	};

	struct Ev_RRTSPath_Found:sc::event<Ev_RRTSPath_Found>{
		Ev_RRTSPath_Found(){ROS_INFO("RRTS Path Found");}
	};

	/**
	 * State Machine
	 */
	struct	ObstAvoidSM:sc::state_machine<ObstAvoidSM, Idle>{};

	/**
	 * States & Transitions
	 */
	struct Idle:sc::simple_state<Idle, ObstAvoidSM>{
	public:
		typedef sc::custom_reaction<Ev_PrePath_Got> reactions;
		Idle(){}
		virtual ~Idle(){}

		sc::result react (const Ev_PrePath_Got &){
			return transit<Moving>();
		}
	};

	struct Moving: sc::simple_state<Moving,ObstAvoidSM,NormalPlan>{
	public:
		typedef sc::custom_reaction<Ev_Reach_Dest> reactions;
		Moving(){
		}
		virtual ~Moving(){
		}
		sc::result react(const Ev_Reach_Dest &){
			return transit<Idle>();
		}
	};
}

#endif /* PLANNERSHIFTAUTO_HPP_ */
