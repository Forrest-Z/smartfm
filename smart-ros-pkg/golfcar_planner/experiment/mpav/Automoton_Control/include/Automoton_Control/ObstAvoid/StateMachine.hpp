/*
 * ObstAvoidSM.hpp
 *
 *  Created on: Nov 16, 2013
 *      Author: liuwlz
 */

#ifndef PLANNERSHIFTAUTO_HPP_
#define PLANNERSHIFTAUTO_HPP_

#include <ros/ros.h>

#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/mpl/list.hpp>

#include <rrts_node_exp.h>
#include <Goal_Generator/ReplanGoal.hpp>
#include <Speed_Control/SMSpeedControl.hpp>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

namespace MPAV{
	/**
	 * Pre-definitions of states and state machine
	 */
	struct ObstAvoidSM;

	/**
	 * State Intersection: Handling the intersection
	 */
	struct Intersection;

	/**
	 * State Idel: Waiting for GlobalPath
	 */
	struct	Idle;

	/**
	 * State Moving: Heading to destination
	 */
	struct Moving;
	/**
	 * State RRTSPlan: Replanning to find path
	 */
	struct RRTSPlan;
	/**
	 *State NormPlan: Following Pre-defined path
	 */
	struct NormalPlan;
	/**
	 * State EmerStop: Emergency Stop due to obstacle or Replan path unsafe
	 */
	struct EmerStop;

	/**
	 * State HeadSubGoal: Heading to the subgoal
	 */
	struct HeadSubGoal;
	/**
	 * State Machine: Initialise some objects for states implementation
	 */
	struct	ObstAvoidSM:sc::state_machine<ObstAvoidSM, Idle>{
		PlannerExp *RRTStar;
		ReplanGoal *SubGoal;
		SMSpeedControl *SpeedControl;
		pnc_msgs::move_status move_status;
		bool SMStatus[6];
	};

	/**
	 * Events
	 */
	struct Ev_PrePath_Got:sc::event<Ev_PrePath_Got>{
		Ev_PrePath_Got(){ROS_INFO("Reference Path Got");}
	};

	struct Ev_Obst_Detected :sc::event<Ev_Obst_Detected>{
		Ev_Obst_Detected(){}
	};

	struct	Ev_Reach_Temp_Goal:sc::event<Ev_Reach_Temp_Goal>{
		Ev_Reach_Temp_Goal(){}
	};

	struct Ev_Reach_Dest:sc::event<Ev_Reach_Dest>{
		Ev_Reach_Dest(){ROS_INFO("Reach Destination");}
	};

	struct Ev_RRTSPath_Found:sc::event<Ev_RRTSPath_Found>{
		Ev_RRTSPath_Found(){}
	};

	struct Ev_RRTSPath_Unsafe:sc::event<Ev_RRTSPath_Unsafe>{
		Ev_RRTSPath_Unsafe(){}
	};

	struct Ev_Enter_Intersection:sc::event<Ev_Enter_Intersection>{
		Ev_Enter_Intersection(){ROS_INFO("Enter Intersection");}
	};

	struct Ev_Intersection_Safe:sc::event<Ev_Intersection_Safe>{
		Ev_Intersection_Safe(){ROS_INFO("Intersection: Safe to Go");}
	};

	/**
	 * States Explanation & Transitions
	 */
	struct Idle:sc::simple_state<Idle, ObstAvoidSM>{
	public:
		typedef sc::custom_reaction<Ev_PrePath_Got> reactions;
		Idle(){
			ROS_INFO("Entering Idle State");
		}
		virtual ~Idle(){
			ROS_INFO("Leaving Idle State");
		}

		sc::result react (const Ev_PrePath_Got &){
			return transit<Moving>();
		}
	};

	struct Moving: sc::simple_state<Moving,ObstAvoidSM,NormalPlan>{
	public:
		typedef sc::custom_reaction<Ev_Reach_Dest> reactions;
		Moving(){
			ROS_INFO("Entering Moving State");
		}
		virtual ~Moving(){
			ROS_INFO("Leaving Moving State");
		}
		sc::result react(const Ev_Reach_Dest &){
			return transit<Idle>();
		}
	};

	struct NormalPlan: sc::state<NormalPlan,Moving>{
	public:
		typedef mpl::list<sc::custom_reaction<Ev_Obst_Detected>,
				sc::custom_reaction<Ev_Reach_Dest>,
				sc::custom_reaction<Ev_RRTSPath_Unsafe>,
				sc::custom_reaction<Ev_PrePath_Got>,
				sc::custom_reaction<Ev_Enter_Intersection> > reactions;
		typedef sc::state<NormalPlan, Moving> Base;
		NormalPlan(my_context ctx):Base(ctx){
			ROS_INFO("Entering NormPlan State");
			context<ObstAvoidSM>().RRTStar->planner_timer.stop();
			context<ObstAvoidSM>().move_status.emergency = false;
			context<ObstAvoidSM>().move_status.path_exist = true;
			context<ObstAvoidSM>().SpeedControl->SetStatus(0);
		}

		virtual ~NormalPlan(){
			ROS_INFO("Leaving NormPlan State");
		}

		sc::result react(const Ev_Obst_Detected&){
			return transit<RRTSPlan>();
		}

		sc::result react(const Ev_Reach_Dest&){
			return forward_event();
		}

		sc::result react(const Ev_RRTSPath_Unsafe&){
			return discard_event();
		}

		sc::result react(const Ev_PrePath_Got&){
			return discard_event();
		}

		sc::result react(const Ev_Enter_Intersection&){
			return transit<Intersection>();
		}
	};

	struct Intersection: sc::state<Intersection,Moving>{
	public:
		typedef mpl::list<
				sc::custom_reaction<Ev_Reach_Dest>,
				sc::custom_reaction<Ev_RRTSPath_Unsafe>,
				sc::custom_reaction<Ev_PrePath_Got>,
				sc::custom_reaction<Ev_Intersection_Safe> > reactions;
		typedef sc::state<Intersection, Moving> Base;
		Intersection(my_context ctx):Base(ctx){
			ROS_INFO("Entering Intersection State");
			context<ObstAvoidSM>().RRTStar->planner_timer.stop();
			context<ObstAvoidSM>().move_status.emergency = false;
			context<ObstAvoidSM>().move_status.path_exist = true;
			context<ObstAvoidSM>().SpeedControl->SetStatus(3);
		}

		virtual ~Intersection(){
			ROS_INFO("Safe to Go. Leaving Intersection State");
		}

		sc::result react(const Ev_Reach_Dest&){
			return forward_event();
		}

		sc::result react(const Ev_RRTSPath_Unsafe&){
			return discard_event();
		}

		sc::result react(const Ev_PrePath_Got&){
			return discard_event();
		}

		sc::result react(const Ev_Intersection_Safe&){
			return transit<NormalPlan>();
		}
	};

	struct RRTSPlan:sc::state<RRTSPlan, Moving, EmerStop>{
	public:
		typedef mpl::list<sc::custom_reaction<Ev_Reach_Temp_Goal>,
				sc::custom_reaction<Ev_Reach_Dest>,
				sc::custom_reaction<Ev_Obst_Detected> > reactions;
		typedef sc::state<RRTSPlan, Moving, EmerStop> Base;

		RRTSPlan(my_context ctx ): Base (ctx){
			ROS_INFO("Entering RRTSPlan State");
			geometry_msgs::PoseStamped sub_goal;

			ROS_INFO("Setting Goal");
			while (context<ObstAvoidSM>().SubGoal->getSubgoal(sub_goal) == 0){
				ROS_INFO("Failed to get goal");
			}
			context<ObstAvoidSM>().RRTStar->set_goal(sub_goal);

			ROS_INFO("Replanning");
			context<ObstAvoidSM>().RRTStar->planner_timer.start();
		}

		virtual ~RRTSPlan(){
			ROS_INFO("Leaving RRTSPlan State");
			context<ObstAvoidSM>().RRTStar->planner_timer.stop();
		}

		sc::result react(const Ev_Reach_Temp_Goal &){
			return transit<NormalPlan>();
		}

		sc::result react(const Ev_Reach_Dest &){
			return forward_event();
		}

		sc::result react(const Ev_Obst_Detected&){
			return discard_event();
		}
	};

	struct EmerStop:sc::state<EmerStop, RRTSPlan>{

		typedef mpl::list<
				sc::custom_reaction<Ev_Obst_Detected>,
				sc::custom_reaction<Ev_RRTSPath_Found>
		> reactions;

		typedef sc::state<EmerStop, RRTSPlan> Base;
		EmerStop(my_context ctx):Base(ctx){
			ROS_INFO("Waiting for Replan Path");
			context<ObstAvoidSM>().move_status.emergency = false;
			context<ObstAvoidSM>().move_status.path_exist = true;
			context<ObstAvoidSM>().SpeedControl->SetStatus(1);
		}
		virtual ~EmerStop(){

		}
		sc::result react(const Ev_RRTSPath_Found&){
			return transit<HeadSubGoal>();
		}
		sc::result react(const Ev_Obst_Detected&){
			return discard_event();
		}
	};

	struct HeadSubGoal:sc::state<HeadSubGoal, RRTSPlan>{
		typedef mpl::list<
				sc::custom_reaction<Ev_RRTSPath_Unsafe>,
				sc::custom_reaction<Ev_Reach_Temp_Goal> ,
				sc::custom_reaction<Ev_Reach_Dest>
		> reactions;

		typedef sc::state<HeadSubGoal, RRTSPlan> Base;
		HeadSubGoal(my_context ctx):Base(ctx){
			ROS_INFO("Heading to SubGoal");
			context<ObstAvoidSM>().move_status.emergency = false;
			context<ObstAvoidSM>().move_status.path_exist = true;
			context<ObstAvoidSM>().SpeedControl->SetStatus(2);
		}

		virtual ~HeadSubGoal(){

		}

		sc::result react(const Ev_RRTSPath_Unsafe&){
			return transit<EmerStop>();
		}

		sc::result react(const Ev_Reach_Temp_Goal&){
			return forward_event();
		}

		sc::result react(const Ev_Reach_Dest&){
			return forward_event();
		}
	};
}

#endif /* PLANNERSHIFTAUTO_HPP_ */
