/*
 * NavigationSMNode.hpp
 *
 *  Created on: 29 Nov, 2013
 *      Author: liuwlz
 */

#ifndef NAVIGATIONSMNODE_HPP_
#define NAVIGATIONSMNODE_HPP_

#include <ros/ros.h>
#include <Automoton_Control/Navigation/NaviSM.hpp>

namespace MPAV{
	class NaviStateMachine{
		NaviSM navi_sm;
	public:
		NaviStateMachine();
		~NaviStateMachine();
	};
}


#endif /* NAVIGATIONSMNODE_HPP_ */
