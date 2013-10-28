/*
 * main.cpp
 *
 *  Created on: May 22, 2013
 *      Author: liuwlz
 */

#include "rrts_node_exp.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrts_node");

  PlannerExp my_planner;

  ros::spin();
}
