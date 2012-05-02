#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Continuous flow simulation with direct output to ROS for visualization.'''

import time
import sys


import roslib; roslib.load_manifest('tencon_sim')
import rospy

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

from tencon_sim import *



rospy.init_node('tencon_cont_sim', sys.argv)
rospy.set_param('/use_sim_time', True)
clockPub = rospy.Publisher('/clock', Clock)
basePub = rospy.Publisher('base', PointCloud)
infraPub = rospy.Publisher('infra', PointCloud)
pedPub = rospy.Publisher('pedestrians', PointCloud)



#------------------------------------------------------------------------------
# simulation parameters
#

params = {}

# time step of the simulation:
params['sim_time_step'] = 0.1
    
# position of the beginning of the pedestrian crossing on the pedestrian axis
params['ped_crossing_length'] = 2
    
# position of the beginning of the pedestrian crossing on the vehicle axis
params['ped_crossing_width'] = 1
    
# Pedestrian velocity
params['ped_vel'] = 1
params['ped_start_pos'] = -20    

params['veh_start_pos'] = -100
params['veh_max_vel'] = 3
params['veh_acc'] = 3
params['safety_dist'] = 2
    
params['infra_fov_dist']= 30

params['lambda_ped'] = 0.1
params['lambda_veh'] = 0.8


flow_sim = FlowSim(params)


def publish(sim):
    T = rospy.Time.from_sec(sim.t)
    clockPub.publish( Clock(clock=T) )
    
    msg = PointCloud()
    msg.header.stamp = T
    msg.header.frame_id = 'map'
    for p in sim.peds:
        msg.points.append(Point32(x=0, y=-p.x, z=0))
    pedPub.publish(msg)
    
    for k, pub, offset in (('base', basePub, -0.5), ('infra', infraPub, 0.5)):
        msg = PointCloud()
        msg.header.stamp = T
        msg.header.frame_id = 'map'
        for v in sim.vehs[k]:
            msg.points.append(Point32(x=v.x, y=offset))
        pub.publish(msg)



old_time = time.time()
while not rospy.is_shutdown():
    flow_sim.step()
    publish(flow_sim)
    if time.time() - old_time > 3:
        old_time = time.time()
        try:
            flow_sim.print_dt_stats()
            print '-'*20
        except RuntimeError:
            pass
    time.sleep(params['sim_time_step'])
