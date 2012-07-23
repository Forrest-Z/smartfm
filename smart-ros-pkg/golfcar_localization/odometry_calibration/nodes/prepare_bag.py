#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os

import roslib; roslib.load_manifest('odometry_calibration')
import rospy
import rosbag


inbag_fn = sys.argv[1]
outbag_fn = os.path.splitext(inbag_fn)[0]+'_filt.bag'
print 'Reading from', inbag_fn, 'and saving to', outbag_fn

first_time = True
t0 = None

with rosbag.Bag(outbag_fn, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(inbag_fn).read_messages():
        if t0 is None:
            t0 = t
        if topic == '/amcl_pose' and first_time and t>t0+rospy.Duration(3):
            first_time = False
            msg.header.seq = 0
            msg.header.stamp = rospy.Time()
            outbag.write('/initialpose', msg, t)
        elif topic.startswith(("/encoder_counts", "/ms/imu", "/scan", "/sick_scan2", "/sickldmrs")):
            outbag.write(topic, msg, t)
