#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('odometry_calibration')
import rospy

import tf
import math
from std_msgs.msg import Bool


rospy.init_node('force_stop_at_ped_crossing')

pub = rospy.Publisher('safety_stop', Bool)
listener = tf.TransformListener()
stop_duration = 5
stop_x = 141
stop_y = 52
thr = 5

while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('/base_link', '/map', rospy.Time(0))
    except tf.Exception as e:
        rospy.logwarn("Could not transform from base_link to map: " + str(e))
        continue
    dx = trans[0]-stop_x
    dy = trans[1]-stop_y
    d = math.sqrt(dx*dx + dy*dy)
    if d < thr:
        pub.publisher(Bool(True))
        rospy.sleep(stop_duration)
        pub.publisher(Bool(False))
        break
    else:
        rospy.sleep(.1)
