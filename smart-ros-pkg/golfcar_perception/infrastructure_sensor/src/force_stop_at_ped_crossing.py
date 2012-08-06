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
thr = 8

while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except tf.Exception as e:
        rospy.logwarn("Could not transform from map to base_link: " + str(e))
        continue
    rospy.loginfo("Position of the robot: (%f, %f)" % (trans[0], trans[1]))
    dx = trans[0]-stop_x
    dy = trans[1]-stop_y
    d = math.sqrt(dx*dx + dy*dy)
    rospy.loginfo("Distance to the stop point: %f. Stopping distance: %f" % (d, thr))
    if d < thr:
        rospy.loginfo("Stopping for %f seconds" % stop_duration)
        pub.publish(Bool(True))
        rospy.sleep(stop_duration)
        rospy.loginfo("Resuming motion")
        pub.publish(Bool(False))
        break
    else:
        rospy.sleep(.1)

rospy.loginfo("Quitting...")
