#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('dead_reckoning')
import rospy
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import diagnostic_updater as DIAG


'''Republishes an odometry topic to 'odom' and broadcasts the corresponding
transform to the base_link frame.
This is useful to select which of multiple odometry sources becomes the main
odometry ('odom') for the navigation stack and the rest of the high level code.
'''


rospy.init_node('odom_republish')

odo_pub = rospy.Publisher('odom', Odometry)
odomBroadcaster = TransformBroadcaster()

diag_updater = DIAG.Updater()
diag_updater.setHardwareID('none')
diag_param_fs = DIAG.FrequencyStatusParam({'min': 5}, 0.1, 5)
diag_task_fs = DIAG.FrequencyStatus(diag_param_fs)
diag_updater.add(diag_task_fs)

def odoCallBack(msg):
    msg.header.frame_id = 'odom'
    odo_pub.publish(msg)
    diag_task_fs.tick()
    diag_updater.update()

    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    odomBroadcaster.sendTransform(
        (p.x, p.y, p.z),
        (o.x, o.y, o.z, o.w),
        msg.header.stamp, 'base_link', 'odom'
    )


odo_sub = rospy.Subscriber('input_topic', Odometry, odoCallBack)
rospy.spin()
