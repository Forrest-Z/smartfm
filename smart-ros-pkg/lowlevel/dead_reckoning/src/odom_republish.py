#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('dead_reckoning')
import rospy
import tf.transformations
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import diagnostic_updater as DIAG


'''Republishes and odometry topic to 'odom' and broadcasts an identity transform
between the 2 frames.
This is useful to select which of multiple odometry sources becomes the main
odometry ('odom') for the navigation stack and the rest of the high level code.
'''


rospy.init_node('odom_republish')

output_frame = rospy.get_param('~output_frame', 'odom')

odo_pub = rospy.Publisher('odom', Odometry)
odomBroadcaster = TransformBroadcaster()

diag_updater = DIAG.Updater()
diag_updater.setHardwareID('none')
diag_param_fs = DIAG.FrequencyStatusParam({'min': 5, 'max': 1000}, 0.1, 20)
diag_task_fs = DIAG.FrequencyStatus(diag_param_fs)
diag_updater.add(diag_task_fs)

def odoCallBack(msg):
    input_frame = msg.header.frame_id
    msg.header.frame_id = output_frame
    odo_pub.publish(msg)
    diag_task_fs.tick()
    diag_updater.update()

    odomBroadcaster.sendTransform(
        (0, 0, 0),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        msg.header.stamp, input_frame, output_frame
    )


odo_sub = rospy.Subscriber('input_topic', Odometry, odoCallBack)
rospy.spin()