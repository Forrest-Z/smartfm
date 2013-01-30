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

class OdomRepublish:

    def __init__(self):
        self.initialized = False
        self.pre_x = 0.0
        self.pre_y = 0.0
        self.pre_z = 0.0
        self.odo_sub = rospy.Subscriber('input_topic', Odometry, self.odoCallBack)
        self.odo_pub = rospy.Publisher('odom', Odometry)
        self.odomBroadcaster = TransformBroadcaster()

        self.diag_updater = DIAG.Updater()
        self.diag_updater.setHardwareID('none')
        diag_param_fs = DIAG.FrequencyStatusParam({'min': 5}, 0.1, 5)
        self.diag_task_fs = DIAG.FrequencyStatus(diag_param_fs)
        self.diag_updater.add(self.diag_task_fs)

    def odoCallBack(self, msg):
        msg.header.frame_id = 'odom'
        
        self.diag_task_fs.tick()
        self.diag_updater.update()

        if self.initialized == False:
          self.pre_x = msg.pose.pose.position.x
          self.pre_y = msg.pose.pose.position.y
          self.pre_z = msg.pose.pose.position.z
          self.initialized = True
        p = msg.pose.pose.position
        p.x = p.x - self.pre_x
        p.y = p.y - self.pre_y
        p.z = p.z - self.pre_z
        o = msg.pose.pose.orientation
        self.odomBroadcaster.sendTransform(
            (p.x, p.y, p.z),
            (o.x, o.y, o.z, o.w),
            msg.header.stamp, 'base_link', 'odom'
        )
        msg.pose.pose.position = p
        self.odo_pub.publish(msg)

if __name__=='__main__':
      rospy.init_node('odom_republish')
      node = OdomRepublish()
     

      rospy.spin()

