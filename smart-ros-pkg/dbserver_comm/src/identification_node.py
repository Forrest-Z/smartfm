#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
Creates a new vehicle entry in the database at startup, and deletes it when
quiting. Meanwhile, listen from string messages on topic
'missions/currentLocation' and update the database.
'''


import roslib; roslib.load_manifest('dbserver_comm')
import rospy
from std_msgs.msg import String
from dbserver_comm import DBServerComm



rospy.init_node('identification_node')
dbserverComm = DBServerComm()

def callback(string):
    dbserverComm.setCurrentLocation(string.data)

dbserverComm.identify()
sub = rospy.Subscriber('missions/currentLocation', String, callback)
rospy.spin()
dbserverComm.deleteVehicle()
