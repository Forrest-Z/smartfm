#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
Creates a new vehicle entry in the database at startup, and deletes it when quiting
'''


import roslib; roslib.load_manifest('dbserver_comm')
import rospy
from dbserver_comm import DBServerComm


rospy.init_node('identification_node')
dbserverComm = DBServerComm()
dbserverComm.identify()
rospy.spin()
dbserverComm.deleteVehicle()
