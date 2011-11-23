#!/usr/bin/env python
# -*- coding: utf-8 -*-


''' Updates the database with the current GPS location.

-) Get the vehicleID from parameter server and create an entry in the database. Check if the entry is already present...
-) Publish the location of the car. Subscribe to the GPS topic and forward it to the dbserver.

Communication with the database is through server side PHP scripts.
'''


import roslib; roslib.load_manifest('dbserver_comm')
import rospy
from sensor_msgs.msg import NavSatFix

from dbserver_comm import DBServerComm


rospy.init_node('pub_gps')

dbserverComm = DBServerComm()
gpsPubPeriod = rospy.get_param('~gpsPubPeriod', 10)


def gpsCB(gpsmsg):
    gpsPose = {'Latitude':gpsmsg.latitude, 'Longitude':gpsmsg.longitude}
    dbserverComm.calldb("veh_update_status.php", gpsPose)

gpsSub = rospy.Subscriber('fix', NavSatFix, gpsCB)
rospy.spin()
