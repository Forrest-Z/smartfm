#!/usr/bin/env python
# -*- coding: utf-8 -*-


''' Gets and publish the station list.

Periodically monitors the station list on the db server and publishes it.

Parameters:
- /dbserver/url: the URL of the server (defaults to http://fmautonomy.no-ip.info/dbserver)
- /dbserver/station_lister/check_period: checking the database at this period (defaults to 15 sec)
- /dbserver/station_lister/pub_period: publishing the messages at this period (defaults to 1 sec)

Topics published:
- /dbserver/stations (type dbserver_comm/Stations)
'''


import roslib; roslib.load_manifest('dbserver_comm')
import rospy

from dbserver_comm.msg import Station, Stations
from dbserver_comm import DBServerComm

import time



rospy.init_node('station_lister')

checkPeriod = rospy.get_param('/dbserver/station_lister/check_period', 15)
pubPeriod = rospy.get_param('/dbserver/station_lister/pub_period', 1)

pub = rospy.Publisher('/dbserver/stations', Stations)

dbserverComm = DBServerComm()


lastCheck = - checkPeriod
msg = Stations()

while not rospy.is_shutdown():
    if rospy.get_time() > lastCheck + checkPeriod:
        msg.stations = [Station(**s) for s in dbserverComm.listStations()]

    pub.publish(msg)
    time.sleep(pubPeriod)
