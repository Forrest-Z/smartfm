#!/usr/bin/env python
# -*- coding: utf-8 -*-

''' A common class for communication with the dbserver '''

import roslib; roslib.load_manifest('dbserver_comm')
import rospy

import urllib, urllib2
from xml.dom import minidom

class DBServerComm:
    def __init__(self):
        self.vehicleID = rospy.get_param('~vehicleID', 'golfcart1')
        self.server = rospy.get_param('/dbserver/url', 'http://fmautonomy.no-ip.info/dbserver')

    def calldb(self, php, params={}):
        params.update({'VehicleID': self.vehicleID});
        f = urllib2.urlopen(self.server+"/"+php, urllib.urlencode(params))
        dom = minidom.parse(f)
        f.close()
        nodes = dom.getElementsByTagName('status')
        if len(nodes)>0 and nodes[0].getAttribute('code') == 'err':
            raise Exception( nodes[0].getAttribute('msg') )
        else:
            return dom
