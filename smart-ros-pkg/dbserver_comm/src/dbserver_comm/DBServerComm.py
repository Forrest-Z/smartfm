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
        ''' Function to interact with the dbserver: calls the script 'php' with
        the arguments in 'params' plus the vehicleID, and reads the return xml doc.
        Raises an exception if there was an error, and returns the parsed xml object
        otherwise.
        '''
        params.update({'VehicleID': self.vehicleID});
        f = urllib2.urlopen(self.server+"/"+php, urllib.urlencode(params))
        xml = f.read()
        f.close()
        #print xml
        dom = minidom.parseString(xml)
        nodes = dom.getElementsByTagName('status')
        if len(nodes)>0 and nodes[0].getAttribute('code') == 'err':
            msg = 'Error while calling ' + php
            if nodes[0].hasAttribute('msg') and nodes[0].getAttribute('msg')!='':
                msg = msg + ': ' + nodes[0].getAttribute('msg')
            raise Exception( msg )
        else:
            return dom

    def identify(self):
        self.calldb('delete_vehicle.php')
        self.calldb('new_vehicle.php')

    def deleteVehicle(self):
        self.calldb('delete_vehicle.php')

    def setCurrentLocation(self, currentLocation):
        self.calldb('veh_update_status.php', {'CurrentLocation': currentLocation})

    def listStations(self):
        dom = self.calldb('list_stations.php')
        nodes = dom.getElementsByTagName('station')

        def nodeToDic(node):
            res = {}
            res['name'] = str(node.getAttribute('name'))
            res['longitude'] = float(node.getAttribute('longitude'))
            res['latitude'] = float(node.getAttribute('latitude'))
            return res

        return [nodeToDic(n) for n in nodes]

    def getVehicleStatus(self):
        dom = self.calldb('list_vehicles.php')
        nodes = dom.getElementsByTagName('vehicle')
        if len(nodes)==0:
            raise Exception("No vehicle with VehicleID: " + self.vehicleID)
        elif len(nodes)>1:
            raise Exception("More than one vehicle with VehicleID: " + self.vehicleID)
        else:
            return str(nodes[0].getAttribute('VehicleID'))