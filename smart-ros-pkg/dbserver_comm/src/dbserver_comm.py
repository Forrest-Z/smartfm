#!/usr/bin/env python
# -*- coding: utf-8 -*-


''' A node to handle the communication with the dbserver.

-) Get the vehicleID from parameter server and create an entry in the database. Check if the entry is already present...
-) Publish the location of the car. Subscribe to the GPS topic and forward it to the dbserver.
-) Check for a new mission or mission cancelling.

Communication with the database is through server side PHP scripts.
'''


import roslib; roslib.load_manifest('dbserver_comm')
import rospy
from sensor_msgs.msg import NavSatFix
from dbserver_comm.msg import Mission

import urllib, urllib2
import time
from xml.dom import minidom


def  nodeToMissionMsg(node):
    m = Mission()
    m.requestID = node.getAttribute('requestID')
    m.customerID = node.getAttribute('customerID')
    m.status = node.getAttribute('status')
    m.pickUpLocation = node.getAttribute('pickUpLocation')
    m.dropOffLocation = node.getAttribute('dropOffLocation')
    return m



class DBServerComm:
    def __init__(self):
        self.gpsPose = None
        self.gpsLastPub = 0
        self.gpsPoseUpdated = False
        self.gpsPubPeriod = rospy.get_param('~gpsPubPeriod', 10)
        self.vehicleID = rospy.get_param('~vehicleID', 'golfcart1')
        self.server = rospy.get_param('~dbServerUrl', 'http://fmautonomy.no-ip.info/dbserver')

        self.currentMission = None

        self.gpsSub = rospy.Subscriber('/fix', NavSatFix, self.gpsCB)
        self.missionPub = rospy.Publisher('/missions/assignments', Mission)
        self.missionSub = rospy.Subscriber('/missions/feedback', Mission, self.missionCB)

    def __del__(self):
        self.calldb("delete_vehicle.php")

    def calldb(self, php, params={}):
        params.update({'VehicleID': self.vehicleID});
        f = urllib2.urlopen(self.server+"/"+php, urllib.urlencode(params))
        dom = minidom.parse(f)
        f.close()
        node = dom.getElementsByTagName('status')[0]
        if node.getAttribute('code') == 'err':
            raise Exception( node.getAttribute('msg') )
        else:
            return dom

    def updateVehStatus(self, params):
        self.calldb("veh_update_status.php", params)

    def identifyLoop(self):
        while not rospy.is_shutdown():
            try:
                self.calldb("new_vehicle.php")
            except Exception, e:
                print "Exception occured:", e
            else:
                print "Identified"
                break

    def gpsCB(self, gpsmsg):
        self.gpsPose = {'Latitude':gpsmsg.latitude, 'Longitude':gpsmsg.longitude}
        self.gpsPoseUpdated = True

    def pubGpsLoc(self):
        if self.gpsPose is not None and self.gpsPoseUpdated and time.time() > self.gpsLastPub + self.gpsPubPeriod:
            self.updateVehStatus(self.gpsPose)
            self.gpsLastPub = time.time()
            self.gpsPoseUpdated = False

    def getMissionList(self):
        dom = self.calldb("list_veh_requests.php")
        return [nodeToMissionMsg(node) for node in dom.getElementsByTagName('request')]

    def getNextMissionAssignment(self):
        for m in self.getMissionList():
            if m['status'] == 'Acknowledged':
                return m
        return None

    def getNextMissionAssignmentLoop(self):
        self.currentMission = self.getNextMissionAssignment()
        while self.currentMission is None and not rospy.is_shutdown():
            time.sleep(5)
            self.currentMission = self.getNextMissionAssignment()
        self.missionPub.publish(self.currentMission)

    def missionCB(self, mission):
        assert( self.currentMission is not None and self.currentMission.missionID==mission.missionID )
        self.updateVehStatus({'Status': mission.status})
        self.currentMission.status = mission.status

        status = None
        if mission.status == 'GoingToPickupLocation': status = 'Confirmed'
        elif mission.status == 'GoingToDropOffLocation': status = 'Processing'
        elif mission.status == 'WaitingForAMission': status = 'Completed'
        if status is not None:
            self.calldb('veh_update_request.php', {'MissionID':mission.missionID, 'Status':status})
            if status == 'Completed': self.currentMission = None


if __name__ == '__main__':

    rospy.init_node('dbserver_comm')
    dbsc = DBServerComm()
    dbsc.identifyLoop()

    while not rospy.is_shutdown():
        dbsc.getNextMissionAssignmentLoop()
        while dbsc.currentMission is not None and not rospy.is_shutdown():
            time.sleep(5)
