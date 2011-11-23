#!/usr/bin/env python
# -*- coding: utf-8 -*-


''' A node to handle the communication with the dbserver.

-) Gets the vehicleID from parameter server.
-) Checks for a new mission or mission cancelling.
-) Listens for mission messages and update the database.

Communication with the database is through server side PHP scripts.
'''


import roslib; roslib.load_manifest('dbserver_comm')
import rospy

from dbserver_comm.msg import Mission
from dbserver_comm import DBServerComm

import time




def nodeToMissionMsg(node):
    m = Mission()
    m.requestID = node.getAttribute('requestID')
    m.customerID = node.getAttribute('customerID')
    m.status = node.getAttribute('status')
    m.pickUpLocation = node.getAttribute('pickUpLocation')
    m.dropOffLocation = node.getAttribute('dropOffLocation')
    return m





class MissionNode:
    def __init__(self):
        self.dbserverComm = DBServerComm()
        self.currentMission = None

        self.missionPub = rospy.Publisher('missions/assignments', Mission)
        self.missionSub = rospy.Subscriber('missions/feedback', Mission, self.missionCB)


    def getMissionList(self):
        dom = self.dbserverComm.calldb("list_veh_request.php")
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
        self.dbserverComm.calldb("veh_update_status.php", {'Status': mission.status})
        self.currentMission.status = mission.status

        status = None
        if mission.status == 'GoingToPickupLocation': status = 'Confirmed'
        elif mission.status == 'GoingToDropOffLocation': status = 'Processing'
        elif mission.status == 'WaitingForAMission': status = 'Completed'
        if status is not None:
            self.dbserverComm.calldb('veh_update_request.php', {'MissionID':mission.missionID, 'Status':status})
            if status == 'Completed': self.currentMission = None





if __name__ == '__main__':

    rospy.init_node('dbserver_comm')
    n = MissionNode()

    while not rospy.is_shutdown():
        n.getNextMissionAssignmentLoop()
        while n.currentMission is not None and not rospy.is_shutdown():
            time.sleep(5)
