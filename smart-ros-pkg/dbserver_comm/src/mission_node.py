#!/usr/bin/env python
# -*- coding: utf-8 -*-


''' A node to handle mission requests and update mission/vehicle status.

-) Checks for a new mission or mission cancelling and publish Mission messages
   on 'missions/assignments'.
-) Listens for Mission messages on 'missions/feedback' and updates the database.
'''


import roslib; roslib.load_manifest('dbserver_comm')
import rospy

from dbserver_comm.msg import Mission
from dbserver_comm import DBServerComm

import time


rospy.init_node('mission_node')
dbserverComm = DBServerComm()
missionPub = rospy.Publisher('missions/assignments', Mission)
currentMission = None


def getNextMissionAssignment():
    dom = dbserverComm.calldb('list_requests.php')
    print dom.toxml()
    for node in dom.getElementsByTagName('request'):
        if node.getAttribute('status') == 'Acknowledged':
            m = Mission()
            m.requestID = int(node.getAttribute('requestID'))
            m.customerID = str(node.getAttribute('customerID'))
            m.status = str(node.getAttribute('status'))
            m.pickUpLocation = str(node.getAttribute('pickUpLocation'))
            m.dropOffLocation = str(node.getAttribute('dropOffLocation'))
            return m
    return None


def checkMissionCancel():
    dom = dbserverComm.calldb('list_requests.php')


def missionCB(mission):
    '''Updates the status of the vehicle and the status of the mission.'''
    if currentMission is None:
        rospy.logdebug('Received a mission feedback but not following any mission now!')
        return

    if currentMission.missionID != mission.missionID:
        rospy.logdebug('Received a mission feedback for a mission other than the one I am following now!')
        return

    dbserverComm.calldb('veh_update_status.php', {'Status': mission.status})
    currentMission.status = mission.status

    status = None
    if mission.status == 'GoingToPickupLocation':
        status = 'Confirmed'
    elif mission.status == 'GoingToDropOffLocation':
        status = 'Processing'
    elif mission.status == 'WaitingForAMission':
        status = 'Completed'
    if status is not None:
        dbserverComm.calldb('veh_update_request.php', {'MissionID':mission.missionID, 'Status':status})
        if status == 'Completed':
            currentMission = None



missionSub = rospy.Subscriber('missions/feedback', Mission, missionCB)

while not rospy.is_shutdown():
    nextMission = getNextMissionAssignment()
    if nextMission is not None:
        currentMission = nextMission
        missionPub.publish(currentMission)
        nextMission = None
    time.sleep(5)
