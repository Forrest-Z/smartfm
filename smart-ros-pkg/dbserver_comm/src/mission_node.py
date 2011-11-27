#!/usr/bin/env python
# -*- coding: utf-8 -*-


''' A node to handle mission requests and update mission/vehicle status.

-) Checks for a new mission or mission cancelling and publish Mission messages
   on 'missions/assignments'.
-) Listens for Mission messages on 'missions/feedback' and updates the database.
'''


import roslib; roslib.load_manifest('dbserver_comm')
import rospy

import threading

from dbserver_comm.msg import Mission
from dbserver_comm import DBServerComm


# TODO: add a mutex/lock on currentMission


rospy.init_node('mission_node')
dbserverComm = DBServerComm()
missionPub = rospy.Publisher('missions/assignments', Mission)
currentMission = None
lock = threading.Lock()


def nodeToMission(node):
    m = Mission()
    m.requestID = int(node.getAttribute('requestID'))
    m.customerID = str(node.getAttribute('customerID'))
    m.status = str(node.getAttribute('status'))
    m.pickUpLocation = str(node.getAttribute('pickUpLocation'))
    m.dropOffLocation = str(node.getAttribute('dropOffLocation'))
    return m


def waitNextMission():
    while not rospy.is_shutdown():
        rospy.loginfo('Waiting for a new mission.')
        dom = dbserverComm.calldb('list_requests.php')
        for node in dom.getElementsByTagName('request'):
            m = nodeToMission(node)
            if m.status=='Acknowledged':
                return m
        rospy.sleep(5)


def checkMissionStatus():
    with lock:
        dom = dbserverComm.calldb('list_requests.php', {'requestID': currentMission.requestID})
        node = dom.getElementsByTagName('request')[0]
        currentMission.status = str(node.getAttribute('status'))


def missionCB(mission):
    with lock:
        if currentMission is None:
            rospy.logdebug('Received a mission feedback but not following any mission now!')
        elif currentMission.missionID != mission.missionID:
            rospy.logdebug('Received a mission feedback for a mission other than the one I am following now!')
        else:
            dbserverComm.calldb('veh_update_status.php', {'Status': mission.vehicleStatus})
            dbserverComm.calldb('veh_update_request.php', {'MissionID':mission.missionID, 'Status': mission.status})
            currentMission.status = mission.status
            currentMission.vehicleStatus = mission.vehicleStatus
            if currentMission.status == 'Completed':
                currentMission = None


missionSub = rospy.Subscriber('missions/feedback', Mission, missionCB)


while not rospy.is_shutdown():
    if currentMission is None or currentMission.status=='Cancelled':
        m = waitNextMission()
        with lock:
            currentMission = m
    else:
        checkMissionStatus()
    missionPub.publish(currentMission)
    rospy.sleep(5)
