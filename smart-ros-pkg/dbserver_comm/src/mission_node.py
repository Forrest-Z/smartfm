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

from dbserver_comm.srv import MissionRequest
from dbserver_comm.msg import Mission
from dbserver_comm import DBServerComm


def nodeToMission(node):
    '''Converts an XML node to a mission object.'''
    m = Mission()
    m.requestID = int(node.getAttribute('requestID'))
    m.customerID = str(node.getAttribute('customerID'))
    m.status = str(node.getAttribute('status'))
    m.pickUpLocation = str(node.getAttribute('pickup'))
    m.dropOffLocation = str(node.getAttribute('dropoff'))
    return m


class MissionNode:

    def __init__(self):
        self.dbserverComm = DBServerComm()
        self.currentMission = None
        self.lock = threading.RLock()
        self.missionReqSrv = rospy.Service('missions/assignments', MissionRequest, self.missionReqCB)
        self.missionSub = rospy.Subscriber('missions/feedback', Mission, self.missionCB)


    def checkForMission(self):
        '''
        Checks the database if there is a mission pending (with status
        'Acknowledged'). If there is, sets it as the current mission (currentMission)
        and returns True, otherwise returns False.
        '''
        dom = self.dbserverComm.calldb('list_requests.php')
        #print dom.toxml()
        for node in dom.getElementsByTagName('request'):
            m = nodeToMission(node)
            if m.status=='Confirmed':
                with self.lock: self.currentMission = m
                return True
        return False


    def checkMissionStatus(self):
        '''
        Checks the database for the status of the current mission and updates
        currentMission.
        '''
        with self.lock:
            dom = self.dbserverComm.calldb('list_requests.php', {'requestID': self.currentMission.requestID})
            node = dom.getElementsByTagName('request')[0]
            self.currentMission.status = str(node.getAttribute('status'))


    def missionReqCB(self, req):
        with self.lock:
            if req.checkStatus:
                if self.currentMission is None:
                    rospy.logdebug('Checking the status of the current mission while not currently on a mission.')
                    return None
                self.checkMissionStatus()
                rospy.loginfo('Checked mission status: ' + self.currentMission.status)
                return self.currentMission

            self.currentMission = None
            while not rospy.is_shutdown():
                rospy.loginfo('Checking for a new mission.')
                if self.checkForMission():
                    print 'currentMission:', self.currentMission
                    rospy.loginfo('Got a new mission: from %s to %s' % (self.currentMission.pickUpLocation, self.currentMission.dropOffLocation))
                    return self.currentMission
                if req.blocking:
                    rospy.sleep(5)
                else:
                    rospy.loginfo('No new mission.')
                    return None


    def missionCB(self, mission):
        with self.lock:
            if self.currentMission is None:
                rospy.logdebug('Received a mission feedback but not following any mission now!')
            elif self.currentMission.requestID != mission.requestID:
                rospy.logdebug('Received a mission feedback for a mission other than the one I am following now!')
            else:
                self.dbserverComm.calldb('veh_update_status.php', {'Status': mission.vehicleStatus})
                self.dbserverComm.calldb('veh_update_request.php', {'RequestID': mission.requestID, 'Status': mission.status})
                self.currentMission.status = mission.status
                self.currentMission.vehicleStatus = mission.vehicleStatus
                if self.currentMission.status == 'Completed':
                    self.currentMission = None




if __name__=='__main__':
    rospy.init_node('mission_node')
    node = MissionNode()
    rospy.spin()
