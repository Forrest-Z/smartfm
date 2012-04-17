#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
ROS node in charge of interfacing with the encoders phidget, and Encoders messages
on the "encoders" topic.

Parameters:
- dist_btw_wheels: distance between wheels (default value: 0.995).
- wheel_size: size (perimeter) of the wheels (default value: 1.335).
- left_correction_factor: correction factor to balance the 2 wheels
  (default value: 1.011). Increase it when the vehicle drifts to the left
  (to be verified).
- period: publish only after this period has elapsed (default value: 0.02).
  This limits the publishing rate. It seems the phidget publishes a count
  change every 8ms.
- min_pub_period: if set, we want to publish messages with this period, even if
  encoders are not changing. Otherwise, publish only when the encoders value is
  changing.
'''

import roslib; roslib.load_manifest('phidget_encoders')
import rospy

from Phidgets.PhidgetException import *
from Phidgets.Events.Events import *
from Phidgets.Devices.Encoder import Encoder

from phidget_encoders.msg import Encoders as EncodersMsg
import diagnostic_updater as DIAG


def err(e):
    '''A helper function to report Phidget errors'''
    rospy.logerr("Phidget error %i: %s. Exiting..." % (e.code, e.details))
    exit(1)


class CountBuffer:
    def __init__(self, wheelSize, counts=6000):
        self.wheelSize = wheelSize
        self.counts = counts
        self.reset()

    def reset(self):
        self.dp = 0.0 #dist (m)
        self.dt = 0.0 #time (sec)
        self.n = 0 #number of values cumulated

    def add(self, e):
        self.dp += e.positionChange * self.wheelSize / self.counts
        self.dt += e.time * 1e-6 #usec to sec
        self.n += 1


class PhidgetEncoder:
    '''Monitor the pose of each encoders'''

    def __init__(self):
        self.distBtwWheels = rospy.get_param('~dist_btw_wheels',0.995)
        wheelSize = rospy.get_param('~wheel_size', 1.32212398060626)#original:1.335)
        leftCorrectionFactor = rospy.get_param('~left_correction_factor', 1.011)

        self.minPubPeriod = rospy.get_param('~min_pub_period', None)
        self.period = rospy.get_param('~period', 0.02)

        self.left = 0
        self.right = 1
        self.countBufs = {self.left: CountBuffer(wheelSize*leftCorrectionFactor), self.right: CountBuffer(-wheelSize)}
        self.lastPub = None

        self.dt = 0.0
        self.d_th = 0.0
        self.d_dist = 0.0

        self.initPhidget()
        self.encoder.setEnabled(self.left, True)
        self.encoder.setEnabled(self.right, True)

        self.encodersPub = rospy.Publisher('encoders', EncodersMsg)

        # diagnostics
        self.diag_updater = DIAG.Updater()
        self.diag_updater.setHardwareID('none')
        f1 = 1.0/self.period
        f2 = 0
        if self.minPubPeriod is not None:
            f2 = 1.0/self.minPubPeriod
        f = {'min': min([f1,f2]), 'max': max([f1,f2])}
        fs_params = DIAG.FrequencyStatusParam(f, 0.1, 20)
        self.fs_diag = DIAG.HeaderlessTopicDiagnostic('encoders', self.diag_updater, fs_params)


    def initPhidget(self):
        '''Connect to the phidget and init communication.'''

        #Create an encoder object (from the Phidget library)
        try:
            self.encoder = Encoder()
        except RuntimeError as e:
            rospy.logerr("Runtime exception: %s. Exiting..." % e.details)
            exit(1)

        #Set the callback
        try:
            self.encoder.setOnPositionChangeHandler(self.encoderPositionChange)
        except PhidgetException as e:
            err(e)


        rospy.loginfo("Opening phidget object....")
        try:
            self.encoder.openPhidget()
        except PhidgetException as e:
            err(e)

        rospy.loginfo("Waiting for attach....")
        try:
            self.encoder.waitForAttach(10000)
        except PhidgetException as e:
            rospy.logerr("Phidget error %i: %s" % (e.code, e.details))
            try:
                self.encoder.closePhidget()
            except PhidgetException as e:
                err(e)
            exit(1)


    def closePhidget(self):
        rospy.loginfo("Closing...")
        try:
            self.encoder.closePhidget()
        except PhidgetException as e:
            err(e)


    def encoderPositionChange(self, e):
        '''A callback function called whenever the position changed'''

        #rospy.loginfo("Encoder %i: Encoder %i -- Change: %i -- Time: %i -- Position: %i" % (e.device.getSerialNum(), e.index, e.positionChange, e.time, self.encoder.getPosition(e.index)))

        if e.index in self.countBufs.keys():
            self.countBufs[e.index].add(e)
            dts = [b.dt for b in self.countBufs.values()];
	    if min(dts) >= self.period:
                if self.countBufs[self.left].n == self.countBufs[self.right].n:
                    #rospy.loginfo("Encoders: got equal number of counts.")
                    self.dt = sum(dts)/len(dts)
                    #rospy.loginfo("Encoders: dt1 = %f, dt2 = %f --> dt=%f" % (dts[0], dts[1], self.dt))
                    self.process()
                else:
                    pass
                    #rospy.loginfo("encoders: time criteria met, but not count criteria")


    def process(self):
        self.now = rospy.Time.now()
        self.pubEncodersMsg()
        self.lastPub = self.now


    def pubEncodersMsg(self):
        dl = self.countBufs[self.left].dp
        dr = self.countBufs[self.right].dp
        self.d_dist = (dl+dr)/2
        self.d_th = (dr-dl)/self.distBtwWheels

        encodersMsg = EncodersMsg()
        encodersMsg.stamp = self.now
        encodersMsg.dt = self.dt
        encodersMsg.d_left = dl;
        encodersMsg.d_right = dr;
        encodersMsg.d_dist = self.d_dist
        encodersMsg.d_th = self.d_th
        if self.dt>0:
            encodersMsg.v = self.d_dist/self.dt
            encodersMsg.w = self.d_th/self.dt
        else:
            encodersMsg.v = 0
            encodersMsg.w = 0

        self.encodersPub.publish(encodersMsg)

        # diagnostics
        self.fs_diag.tick()
        self.diag_updater.update()

        self.countBufs[self.left].reset()
        self.countBufs[self.right].reset()


    def loop(self):
        ''' We want to publish messages even if the vehicle is not moving. '''

        if self.minPubPeriod is None:
            rospy.sleep(10)
            return

        if self.lastPub is None:
            self.dt = 0
            self.process()
        else:
            dt = (rospy.Time.now()-self.lastPub).to_sec()
            if dt > self.minPubPeriod:
                self.dt = dt
                self.process()

        rospy.sleep(self.minPubPeriod)



if __name__=='__main__':
    rospy.init_node('encoders_node', log_level=rospy.DEBUG)
    enc = PhidgetEncoder()
    rospy.loginfo('encoders: Spinning...')

    while not rospy.is_shutdown():
        enc.loop()

    enc.closePhidget()
    rospy.loginfo("Done.")
    exit(0)
