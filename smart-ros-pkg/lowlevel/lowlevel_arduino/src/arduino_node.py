#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
ROS node in charge of interfacing with the arduino. Messages from the different
arduino topics are cached and sent to the arduino at a fixed rate. This rate can
be specified by parameter 'rate' (defaults to 10Hz).
'''

import roslib; roslib.load_manifest('lowlevel_arduino')
import rospy
from std_msgs.msg import Bool, Float64
from lowlevel_arduino.msg import Arduino
import diagnostic_updater as DIAG


# a helper function
def setupSub(topic, msg_t, cb):
    sub = rospy.Subscriber(topic, msg_t, cb)
    rospy.loginfo('Setup Subscriber on %s [%s]' % (topic, msg_t._type))
    return sub


class ArduinoNode:

    def __init__(self):
        self.brake_change_threshold = rospy.get_param('~brake_change_threshold', 0)
        self.steer_change_threshold = rospy.get_param('~steer_change_threshold', 0)

        self.arduino_cmd_msg = Arduino()
        self.arduino_pub = rospy.Publisher('arduino_cmd', Arduino)

        rate = rospy.get_param('~rate', 10)
        rospy.loginfo('Sending messages to the Arduino board every %dms' % int(1000.0/rate))

        # monitor the last time we received a button_state_emergency message
        self.last_watchdog = None

        self.arduino_sub = rospy.Subscriber('button_state_emergency', Bool, self.arduinoCB)
        self.throttle_sub = setupSub('throttle', Float64, self.throttleCB)
        self.brake_sub = setupSub('brake_angle', Float64, self.brakeCB)
        self.steer_sub = setupSub('steer_angle', Float64, self.steerCB)

        self.diag_updater = DIAG.Updater()
        self.diag_updater.setHardwareID('none')

        # a frequency diagnostic: make sure we are sending the arduino_cmd message at the
        # appropriate frequency
        fs_param = DIAG.FrequencyStatusParam({'min': rate, 'max': rate}, 0.1, 5)
        self.fs_diag = DIAG.FrequencyStatus(fs_param)
        self.diag_updater.add(self.fs_diag)

        # watchdog diagnostic: reports an error when the board does not respond
        self.diag_updater.add( DIAG.FunctionDiagnosticTask('board watchdog', self.watchdog_diag) )

        # emergency monitor
        self.emergency_state = False
        self.diag_updater.add( DIAG.FunctionDiagnosticTask('emergency button', self.emergency_diag) )

        self.timer = rospy.Timer(rospy.Duration(1.0/rate), self.timer_cb)


    def arduinoCB(self, msg):
        # A callback to update last_watchdog whenever a button_state_emergency message
        # is received
        self.emergency_state = msg.data
        self.last_watchdog = rospy.Time.now()
        self.diag_updater.update()


    def watchdog_diag(self, stat):
        # a function diagnostic to monitor the watchdog status
        if self.last_watchdog is None:
            stat.summary(2, 'Arduino board not started')
        else:
            d = (rospy.Time.now()-self.last_watchdog).to_sec()
            if d > .6:
                stat.summary(2, 'Arduino board died')
            else:
                stat.summary(0, 'Arduino board is alive')
            stat.add('Last message received', self.last_watchdog.to_sec())
            stat.add('Elapsed since last message', d)
        return stat


    def emergency_diag(self, stat):
        # a function diagnostic to monitor the emergency button status
        if self.last_watchdog is None:
            stat.summary(2, 'Arduino board not started')
        elif self.emergency_state:
            stat.summary(2, 'Emergency state')
        else:
            stat.summary(0, 'Normal running state')
        return stat


    def throttleCB(self, msg):
        self.arduino_cmd_msg.throttle = msg.data


    def brakeCB(self, msg):
        # in speed controller, braking is done in the negative direction,
        # but in arduino it is in the positive direction.
        a = -msg.data

        # Reduce a bit the amount of changes on the brake: fine positioning is not
        # required, and the brake makes strange noise when there are many small
        # steps.
        if abs(self.arduino_cmd_msg.brake_angle-a) > self.brake_change_threshold:
            self.arduino_cmd_msg.brake_angle = a


    def steerCB(self, msg):
        # in steering controller, positive steering is to the right. For arduino
        # it is the opposite.
        a = -msg.data

        # Reduce a bit the amount of changes on the steering: fine positioning is not
        # required, and the motor makes strange noise when there are many small
        # steps.
        if abs(self.arduino_cmd_msg.steer_angle-a) > self.steer_change_threshold:
            self.arduino_cmd_msg.steer_angle = a


    def timer_cb(self, dummy):
        # Main loop
        self.arduino_pub.publish(self.arduino_cmd_msg)
        self.fs_diag.tick()
        self.diag_updater.update()




rospy.init_node('arduino_node')
n = ArduinoNode()
rospy.spin()
