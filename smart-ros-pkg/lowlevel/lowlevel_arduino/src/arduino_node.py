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


rospy.init_node('arduino_node')

brake_change_threshold = rospy.get_param('~brake_change_threshold', 0)
steer_change_threshold = rospy.get_param('~steer_change_threshold', 0)

arduino_cmd_msg = Arduino()
arduino_pub = rospy.Publisher('arduino_cmd', Arduino)

rate = rospy.get_param('~rate',10)
rospy.loginfo('Sending messages to the Arduino board every %dms' % int(1000.0/rate))



# diagnostics

fs_diag = DIAG.FrequencyStatus(DIAG.FrequencyStatusParam({'min': rate, 'max': rate}, 0.1, 20))

last_watchdog = None

def watchdog(stat):
    if last_watchdog is None:
        stat.summary(2, 'Arduino board not started')
    else:
        d = (rospy.Time.now()-last_watchdog).to_sec()
        if d > .5:
            stat.summary(2, 'Arduino board died')
        else:
            stat.summary(0, 'Arduino board is alive')
        stat.add('Last message received', last_watchdog.to_sec())
        stat.add('Elapsed since last message', d)
    return stat

watchdog_diag = DIAG.GenericFunctionDiagnosticTask('board watchdog', watchdog)

diag_updater = DIAG.Updater()
diag_updater.setHardwareID('none')
diag_updater.add(fs_diag)
diag_updater.add(watchdog_diag)

def arduinoCB(msg):
    # subscribe to button state topic to check that the arduino is still alive
    global last_watchdog
    last_watchdog = rospy.Time.now()
    diag_updater.update()

arduino_sub = rospy.Subscriber('button_state_emergency', Bool, arduinoCB)





def throttleCB(msg):
    arduino_cmd_msg.throttle = msg.data

def brakeCB(msg):
    # in speed controller, braking is done in the negative direction,
    # but in arduino it is in the positive direction.
    a = -msg.data

    # Reduce a bit the amount of changes on the brake: fine positioning is not
    # required, and the brake makes strange noise when there are many small
    # steps.
    if abs(arduino_cmd_msg.brake_angle-a) > brake_change_threshold:
        arduino_cmd_msg.brake_angle = a

def steerCB(msg):
    # in steering controller, positive steering is to the right. For arduino
    # it is the opposite.
    a = -msg.data

    # Reduce a bit the amount of changes on the steering: fine positioning is not
    # required, and the motor makes strange noise when there are many small
    # steps.
    if abs(arduino_cmd_msg.steer_angle-a) > steer_change_threshold:
        arduino_cmd_msg.steer_angle = a

def setupSub(topic,msg_t,cb):
    sub = rospy.Subscriber(topic, msg_t, cb)
    rospy.loginfo('Setup Subscriber on %s [%s]' % (topic, msg_t._type))
    return sub


throttle_sub = setupSub('throttle', Float64, throttleCB)
brake_sub = setupSub('brake_angle', Float64, brakeCB)
steer_sub = setupSub('steer_angle', Float64, steerCB)



while not rospy.is_shutdown():
    arduino_pub.publish(arduino_cmd_msg)
    fs_diag.tick()
    diag_updater.update()
    rospy.sleep(1.0/rate)
