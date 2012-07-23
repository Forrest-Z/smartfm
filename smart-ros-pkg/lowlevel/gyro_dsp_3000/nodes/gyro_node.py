#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Driver for the DSP 3000 optical gyroscope.

Must be given the port as a parameter. Publishes the angular position in degrees on
topic ~angle.
'''

# Possible improvements:
#It is currently configured to let the gyro do the integration, and direcly outputs
#the angular position (in degrees). But this gyro can also output angular rate or
#angular deltas.
#This could be passed as a parameter. The message could contain the 3 of them, plus
#a header. One of the 3 values would be set by the gyro, and the 2 others computed
#from it.

import roslib; roslib.load_manifest('gyro_dsp_3000')
import rospy
import serial

from std_msgs.msg import Float64


class GyroNode:

    def __init__(self):
        self.pub = rospy.Publisher('angle', Float64)
        self.port = None
        self.buffer = []

    def open(self, port):
        try:
            self.ser = serial.Serial(port=port, baudrate=38400, bytesize=8,
                            parity='N', stopbits=1, timeout=None, xonxoff=False,
                            rtscts=False, writeTimeout=None, dsrdtr=False)
        except serial.serialutil.SerialException:
            rospy.logfatal('Could not open ' + port)
            raise

        self.port = port
        rospy.loginfo('Opened connection to gyro on port ' + port)

        #select format
        #'R': rate: returns the angular velocity
        #'A': incremental angle: returns the angular difference since last reading
        #'P': integrated angle: returns the absolute angle position (integration is done
        #     on the gyro.
        self.ser.write('P')


    def process_message_data(self):
        msgstr = ''.join([str(b) for b in self.buffer])
        msgs = msgstr.split()
        if len(msgs)==2:
            try:
                value = float(msgs[0])
                status = float(msgs[1])
            except ValueError:
                rospy.logwarn('problem converting to float: %s %s' % (msgs[0], msgs[1]))
            else:
                rospy.logdebug('%f, %f' % (value, status))
                if status==1:
                    self.pub.publish(value)
                else:
                    rospy.logwarn('got a message with invalid status')
        else:
            rospy.logwarn('got an incomplete message [%s]' % msgstr)

    def spin(self):
        '''main loop:
        read from the serial port and store bytes in self.buffer
        messages are delimited by \n and \r
        '''
        while not rospy.is_shutdown():
            b = self.ser.read()
            if b == '\n' or b == '\r':
                if len(self.buffer)>0:
                    self.process_message_data()
                self.buffer = []
            else:
                self.buffer.append(b)


if __name__=='__main__':
    rospy.init_node('gyro_dsp_3000')
    port = rospy.get_param('~port')
    node = GyroNode()
    node.open(port)
    node.spin()
