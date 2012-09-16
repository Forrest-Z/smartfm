#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Modifications and improvements by Brice Rebsamen (brice.rebsamen@gmail.com), 2012

import roslib
roslib.load_manifest('diagnostic_common_diagnostics')
import rospy
import diagnostic_updater as DIAG

import sys
import socket
from subprocess import Popen, PIPE
import time
import re
import optparse
import threading
import copy

def ntp_diag(st, host, off, error_offset):
    try:
        p = Popen(["ntpdate", "-q", host], stdout=PIPE, stdin=PIPE, stderr=PIPE)
        res = p.wait()
        (o,e) = p.communicate()
    except OSError, (errno, msg):
        if errno == 4:
            return None #ctrl-c interrupt
        else:
            raise
    if (res == 0):
        measured_offset = float(re.search("offset (.*),", o).group(1))*1000000

        st.level = DIAG.DiagnosticStatus.OK
        st.message = "OK"
        st.values = [ DIAG.KeyValue("Offset (us)", str(measured_offset)),
                        DIAG.KeyValue("Offset tolerance (us)", str(off)),
                        DIAG.KeyValue("Offset tolerance (us) for Error", str(error_offset))]

        if (abs(measured_offset) > off):
            st.level = DIAG.DiagnosticStatus.WARN
            st.message = "NTP Offset Too High"
        if (abs(measured_offset) > error_offset):
            st.level = DIAG.DiagnosticStatus.ERROR
            st.message = "NTP Offset Too High"

    else:
        st.level = DIAG.DiagnosticStatus.ERROR
        st.message = "Error Running ntpdate. Returned %d" % res
        st.values = [ DIAG.KeyValue("Offset (us)", "N/A"),
                        DIAG.KeyValue("Offset tolerance (us)", str(off)),
                        DIAG.KeyValue("Offset tolerance (us) for Error", str(error_offset)),
                        DIAG.KeyValue("Output", o),
                        DIAG.KeyValue("Errors", e) ]

    return st

def create_timed_message(stat, stamp):
    st = copy.deepcopy(stat)
    st.values.append( DIAG.KeyValue("Measurement time",str(stamp.to_sec())) )
    elapsed = rospy.get_rostime()-stamp
    st.values.append( DIAG.KeyValue("Elapsed since last measurement (s)", str(elapsed.to_sec())) )
    return st


class NTPMonitorNode:
    def __init__(self, argv=sys.argv):
        self.parse_options(argv)
        self.pub = rospy.Publisher("/diagnostics", DIAG.DiagnosticArray)
        self.mutex = threading.Lock()

        self.stat = None
        self.stat_stamp = None
        self.self_stat = None
        self.self_stat_stamp = None

        self.diag_timer_callback(None)
        self.pub_timer = rospy.Timer(rospy.Duration(0.5), self.pub_timer_callback)
        self.diag_timer = rospy.Timer(rospy.Duration(60), self.diag_timer_callback)

    def diag_timer_callback(self, dummy):
        stat = DIAG.DiagnosticStatus()
        stat.level = DIAG.DiagnosticStatus.OK
        stat.name = "NTP offset from "+ self.diag_hostname + " to " + self.ntp_hostname
        stat.message = "OK"
        stat.hardware_id = self.hostname
        stat.values = []
        stat = ntp_diag(stat, self.ntp_hostname, self.offset, self.error_offset)
        with self.mutex:
            self.stat = stat
            self.stat_stamp = rospy.get_rostime()

        if self.do_self_test:
            self_stat = DIAG.DiagnosticStatus()
            self_stat.level = DIAG.DiagnosticStatus.OK
            self_stat.name = "NTP self-offset for "+ self.diag_hostname
            self_stat.message = "OK"
            self_stat.hardware_id = self.hostname
            self_stat.values = []
            self_stat = ntp_diag(self_stat, self.hostname, self.self_offset, self.error_offset)
            with self.mutex:
                self.self_stat = self_stat
                self.self_stat_stamp = rospy.get_rostime()

    def pub_timer_callback(self, dummy):
        msg = DIAG.DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()
        with self.mutex:
            if self.stat is not None:
                msg.status.append( create_timed_message(self.stat, self.stat_stamp) )
            if self.do_self_test and self.self_stat is not None:
                msg.status.append( create_timed_message(self.self_stat, self.self_stat_stamp) )
        self.pub.publish(msg)

    def parse_options(self, argv):
        parser = optparse.OptionParser(usage="usage: ntp_monitor ntp-hostname []")
        parser.add_option("--offset-tolerance", dest="offset_tol",
                        action="store", default=500,
                        help="Offset from NTP host", metavar="OFFSET-TOL")
        parser.add_option("--error-offset-tolerance", dest="error_offset_tol",
                        action="store", default=5000000,
                        help="Offset from NTP host. Above this is error", metavar="OFFSET-TOL")
        parser.add_option("--self_offset-tolerance", dest="self_offset_tol",
                        action="store", default=500,
                        help="Offset from self", metavar="SELF_OFFSET-TOL")
        parser.add_option("--diag-hostname", dest="diag_hostname",
                        help="Computer name in diagnostics output (ex: 'c1')",
                        metavar="DIAG_HOSTNAME",
                        action="store", default=None)
        parser.add_option("--no-self-test", dest="do_self_test",
                        help="Disable self test",
                        action="store_false", default=True)
        options, args = parser.parse_args(argv)

        if (len(args) != 2):
            parser.error("Invalid arguments. Must have HOSTNAME [args]. %s" % args)

        try:
            self.offset = int(options.offset_tol)
            self.self_offset = int(options.self_offset_tol)
            self.error_offset = int(options.error_offset_tol)
        except:
            parser.error("Offsets must be numbers")

        self.diag_hostname = options.diag_hostname
        self.ntp_hostname = args[1]
        self.hostname = socket.gethostname()
        if self.diag_hostname is None:
            self.diag_hostname = self.hostname
        self.do_self_test = options.do_self_test


if __name__ == "__main__":
    rospy.init_node('ntp_monitor', anonymous=True)
    try:
        node = NTPMonitorNode( rospy.myargv() )
        rospy.spin()
    except KeyboardInterrupt: pass
    except SystemExit: pass
    except:
        import traceback
        traceback.print_exc()

