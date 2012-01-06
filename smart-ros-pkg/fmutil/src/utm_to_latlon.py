#!/usr/bin/python

import roslib; roslib.load_manifest('fmutil')
import rospy
from std_msgs.msg import Float64
import fmutil.srv
from fmutil.UtmConv import *


def callback(req):
    lat, lon = fmutil.UtmConv.utm_to_latlon("WGS-84", req.zone, req.easting, req.northing)
    return fmutil.srv.UtmToLatLonResponse(lat,lon)


rospy.init_node('utm_to_latlon_server')
s = rospy.Service('utm_to_latlon', fmutil.srv.UtmToLatLon, callback)
rospy.spin()
