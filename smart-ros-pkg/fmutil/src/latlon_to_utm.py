#!/usr/bin/python

import roslib; roslib.load_manifest('fmutil')
import rospy
from std_msgs.msg import Float64
import fmutil.srv
from fmutil.UtmConv import *


def callback(req):
    z, e, n  = fmutil.UtmConv.latlon_to_utm("WGS-84", req.latitude, req.longitude)
    return fmutil.srv.LatLonToUtmResponse(z,e,n)


rospy.init_node('latlon_to_utm_server')
s = rospy.Service('latlon_to_utm', fmutil.srv.LatLonToUtm, callback)
rospy.spin()
