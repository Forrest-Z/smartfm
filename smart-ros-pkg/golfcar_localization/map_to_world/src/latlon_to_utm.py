#!/usr/bin/python

import roslib; roslib.load_manifest('map_to_world')
import rospy
from std_msgs.msg import Float64
import map_to_world.srv
from map_to_world.UtmConv import *


def callback(req):
    z, e, n  = map_to_world.UtmConv.latlon_to_utm("WGS-84", req.latitude, req.longitude)
    return map_to_world.srv.LatLonToUtmResponse(z,e,n)


rospy.init_node('latlon_to_utm_server')
s = rospy.Service('latlon_to_utm', map_to_world.srv.LatLonToUtm, callback)
rospy.spin()
