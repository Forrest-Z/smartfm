#!/usr/bin/python

import roslib; roslib.load_manifest('map_to_world')
import rospy
from std_msgs.msg import Float64
import map_to_world.srv
from map_to_world.UtmConv import *


def callback(req):
    lat, lon = map_to_world.UtmConv.utm_to_latlon("WGS-84", req.zone, req.easting, req.northing)
    return map_to_world.srv.UtmToLatLonResponse(lat,lon)


rospy.init_node('utm_to_latlon_server')
s = rospy.Service('utm_to_latlon', map_to_world.srv.UtmToLatLon, callback)
rospy.spin()
