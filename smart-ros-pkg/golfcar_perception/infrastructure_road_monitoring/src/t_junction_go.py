#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Assumes two cameras monitoring the T-junction, one looking towards EA (tracksEA),
the other towards SDE (tracksSDE). Outputs a boolean telling the car whether it's
safe to cross the junction (from DCC to EA). This boolean is written in a
database on FMAutonomy.
'''

import roslib; roslib.load_manifest('infrastructure_road_monitoring')
import rospy
from std_msgs.msg import Bool

from vision_motion_detection.msg import Blob, Blobs, Track, Tracks

import urllib, urllib2
from xml.dom import minidom
import sys

import threading


# from http://www.ariel.com.au/a/python-point-int-poly.html
def point_inside_polygon(x, y, poly):
    '''Determines if a point is inside a given polygon or not.
    Polygon is a list of (x,y) pairs.
    '''
    n = len(poly)
    inside = False

    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside


def test_connection():
    try:
        url = rospy.get_param('/infra_url')
    except KeyError:
        rospy.logfatal('rosparam /infra_url must be set first')
        sys.exit(1)

    try:
        f = urllib2.urlopen(url+'/query.php')
        xml = f.read()
        f.close()
    except urllib2.URLError:
        rospy.logfatal('Could not contact the server %s' % url)
        sys.exit(1)

    rospy.loginfo('Using %s as URL' % url)
    return url

class Node:
    def __init__(self):
        self.url = test_connection()
        self.delay = rospy.get_param('delay', 3)
        self.get_poly_defs()
        self.lock = threading.Lock()

        self.subEA = rospy.Subscriber('tracksEA', Tracks, self.EA_Callback)
        self.subSDE = rospy.Subscriber('tracksSDE', Tracks, self.SDE_Callback)
        self.pub = rospy.Publisher('tjunction_go', Bool)

        self.timer = rospy.Timer(rospy.Duration(1), self.timer_Callback)

        self.last_unsafe = rospy.Time.now().to_sec()
        self.last_decision = False
        self.update_db(False)

    def EA_Callback(self, msg):
        for track in msg.tracks:
            cx = track.blob.centroid.x
            cy = track.blob.centroid.y
            vx = track.xvel
            vy = track.yvel
            if point_inside_polygon(cx,cy,self.ea_poly) and vy>0:
                self.last_unsafe = rospy.Time.now().to_sec()
                break
        self.pub_result()

    def SDE_Callback(self, msg):
        for track in msg.tracks:
            cx = track.blob.centroid.x
            cy = track.blob.centroid.y
            vx = track.xvel
            vy = track.yvel
            if point_inside_polygon(cx,cy,self.sde_poly) and vy>0:
                self.last_unsafe = rospy.Time.now().to_sec()
                break
        self.pub_result()

    def get_poly_defs(self):
        self.ea_poly = rospy.get_param('~ea_poly', [])
        self.sde_poly = rospy.get_param('~sde_poly', [])

    def timer_Callback(self, event):
        self.get_poly_defs()

    def update_db(self, status):
        try:
            f = urllib2.urlopen(self.url+'/update.php', urllib.urlencode({'Id':'tjunc', 'Status':status}))
            xml = f.read()
            f.close()
        except urllib2.URLError:
            rospy.logwarn('Could not contact DB server')
        else:
            #print xml
            dom = minidom.parseString(xml)
            nodes = dom.getElementsByTagName('status')
            if len(nodes)>0 and nodes[0].getAttribute('code') == 'err':
                emsg = 'Error while calling update.php'
                if nodes[0].hasAttribute('msg') and nodes[0].getAttribute('msg')!='':
                    emsg += ': ' + nodes[0].getAttribute('msg')
                raise Exception( emsg )

    def pub_result(self):
        self.lock.acquire()
        go = rospy.Time.now().to_sec() > self.last_unsafe + self.delay
        if self.last_decision != go:
            self.update_db(go)
            self.last_decision = go
            if go:
                rospy.loginfo('safe to go')
            else:
                rospy.loginfo('not safe to go')
        self.lock.release()
        self.pub.publish(Bool(go))


if __name__=='__main__':
    rospy.init_node('t_junction_go')
    node = Node()
    rospy.spin()
