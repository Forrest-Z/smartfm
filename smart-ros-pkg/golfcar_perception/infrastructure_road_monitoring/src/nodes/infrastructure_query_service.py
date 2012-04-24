#!/usr/bin/env python
# -*- coding: utf-8 -*-

''' Queries the database server for the status of an infrastructure sensor. '''

import roslib; roslib.load_manifest('infrastructure_road_monitoring')
import rospy

from std_msgs.msg import Bool, String
from infrastructure_road_monitoring.srv import InfrastructureQuery, InfrastructureQueryResponse

import urllib, urllib2
from xml.dom import minidom

url = ''


def parse(xml):
    ''' A helper function that parses the returned XML and checks for errors.
    Returns an error message if an error occurred, the clear_to_go boolean
    otherwise.
    '''

    #print xml
    dom = minidom.parseString(xml)
    nodes = dom.getElementsByTagName('status')
    if len(nodes)==0 or nodes[0].getAttribute('code') == 'err':
        emsg = 'Error occurred while calling query.php'
        if len(nodes)>0 and nodes[0].hasAttribute('msg') and nodes[0].getAttribute('msg')!='':
            emsg += ': ' + nodes[0].getAttribute('msg')
        return emsg
    else:
        nodes = dom.getElementsByTagName('infrastructure')
        if len(nodes)==0:
            return 'Error occurred while calling query.php: no match'
        elif nodes[0].hasAttribute('status'):
            s = nodes[0].getAttribute('status')
            if s=='1':
                return True
            else:
                return False
        else:
            return 'Error occurred while calling query.php: no status attribute'


def query(req):
    try:
        f = urllib2.urlopen(url+'/query.php', urllib.urlencode({'Id':req.id}))
        xml = f.read()
        f.close()
    except urllib2.URLError:
        rospy.logwarn('Could not contact DB server')
        return None
    else:
        res = parse(xml)
        if isinstance(res, basestring):
            rospy.logwarn(res)
            return None
        else:
            return InfrastructureQueryResponse(res)
    return None


rospy.init_node('infrastructure_query_service')
url = rospy.get_param('url', 'http://fmautonomy.no-ip.info/infrastructure')
s = rospy.Service('infrastructure_query', InfrastructureQuery, query)
rospy.spin()
