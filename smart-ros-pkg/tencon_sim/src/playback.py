#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Script to playback simulation data for visualization purpose.

Reads the data recorded during simulation from an XML file. Play it back as ROS
messages so that it can be visualized in ROS.
'''

import time

import roslib; roslib.load_manifest('tencon_sim')
import rospy

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

from tencon_sim import *



class Player:
    
    def __init__(self, file_or_filename):
        self.record = load_record(file_or_filename)
        self.clockPub = rospy.Publisher('/clock', Clock)
        rospy.set_param('/use_sim_time', True)
        self.basePub = rospy.Publisher('base', PointCloud)
        self.infraPub = rospy.Publisher('infra', PointCloud)
        self.pedPub = rospy.Publisher('pedestrians', PointCloud)
        
    def print_records(self):
        print 'not implemented yet'
        pass

    def prompt_record(self):
        while True:
            n = len(self.record.records)
            print 'There are %i records. What do you want to do?' % n
            print 'Enter a number from 0 to %i to playback the corresponding record,' % (n-1)
            print 'or press "p" to print the records, or Ctrl-D to exit.'
            v = raw_input('your choice: ')
            if v=='p':
                self.print_records()
                continue
            try:
                i = int(v)
                if i<0 or i>=n:
                    print 'Wrong input, try again.'
                    continue
                return i
            except ValueError:
                pass
            print 'Wrong input, try again.'
            
    def ui(self):
        n = self.prompt_record()
        base = self.record.records[n]['base']
        infra = self.record.records[n]['infra']
        self.play_record(base, infra)
        
    def play_record(self, base, infra):
        i = 0
        t = rospy.Time.from_sec(0)
        while True:
            if i>=len(base) and i>=len(infra):
                break
            
            peds = []
            old_t = t
            if i<len(base):
                t = rospy.Time.from_sec(base[i].t)
                peds = base[i].peds
            else:
                t = rospy.Time.from_sec(infra[i].t)
                peds = infra[i].peds
            dt = t - old_t
                
            time.sleep(dt.to_sec())
            self.clockPub.publish(Clock(clock=t))
            msg = PointCloud()
            msg.header.stamp = t
            msg.header.frame_id = 'map'
            for p in peds:
                msg.points.append(Point32(x=0, y=-p['x'], z=0))
            self.pedPub.publish(msg)
            
            for record, pub, offset in ((base,self.basePub, -0.5),(infra,self.infraPub,0.5)):
                msg = PointCloud()
                msg.header.stamp = t
                msg.header.frame_id = 'map'
                if i<len(record):
                    #print pub.name + ':' + record[i].__str__()
                    for v in record[i].vehs:
                        msg.points.append(Point32(x=v['x'], y=offset))
                pub.publish(msg)
            
            i = i+1
            
            
                                

if __name__ == '__main__':
    import sys
    rospy.init_node('playback')
    argv = rospy.myargv(argv=sys.argv)
    player = Player(argv[1])
    while not rospy.is_shutdown():
        player.ui()