#!/usr/bin/env python
import sys
import os
import yaml
import random
import rosbag

ROSBAG = '/opt/ros/fuerte/bin/rosbag'
BAGFN = '3394_new.bag'

def get_duration(bagfn):
    b = rosbag.Bag(bagfn)
    info = b._get_yaml_info()
    b.close()
    info = yaml.load(info)
    duration = float(info['duration'])
    return duration

def main(bagfn):
    t = get_duration(bagfn)
    #start = t / 8.0 +2
    start = random.uniform(0, t)

    os.execv(ROSBAG, [ROSBAG, 'play', '-s', str(start), bagfn])


if __name__=='__main__':
    main(BAGFN)
    #main(sys.argv[1])
    #print get_duration(sys.argv[1])
