#!/usr/bin/env python
import sys
import os
import yaml
import random
import rosbag

ROSBAG = '/opt/ros/fuerte/bin/rosbag'
BAGFN = 'pedbags/static1.bag'


def get_duration(bagfn):
    b = rosbag.Bag(bagfn)
    info = b._get_yaml_info()
    b.close()
    info = yaml.load(info)
    duration = float(info['duration'])
    return duration

def main(bagfn):
    t = get_duration(bagfn)
    start = random.uniform(0, t-60)
    os.execv(ROSBAG, [ROSBAG, 'play', '-s', str(start), bagfn])


if __name__=='__main__':
    main(BAGFN)

