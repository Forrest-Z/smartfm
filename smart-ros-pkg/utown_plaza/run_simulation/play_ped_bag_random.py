#!/usr/bin/env python
import sys
import os
import yaml
import random
import rosbag

ROSBAG = '/opt/ros/fuerte/bin/rosbag'
#BAGFN = '3394_new.bag'
BAGFN = 'pedbags/static1.bag'


def get_duration(bagfn):
    b = rosbag.Bag(bagfn)
    info = b._get_yaml_info()
    b.close()
    info = yaml.load(info)
    duration = float(info['duration'])
    return duration

def main(bagfn):
    N = 16
    t = get_duration(bagfn)
    #start = t / 8.0 + random.uniform(-2, 6)

    try:
        i = int(open('/tmp/bagi').read())
    except:
        i = 0
    i += 1
    i = i % N

    open('/tmp/bagi', 'w').write(str(i))

    #start = t/8.0
    #start = random.uniform(0, t)
    #start = 620
    start = i * t/ float(N)

    os.execv(ROSBAG, [ROSBAG, 'play', '-s', str(start), bagfn])


if __name__=='__main__':
    main(BAGFN)
    #main(sys.argv[1])
    #print get_duration(sys.argv[1])
