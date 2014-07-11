#!/usr/bin/env python
import sys
import os
import yaml
import random
import rosbag

ROSBAG = '/opt/ros/fuerte/bin/rosbag'
#BAGFN = '3394_new.bag'
BAGFN = 'pedestrians1.bag'

def get_duration(bagfn):
    b = rosbag.Bag(bagfn)
    info = b._get_yaml_info()
    b.close()
    info = yaml.load(info)
    duration = float(info['duration'])
    return duration

def get_t0(bagfn):
    b = rosbag.Bag(bagfn)
    read_msg = b.read_messages()
    t0 = next(read_msg)[2]
    return t0.to_time()

def gen_pedindex(bagfn):
    t0 = get_t0(bagfn)
    b = rosbag.Bag(bagfn)
    read_msg = b.read_messages(topics=['/golfcart/ped_data_assoc'])
    pedindex = []
    for topic, msg, time in read_msg:
        time = time.to_time()
        if len(msg.pd_vector) > 8:
            t = time - t0
            pedindex.append(t)
    return pedindex

def main(bagfn):
    #t = get_duration(bagfn)
    #start = t / 8.0 + random.uniform(0, 4)
    #start = random.uniform(0, t)
    pedindex = gen_pedindex(bagfn)
    #start = random.choice(pedindex)
    start = 439.553
    os.execv(ROSBAG, [ROSBAG, 'play', '-s', str(start), bagfn])


if __name__=='__main__':
    main(BAGFN)
    #main(sys.argv[1])
    #print get_duration(sys.argv[1])
