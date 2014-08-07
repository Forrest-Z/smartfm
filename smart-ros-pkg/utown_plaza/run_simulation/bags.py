#!/usr/bin/env python
import math
import rosbag
import argh

GOAL_TOLERANCE = 4
COLLISION_DIST = 1

def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 +(a[1]-b[1])**2)

class Analyzer(object):
    def __init__(self, bagfn):
        self.bagfn = bagfn
        self.peds = []
        self.carpos = None
        self.time_start = None
        self.time_goal = None
        self.collision = False
        self.goal = None

    def __call__(self):
        dispatch = self.dispatch()
        bag = rosbag.Bag(self.bagfn)
        msgs = bag.read_messages(topics=dispatch.keys())

        for topic, msg, time in msgs:
            time = time.to_time()
            dispatch[topic](msg, time)

    def update_carpos(self, msg, time):
        pos = msg.pose.position
        self.carpos = (pos.x, pos.y)

        self.update_time_start(time)
        self.check_collision()
        self.check_reached_goal(time)

    def update_peds(self, msg, time):
        peds = msg.ped_local
        if len(peds) > 0:
            p = peds[0].rob_pose
            self.carpos = (p.x, p.y)

        self.peds = [(a.ped_pose.x, a.ped_pose.y) for a in peds]

        self.update_time_start(time)
        self.check_collision()
        self.check_reached_goal(time)

    def update_goal(self, msg, time):
        p = msg.goal.pose.position
        self.goal = (p.x, p.y)

    def update_time_start(self, time):
        if self.time_start is None:
            self.time_start = time

    def check_collision(self):
        mindist = min(dist(self.carpos, p) for p in self.peds)
        if mindist < 1.0:
            print 'Collision!', self.carpos, self.peds
            self.collision = True
        #print self.carpos, self.peds

    def check_reached_goal(self, time):
        if self.goal and dist(self.carpos, self.goal) < GOAL_TOLERANCE and self.time_goal is None:
            self.time_goal = time

    def dispatch(self):
        return {
            '/car_pose': self.update_carpos,
            '/ped_local_frame_vector': self.update_peds,
            '/ped_path_planner/planner/start_goal': self.update_goal,
        }


def analyze(fn):
    a = Analyzer(fn)
    a()

    timelen = a.time_goal - a.time_start
    collision = a.collision

    print 'Time = ', timelen
    print 'Collision = ', collision

def show(fn, topic):
    b = rosbag.Bag(fn)
    msgs = b.read_messages(topics=[topic])
    for topic, msg, time in msgs:
        print msg

if __name__=='__main__':
    argh.dispatch_commands([show, analyze])

