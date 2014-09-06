#!/usr/bin/env python
import math
import rosbag
import argh

GOAL_TOLERANCE = 4
COLLISION_DIST = 0.5

def dist(a, b):
    return math.sqrt((a[0]-b[0])**2 +(a[1]-b[1])**2)

class Analyzer(object):
    def __init__(self, bagfn):
        self.bagfn = bagfn
        self.peds = []
        self.peds_first_appear = {}
        self.carpos = None
        self.time_start = None
        self.time_goal = None
        self.collision = False
        self.max_collision_speed = 0
        self.goal = None
        self.speed = 0

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

    def update_speed(self, msg, time):
        speed = msg.twist.twist.linear.x
        self.speed = speed

    def update_peds(self, msg, time):
        peds = msg.ped_local
        if len(peds) > 0:
            p = peds[0].rob_pose
            self.carpos = (p.x, p.y)

        for a in peds:
            if a.ped_id not in self.peds_first_appear:
                self.peds_first_appear[a.ped_id] = time

        #self.peds = [(a.ped_pose.x, a.ped_pose.y) for a in peds]
        # exclude peds suddenly appear
        self.peds = [(a.ped_pose.x, a.ped_pose.y) for a in peds
                        if time - self.peds_first_appear[a.ped_id] > 5]

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
        if not self.peds:
            return

        mindist = min(dist(self.carpos, p) for p in self.peds)
        #print mindist, self.speed
        if mindist < COLLISION_DIST:
            #print 'Collision!', self.carpos, self.peds, self.speed
            self.collision = True
            self.max_collision_speed = max(self.max_collision_speed, self.speed)
        #print self.carpos, self.peds

    def check_reached_goal(self, time):
        if self.goal and dist(self.carpos, self.goal) < GOAL_TOLERANCE and self.time_goal is None:
            self.time_goal = time

    def dispatch(self):
        return {
            '/car_pose': self.update_carpos,
            '/ped_local_frame_vector': self.update_peds,
            '/ped_path_planner/planner/start_goal': self.update_goal,
            '/odom': self.update_speed,
        }


def analyze_file(fn):
    a = Analyzer(fn)
    a()

    if a.time_goal is None:
        timelen = 200
    else:
        timelen = a.time_goal - a.time_start

    print 'bag fn =', fn
    print 'Time = ', timelen
    print 'Collision = ', a.collision
    print 'Max collision speed = ', a.max_collision_speed

    result = {
            'max_collision_speed': a.max_collision_speed,
            'timelen':  timelen,
            }
    return result

def analyze(*fns):
    for f in fns:
        analyze_file(f)

def animate_beliefs(beliefs, rainbows):
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    fig = plt.figure()
    plt.yticks([])
    plt.xlim((0, 1))

    ims = []
    for b, r in zip(beliefs, rainbows):
        im = plt.barh(-np.arange(len(b)), b, color=r)
        ims.append(im)
    im_ani = animation.ArtistAnimation(fig, ims, interval=1000/3, blit=True)
    im_ani.save('belief.mp4')
    #plt.show()

def show_belief(fn, topic='/golfcart/pomdp_beliefs0'):
    """
    Animate belief for the old belief marker message format.
    """
    import numpy as np
    b = rosbag.Bag(fn)
    msgs = b.read_messages(topics=[topic])
    beliefs = []
    rainbows = []
    for topic, msg, time in msgs:
        time = time.to_time()
        markers = msg.markers
        belief = np.array([m.scale.y for m in markers])
        belief = belief / np.sum(belief)
        beliefs.append(belief)

        rainbow = [(m.color.r, m.color.g, m.color.b) for m in markers]
        rainbows.append(rainbow)
    animate_beliefs(beliefs, rainbows)


def show(fn, topic):
    b = rosbag.Bag(fn)
    msgs = b.read_messages(topics=[topic])
    for topic, msg, time in msgs:
        print msg

if __name__=='__main__':
    argh.dispatch_commands([show, show_belief, analyze])

