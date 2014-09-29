from collections import namedtuple
import numpy as np
from scipy.stats import sem

PedState = namedtuple('PedState', 'id pos vel goal dist2car infront')
CarState = namedtuple('CarState', 'pos vel')
State = namedtuple('State', 'car peds')

SEP = '===================='

def parse_log(r):
    while True:
        line = next(r)
        assert line == SEP
        yield parse_trial(r)

def parse_trial(r):
    dispatch = {
            'state': parse_state,
            'final_state': parse_state,
            'act': parse_val(int),
            'reward': parse_val(float),
            'total_reward': parse_val(float),
            'goal_reached': parse_val(int),
            'collision': parse_val(int),
            }
    keys = tuple(s + '=' for s in dispatch)
    for line in r:
        if line == SEP:
            break
        if line.startswith(keys):
            for k, f in dispatch.items():
                if line.startswith(k):
                    val = f(line, r)
                    yield k, val

def parse_state(line, r):
    carpos = None
    carvel = None
    peds = []
    for line in r:
        if line == ']]':
            break
        if line.startswith('car'):
            vals = line.split('=')[1]
            vals = vals.split('/')
            carpos = eval(vals[0])
            carvel = float(vals[2])
        elif line.startswith('ped'):
            vals = line.split('=')[1]
            vals = vals.split('/')
            pedid = int(vals[0])
            pos = eval(vals[1])
            vel = float(vals[2])
            goal = int(vals[3])
            dist2car = float(vals[4])
            infront = int(vals[5])
            ped = PedState(pedid, pos, vel, goal, dist2car, infront)
            peds.append(ped)
    s = State(CarState(carpos, carvel), peds)
    return s

def parse_val(tp):
    def parse(line, r):
        val = line.split('=')[1]
        val = tp(val)
        return val
    return parse

def reader(fn):
    f = open(fn)
    for line in f.readlines():
        yield line.strip()

Result = namedtuple('Result', 'timelen collision goal_reached mindist goals')

def analyze_trial(t):
    steps = 0
    collision = False
    goal_reached = False
    goal = None
    curr_state = None
    for k, v in t:
        if k == 'state':
            steps += 1
            mindist = min(p.dist2car for p in v.peds)
            goal = v.peds[0].goal
            curr_state = v
        elif k == 'collision':
            #if curr_state.car.vel > 0.2:
            collision = True
        elif k == 'goal_reached':
            goal_reached = True
    timelen = float(steps) / 3 # 3 Hz
    r = Result(timelen, collision, goal_reached, mindist, goal)
    return r

def analyze(fn):
    r = reader(fn)
    results = [analyze_trial(t) for t in parse_log(r)]
    #not_reach_goal = [r for r in results if not r.goal_reached and not r.collision]
    #print not_reach_goal
    collisions = [r for r in results if r.collision]
    success = [r for r in results if r.goal_reached]

    total = len(results)
    
    timelens = [r.timelen for r in success]
    avg_time = np.mean(timelens)
    err_time = sem(timelens)

    collision_rate = len(collisions) / float(len(results))
    err_rate =  2 * np.sqrt(collision_rate*(1-collision_rate)/total)

    print 'total = ', total
    #print 'not_reach_goal = ', len(not_reach_goal) / float(total)
    print 'collision_rate = ', collision_rate , '+/-', err_rate
    print 'avg_time = ', avg_time, '+/-', err_time


if __name__=='__main__':
    import sys
    analyze(sys.argv[1])

