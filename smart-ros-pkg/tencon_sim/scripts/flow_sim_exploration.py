#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Run a number of flow simulations with different parameters values.'''

import sys, traceback
#import cPickle as pickle
import marshal
import itertools
import os, tarfile, tempfile

sys.path.append('src')
from tencon_sim import *


#------------------------------------------------------------------------------
# simulation parameters
#

params = {}

# time step of the simulation:
params['sim_time_step'] = 0.1

# position of the beginning of the pedestrian crossing on the pedestrian axis
params['ped_crossing_length'] = 2

# position of the beginning of the pedestrian crossing on the vehicle axis
params['ped_crossing_width'] = 1

# Pedestrian velocity
params['ped_vel'] = 1
params['ped_start_pos'] = -20

params['veh_start_pos'] = -100
params['veh_max_vel'] = 3
params['veh_acc'] = 3
params['safety_dist'] = 2

params['infra_fov_dist']= 30

params['lambda_ped'] = 0.1
params['lambda_veh'] = 0.3


#------------------------------------------------------------------------------
# exploration parameters
#

lambda_veh = [0.01, 0.05, 0.1, 0.25, 0.5, 0.75, 1]
lambda_ped = [0.01, 0.05, 0.1, 0.25, 0.35, 0.5]
#lambda_veh = [1]
#lambda_ped = [0.5]

#------------------------------------------------------------------------------
# Simulation

ok = True
logdir = tempfile.mkdtemp()
print 'logging in', logdir
for lv, lp in itertools.product(lambda_veh, lambda_ped):
    print 'Simulating with lambda_veh=%g and lambda_ped=%g' % (lv, lp)
    params['lambda_ped'] = lp
    params['lambda_veh'] = lv
    flow_sim = FlowSim(params)
    while ok:
        try:
            flow_sim.step()
            #print flow_sim.t, flow_sim.nvehs
            if min([flow_sim.nvehs['base'], flow_sim.nvehs['infra']]) > 300:
                break
        except KeyboardInterrupt:
            ok = False
        except:
            print '-'*60
            traceback.print_exc()
            print '-'*60
    if not ok:
        break
    filename = '%s/lv_%03d_lp_%03d.dat' % (logdir, int(lv*100), int(lp*100))
    with open(filename, 'wb') as f:
        marshal.dump(flow_sim.log, f)


tar = tarfile.open("result_logs.tar", "w")
for name in os.listdir(logdir):
        tar.add(os.path.join(logdir,name), name)
tar.close()