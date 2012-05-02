#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Run a number of flow simulations with different parameters values.'''

import sys
import cPickle as pickle

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
# exploration parameters, and result container
#

lambda_veh = [0.01, 0.05, 0.1, 0.25, 0.5, 0.75, 1]
lambda_ped = [0.01, 0.05, 0.1, 0.25, 0.35, 0.5]

# Each cell holds the results of one 
# parameter configuration as a dictionary.
results = []
for lv in lambda_veh:
    for lp in lambda_ped:
        results.append({'lambda_veh':lv, 'lambda_ped': lp})




#------------------------------------------------------------------------------
# Simulation

ok = True
for r in results:
    print 'Simulating with lambda_veh=%g and lambda_ped=%g' % (r['lambda_veh'], r['lambda_ped'])
    params['lambda_ped'] = r['lambda_ped']
    params['lambda_veh'] = r['lambda_veh']
    flow_sim = FlowSim(params)
    while ok:
        try:
            flow_sim.step()
            ndtb = len(flow_sim.dt['base'])
            ndti = len(flow_sim.dt['infra'])
            if min([ndtb, ndti]) > 300:
                break
        except KeyboardInterrupt:
            ok = False
        except:
            print 'Caught exception', sys.exc_info()[0]
    if not ok:
        break
    r['dt'] = flow_sim.dt
    
with open('result.pickle.dat', 'w') as f:
    pickle.dump(results, f)
