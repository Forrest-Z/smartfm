#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Run a flow simulation, taking parameters from the command line. Useful when
distributing several simulations onto several cores/machines.
'''

import sys, os, gzip, tempfile, shutil
from optparse import OptionParser

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



#------------------------------------------------------------------------------
# parse command line arguments
#

parser = OptionParser()
parser.add_option("--lp", dest="lp", type='float',
                  help="sets the arrival rate of pedestrians (lambda_ped)",
                  metavar="VAL")
parser.add_option("--lv", dest="lv", type='float',
                  help="sets the arrival rate of vehicles (lambda_veh)",
                  metavar="VAL")
parser.add_option("--logfile", dest="logfile",
                  help="log into FILE", metavar="FILE")
parser.add_option("--veh_max_vel", dest="veh_max_vel", type='float',
                  help="sets the the velocity of vehicles",
                  metavar="VAL", default=params['veh_max_vel'])

(options, args) = parser.parse_args()
params['lambda_ped'] = options.lp
params['lambda_veh'] = options.lv
params['veh_max_vel'] = options.veh_max_vel


#------------------------------------------------------------------------------
# run the simulation
#

fn = tempfile.mktemp()
if options.logfile.endswith('gz'):
    f = gzip.GzipFile(fn, 'wb')
else:
    f = open(fn, 'wb')
flow_sim = FlowSim(params, f)
while min([flow_sim.nvehs['base'], flow_sim.nvehs['infra']]) < 300:
    flow_sim.step()
f.close()
shutil.move(fn, options.logfile)