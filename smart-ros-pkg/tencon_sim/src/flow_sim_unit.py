#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Run a flow simulation, taking parameters from the command line. Useful when
distributing several simulations onto several cores/machines.
'''

import sys, os, gzip, tempfile
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

(options, args) = parser.parse_args()
params['lambda_ped'] = options.lp
params['lambda_veh'] = options.lv


#------------------------------------------------------------------------------
# run the simulation
#

fn = tempfile.mktemp()
f = gzip.GzipFile(fn, 'w')
flow_sim = FlowSim(params, f)
while min([flow_sim.nvehs['base'], flow_sim.nvehs['infra']]) < 300:
    flow_sim.step()
f.close()
os.rename(fn, options.logfile)