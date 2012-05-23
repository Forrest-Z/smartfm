# -*- coding: utf-8 -*-
'''This is code to simulate a flow of vehicles and a flow of pedestrians
interacting at the pedestrian crossing.

Actually we are simulating 2 parallel but independent flows of vehicles: one
flow of BaseVehicle vehicles and one flow of InfraVehicle vehicles. They are
simulated together but the 2 simulations are totally independent. They just share
the same pedestrians and the same initial conditions.

The simulation is collecting results along the way:
- the time it takes each vehicle to complete the course
'''

import math
import numpy as np
import copy

from vehicle import *


class FlowSim:

    def __init__(self, params):
        # experimental parameters
        self.params = copy.deepcopy(params)

        # the current time
        self.t = 0

        # the list of vehicles in each condition
        self.vehs = {'base': [], 'infra': []}

        # the list of pedestrians
        self.peds = []

        # number of vehicles that have crossed the pedestrian crossing
        self.nvehs = {'base': 0, 'infra': 0}

        # data log
        self.log = {'t': [], 'base': [], 'infra': [], 'peds': []}
        self.last_log_t = 0

        # Populate the stream of pedestrians
        N = self.params['lambda_ped'] / self.params['ped_vel'] * abs(self.params['ped_start_pos'])
        while len(self.peds) < int(math.floor(N)):
            self._update_peds()

    def _update_peds(self):
        '''Updates the stream of pedestrians: updates their position and velocity
        and maybe add a new one.'''
        for p in self.peds:
            p.update()

        lam = self.params['lambda_ped'] * self.params['sim_time_step']
        if np.random.poisson(lam):
            self.do_log = True
            p = self.peds.append( Mobile(x0=self.params['ped_start_pos'],
                                    v_max=self.params['ped_vel'],
                                    v0=self.params['ped_vel'], **self.params) )

    def _update_vehs(self):
        '''Updates the streams of vehicles: updates their position and velocity
        and maybe add a new one.'''
        for k in ('base', 'infra'):
            for v in self.vehs[k]:
                v.update(self.vehs[k], self.peds)

        lam = self.params['lambda_veh'] * self.params['sim_time_step']
        if np.random.poisson(lam):
            for k in ('base', 'infra'):
                #if self.vehs[k]!=[] and self.vehs[k][-1].x < self.params['veh_start_pos'] + 2:
                #    raise RuntimeError("Queue full")
                if k=='base':
                    v = BaseVehicle(x0 = self.params['veh_start_pos'],
                                    v_max = self.params['veh_max_vel'],
                                    a = self.params['veh_acc'],
                                    v0 = self.params['veh_max_vel'],
                                    **self.params)
                else:
                    v = InfraVehicle(x0 = self.params['veh_start_pos'],
                                    v_max = self.params['veh_max_vel'],
                                    a = self.params['veh_acc'],
                                    v0 = self.params['veh_max_vel'],
                                    sensor = Sensor([-self.params['infra_fov_dist'],
                                            self.params['ped_crossing_length']]),
                                    **self.params)
                self.vehs[k].append(v)
                self.do_log = True

    def _remove_mobiles(self, remove=True):
        ''' Checks all mobiles for removal condition. If remove is False, then
        only reports whether a mobile needs to be removed.
        '''
        removing = False
        i = 0
        while i < len(self.peds):
            if self.peds[i].x > self.params['ped_crossing_length'] + 1:
                if not remove:
                    return True
                else:
                    removing = True
                    self.peds.remove(self.peds[i])
            else:
                i += 1

        for k in ('base', 'infra'):
            i = 0
            while i<len(self.vehs[k]):
                if self.vehs[k][i].x > 5:
                    if not remove:
                        return True
                    else:
                        removing = True
                        self.vehs[k].remove(self.vehs[k][i])
                        self.nvehs[k] += 1
                else:
                    i += 1

        return removing

    def step(self):
        self.do_log = False
        self._update_peds()
        self._update_vehs()

        if self.do_log or self.t - self.last_log_t >= 1 or self._remove_mobiles(False):
            self.log['t'].append(self.t)
            self.log['peds'].append({})
            for p in self.peds:
                self.log['peds'][-1][p.id] = {'pos': p.x, 'vel': p.v}
            for k in ('base', 'infra'):
                self.log[k].append({})
                for v in self.vehs[k]:
                    self.log[k][-1][v.id] = {'pos': v.x, 'vel': v.v}
            self.last_log_t = self.t

        self._remove_mobiles(True)
        self.t += self.params['sim_time_step']
