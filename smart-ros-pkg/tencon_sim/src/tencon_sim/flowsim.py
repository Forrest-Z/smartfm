# -*- coding: utf-8 -*-

import math
import numpy as np
import scipy.stats
import copy

from vehicle import *


class FlowSim:

    def __init__(self, params):
        self.t = 0
        self.vehs = {'base': [], 'infra': []}
        self.dt = {'base': [], 'infra': []}
        self.peds = []
        self.params = copy.deepcopy(params)
        
        N = self.params['lambda_ped'] / self.params['ped_vel'] * abs(self.params['ped_start_pos'])
        while len(self.peds) < int(math.floor(N)):
            self.update_peds()

    def base_vehicle(self, x0):
        d = self.params['ped_crossing_length']
        return BaseVehicle(x0=x0, v_max=self.params['veh_max_vel'],
                    a=self.params['veh_acc'], v0=self.params['veh_max_vel'], 
                    **self.params)
    
    def infra_vehicle(self, x0):
        sensor = Sensor([-self.params['infra_fov_dist'], 
                         self.params['ped_crossing_length']])
        return InfraVehicle(x0=x0, v_max=self.params['veh_max_vel'],
                        a=self.params['veh_acc'], v0=self.params['veh_max_vel'], 
                        sensor=sensor, **self.params)
    
    def update_peds(self):
        for p in self.peds: 
            p.update()
    
        i=0
        while i < len(self.peds):
            if self.peds[i].x > self.params['ped_crossing_length'] + 1:
                self.peds.remove(self.peds[i])
            else:
                i += 1
    
        lam = self.params['lambda_ped'] * self.params['sim_time_step']
        if np.random.poisson(lam):
            p = self.peds.append( Mobile(x0=self.params['ped_start_pos'], 
                                    v_max=self.params['ped_vel'], 
                                    v0=self.params['ped_vel'], **self.params) )
    
    def update_vehs(self):
        for k in ('base', 'infra'):
            for v in self.vehs[k]:
                v.update(self.vehs[k], self.peds)
            i = 0
            while i<len(self.vehs[k]):
                if self.vehs[k][i].x > 5:
                    self.dt[k].append(self.t - self.vehs[k][i].entry_time)
                    self.vehs[k].remove(self.vehs[k][i])
                else:
                    i += 1
    
        lam = self.params['lambda_veh'] * self.params['sim_time_step']
        if np.random.poisson(lam):
            for k, f in (('base', self.base_vehicle), ('infra', self.infra_vehicle)):
                if self.vehs[k]!=[] and self.vehs[k][-1].x < self.params['veh_start_pos'] - 2:
                    raise RuntimeError("Queue full")
                v = f(self.params['veh_start_pos'])
                v.entry_time = self.t
                self.vehs[k].append(v)
                
    def step(self):
        self.update_peds()
        self.update_vehs()
        self.t += self.params['sim_time_step']

    def print_dt_stats(self):
        if self.dt['infra']==[] or self.dt['base']==[]:
            raise RuntimeError()
        for k in ('base', 'infra'):
            l = self.dt[k][-100:]
            print '%s: t=%g (n=%i, sigma=%g)' % (k, np.average(l), len(l), np.std(l))
        tp = scipy.stats.ttest_ind(self.dt['infra'][-100:], self.dt['base'][-100:])
        print 't=%g, p=%g' % tp
