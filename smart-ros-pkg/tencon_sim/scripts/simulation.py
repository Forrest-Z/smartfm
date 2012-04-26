#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('tencon_sim')
from tencon_sim import *
import copy


def base_vehicle(x0, **kwargs):
    return BaseVehicle(x0, v_max=kwargs['veh_max_vel'],
                a=kwargs['veh_acc'], v0=kwargs['veh_max_vel'], 
                **kwargs)

def infra_vehicle(x0, **kwargs):
    sensor = Sensor([-params['infra_fov_dist'], kwargs['ped_crossing_length']])
    return InfraVehicle(x0, v_max=kwargs['veh_max_vel'],
                    a=kwargs['veh_acc'], v0=kwargs['veh_max_vel'], 
                    sensor=sensor, **kwargs)


def one_vehicle(px, **kwargs):

    def sim(v):
        peds = [Mobile(x0=x, v_max=kwargs['ped_vel'], v0=kwargs['ped_vel'], **kwargs) for x in px]
        t = 0
        records = [Record(t, [v], peds, [None])]
        while v.x <= 4:
            a, msg = v.update([v], peds)
            for p in peds:
                p.update()
            t = t + kwargs['sim_time_step']
            
            record = Record(t, [v], peds, [msg])
            #print record
            records.append(record)
    
        return records
    
    #print 'base'
    base = sim(base_vehicle(kwargs['veh_start_pos'], **kwargs))
    
    #print 'infra'
    infra = sim(infra_vehicle(kwargs['veh_start_pos'], **kwargs))
    
    return base, infra



def many_vehicles(vx, px, **kwargs):

    def sim(vehs):
        peds = [Mobile(x0=x, v_max=kwargs['ped_vel'], v0=kwargs['ped_vel'], **kwargs) for x in px]
        t = 0
        records = [Record(t, vehs, peds, [None for v in vehs])]
        while vehs[-1].x <= 4:
            msgs = []
            for v in vehs:
                a, msg = v.update(vehs, peds)
                msgs.append(msg)
            for p in peds:
                p.update()
            t = t + kwargs['sim_time_step']
            
            record = Record(t, vehs, peds, msgs)
            #print record
            records.append(record)
    
        return records
    
    #print 'base'
    base = sim([base_vehicle(x, **kwargs) for x in vx])
    
    #print 'infra'
    infra = sim([infra_vehicle(x, **kwargs) for x in vx])
    
    return base, infra
    
    

if __name__=='__main__':

    params = {}

    # time step of the simulation:
    params['sim_time_step'] = 0.1

    # position of the beginning of the pedestrian crossing on the pedestrian axis
    params['ped_crossing_length'] = 2

    # position of the beginning of the pedestrian crossing on the vehicle axis
    params['ped_crossing_width'] = 1

    # Pedestrian velocity
    params['ped_vel'] = 1

    params['veh_start_pos'] = -10
    params['veh_max_vel'] = 3
    params['veh_acc'] = 3

    params['infra_fov_dist']= 30
    

    records = Records()
    for i in range(10):
        px = []
        for i,p in enumerate( np.random.poisson(lam=0.1, size=1000) ):
            if p>0:
                x = -params['ped_crossing_length'] - i * params['ped_vel']
                px.append(x)
        vx = []
        for i,p in enumerate( np.random.poisson(lam=1, size=1000) ):
            if p>0:
                x = -params['ped_crossing_width'] - i * params['veh_max_vel']
                vx.append(x)
                if len(vx)==100:
                    break
        #base, infra = one_vehicle(px, **params)
        base, infra = many_vehicles(vx, px, **params)
        records.append(base, infra)
        
    records.print_dt_stats()
    records.save_as_xml('logfile.xml')