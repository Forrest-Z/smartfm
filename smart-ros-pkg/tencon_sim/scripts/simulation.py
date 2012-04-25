#!/usr/bin/env python

from vehicle import *
from record import *
import copy


def base_vehicle(**kwargs):
    return Vehicle(x0=kwargs['veh_start_pos'], v_max=kwargs['veh_max_vel'],
                a=kwargs['veh_acc'], v0=kwargs['veh_max_vel'], 
                sensor=Sensor([kwargs['ped_crossing_start_ped'], 0]), **kwargs)

def infra_vehicle(**kwargs):
    return Vehicle(x0=kwargs['veh_start_pos'], v_max=kwargs['veh_max_vel'],
                    a=kwargs['veh_acc'], v0=kwargs['veh_max_vel'], 
                    sensor=Sensor([-params['infra_fov_dist'], 0]), **kwargs)


def one_vehicle(pedestrians, **kwargs):

    def sim(v):
        peds = copy.deepcopy(pedestrians)
        t = 0
        records = [Record(t, [v], peds, [None])]
        while v.x <= 0:
            s = v.update([], peds)
            for p in peds:
                p.update()
            t = t + kwargs['sim_time_step']
            
            record = Record(t, [v], peds, [s])
            #print record
            records.append(record)
    
        return records
    
    #print 'base'
    base = sim(base_vehicle(**kwargs))
    
    #print 'infra'
    infra = sim(infra_vehicle(**kwargs))
    
    return base[-1].t - infra[-1].t, base, infra
    
    
    

if __name__=='__main__':

    params = {}

    # time step of the simulation:
    params['sim_time_step'] = 0.1

    # position of the begining of the pedestrian crossing on the pedestrian axis
    params['ped_crossing_start_ped'] = -2

    # position of the begining of the pedestrian crossing on the vehicle axis
    params['ped_crossing_start_veh'] = -1

    # Pedestrian velocity
    params['ped_vel'] = 1

    params['veh_start_pos'] = -30
    params['veh_max_vel'] = 3
    params['veh_acc'] = 2

    params['infra_fov_dist']= 30
    

    records = Records()
    for i in range(100):
        peds = []
        for i,p in enumerate( np.random.poisson(lam=0.1, size=100) ):
            if p>0:
                x = params['ped_crossing_start_ped'] - i * params['ped_vel']
                ped = Mobile(x0=x, v_max=params['ped_vel'], v0=params['ped_vel'], **params)
                peds.append(ped)
        #print 'n peds=%d' % len(peds), [p.x for p in peds]
        dt, base, infra = one_vehicle(peds, **params)
        records.append(base, infra)
        
    records.print_dt_stats()
    records.save_as_xml('logfile.xml')