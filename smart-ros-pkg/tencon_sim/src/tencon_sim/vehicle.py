from mobile import Mobile


class Sensor:
    '''A sensor to sense pedestrians.'''

    def __init__(self, fov):
        '''Constructor:
        @param fov is the field of view, an array with 2 numbers. The sensor 
        can see pedestrians between the 2 values of the array.
        '''
        assert isinstance(fov, (list,tuple))
        self.fov = [min(fov), max(fov)]

    def visible(self, p):
        '''Returns whether the sensor can see the pedestrian p.'''
        return p.x >= self.fov[0] and p.x <= self.fov[1]


class Vehicle(Mobile):
    '''A class to represent vehicles.
    
    Vehicles must stop / slow down if they are getting too close to the one
    in front.
    '''
    def __init__(self, x0, v_max, a, v0=0, safety_dist=1, **kwargs):
        #print 'Vehicle:', kwargs
        super(Vehicle, self).__init__(x0, v_max, a, v0, **kwargs)
        self.safety_dist = safety_dist
        
    def update(self, all_vehicles):
        super(Vehicle, self).update()
        
        # The vehicle must stop if there is another vehicle in front
        i = all_vehicles.index(self)
        if i>0:
            # not first vehicle
            v = all_vehicles[i-1]
            d = v.x + v.dist_to_stop() - self.safety_dist
            if self.must_decelerate_to_stop_at_pos(d):
                self.acc(-1)
                return -1, 'Stopping because there is a vehicle %.1fm in front' % d            
            
        self.acc(1)
        return 1, 'Accelerating because no vehicle in front'
    

class InfraVehicle(Vehicle):
    def __init__(self, x0, v_max, a, ped_crossing_length, ped_crossing_width, sensor, ped_vel, v0=0, **kwargs):
        #print 'InfraVehicle:', kwargs
        super(InfraVehicle, self).__init__(x0, v_max, a, v0, **kwargs)
        self.sensor = sensor
        self.ped_vel = ped_vel
        self.ped_crossing_length = ped_crossing_length
        self.ped_crossing_width = ped_crossing_width
        
    def _acc_one_ped(self, p):
        if p.x > self.ped_crossing_length:
            return 1, 'The pedestrian has crossed already.'

        elif p.x < -self.ped_crossing_length:
            m = 'The pedestrian has not entered the crossing yet. '
            t1 = p.time_to_pos(-self.ped_crossing_length)
            t2 = p.time_to_pos(self.ped_crossing_length)
            if self.future_pos(t1, a=1) > self.ped_crossing_width:
                m += 'Accelerating to cross before the pedestrian.'
                return 1, m
            if self.future_pos(t2, a=1) < -self.ped_crossing_width:
                m += 'Accelerating because still very far.'
                return 1, m
            if self.future_pos(t2, a=0) < -self.ped_crossing_width:
                m += 'Cruising because still very far.'
                return 0, m
            if self.x + self.dist_to_stop() < -self.ped_crossing_width:
                m += 'Cruising because still very far.'
                return 0, m
            m += 'Stopping to let the pedestrian cross.'
            return -1, m

        else:
            m = 'The pedestrian is on the crossing already. '
            t = p.time_to_pos(self.ped_crossing_length)                
            if self.future_pos(t, a=1) < -self.ped_crossing_width:
                m += 'Accelerating because still very far.'
                return 1, m
            if self.future_pos(t, a=0) < -self.ped_crossing_width:
                m += 'Cruising to let the pedestrian cross.'
                return 0, m
            if self.x + self.dist_to_stop() < -self.ped_crossing_width:
                m += 'Cruising because still very far.'
                return 0, m
            m += 'Stopping to let the pedestrian cross.'
            return -1, m
        
        raise RuntimeError('logic error: this point should never be reached')


    def acc_peds(self, pedestrians):
        if self.x > 0:
            return 1, 'The vehicle has passed the pedestrian crossing already.'
        
        if not self.must_decelerate_to_stop_at_pos(-self.ped_crossing_width):
            return 1, 'Accelerating because still very far.'
        
#        if self.force_through:
#            return 1, 'Accelerating (forcing through)'
#        
#        if self.x > 0 and round(self.v,3)==0:
#            self.force_through = True

        profile = None        
        for i, p in enumerate(pedestrians):
            if not self.sensor.visible(p):
                continue
            a, m = self._acc_one_ped(p)
            m += ' (pedestrian %i).' % i
            if a == -1:
                return a, m
            elif profile is None or a<profile[0]:
                profile = a, m
            
        # Now we consider the limited range of the sensor: if the range is small
        # the vehicle must slow down so as to be able to stop on time when a
        # pedestrian is detected
        assert self.sensor.fov[0] <= -self.ped_crossing_length
        dxp = -self.ped_crossing_length - self.sensor.fov[0]
        dt = dxp / self.ped_vel
        dxv = self.a * pow(dt,2) / 2
        if self.x < -self.ped_crossing_width-dxv:
            if self.must_decelerate_to_stop_at_pos(-self.ped_crossing_width):
                return -1, 'Stopping for the virtual pedestrian.'
            elif profile is None or profile[0]<1:
                profile = 1, 'Accelerating to cross before a pedestrian appears'
        
        if profile is None:
            return 1, 'No pedestrians on the way'
        else:
            return profile
        
    def update(self, all_vehicles, pedestrians):
        a0, m0 = super(InfraVehicle, self).update(all_vehicles)
        if a0>=0:
            a1, m1 = self.acc_peds(pedestrians)
            if a1<a0:
                self.acc(a1)
                return a1, m1
        return a0, m0


class BaseVehicle(Vehicle):
    def __init__(self, x0, v_max, a, ped_crossing_length, ped_crossing_width, v0=0, **kwargs):
        #print 'BaseVehicle:', kwargs
        super(BaseVehicle, self).__init__(x0, v_max, a, v0, **kwargs)
        self.state = 'before'
        self.ped_crossing_length = ped_crossing_length
        self.ped_crossing_width = ped_crossing_width


    def acc_peds(self, pedestrians):
        if self.state == 'before':
            if self.x >= -self.ped_crossing_width-0.1 and round(self.v, 3)==0:
                self.state = 'stopped'
                return -1, 'Stopped at ped crossing, looking'

            if self.must_decelerate_to_stop_at_pos(-self.ped_crossing_width):
                return -1, 'Stopping at ped crossing'
            
            return 1, 'Accelerating because still very far'
        
        if self.state == 'stopped':
            for p in pedestrians:
                if p.x > -self.ped_crossing_length and p.x < self.ped_crossing_length:
                    return -1, 'Stopped at ped crossing, there is a pedestrian'
            self.state = 'passing'
            return 1, 'Stopped at ped crossing, there is no pedestrian'
        
        if self.state == 'passing':
            if self.x > self.ped_crossing_width:
                self.state = 'passed'
            return 1, 'Passing through the crossing'
        
        if self.state == 'passed':
            return 1, 'Passed the crossing'
        
        raise RuntimeError("Logic error: should not reach this point")
    
    def update(self, all_vehicles, pedestrians):
        a0, m0 = super(BaseVehicle, self).update(all_vehicles)
        if a0>=0:
            a1, m1 = self.acc_peds(pedestrians)
            if a1<a0:
                self.acc(a1)
                return a1, m1
        return a0, m0



class Pedestrian(Mobile):
    '''A class to represent a pedestrian.
    
    Before crossing the road, i.e. at the beginning of the pedestrian
    crossing, a pedestrian checks if the road is clear. It stops if a vehicle is
    on the crossing.
    '''
    
    def __init__(self, x0, v_max, ped_crossing_length, ped_crossing_width, v0=0, sim_time_step=0.1, safety_dist=1, **kwargs):
        super(Pedestrian, self).__init__(x0, v_max, a=0, v0=v0, sim_time_step=sim_time_step, **kwargs)
        self.ped_crossing_length = ped_crossing_length
        self.ped_crossing_width = ped_crossing_width

        
    def update(self, vehicles):
        super(Pedestrian, self).update()
        
        vel = self.v_max
        if self.x > -self.ped_crossing_length-1 and self.x < -self.ped_crossing_length:
            for v in vehicles:
                if v.x > -self.ped_crossing_width and v.x < self.ped_crossing_width:
                    vel = 0
                    break
        self.throttle(vel)