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
    
    def __init__(self, x0, v_max, a, sensor, ped_crossing_start_ped, ped_crossing_start_veh, ped_vel, v0=0, sim_time_step=0.1, safety_dist=1, **kwargs):
        super(Vehicle, self).__init__(x0, v_max, a, v0, sim_time_step, **kwargs)
        self.safety_dist = safety_dist
        self.sensor = sensor
        self.ped_crossing_start_ped = ped_crossing_start_ped
        self.ped_crossing_start_veh = ped_crossing_start_veh
        self.ped_vel = ped_vel
        
    def _check_one_ped(self, p):

        if self.x > 0:
            # The vehicle has passed the crossing point already
            pass

        elif p.x > 0:
            # the pedestrian has crossed already
            pass

        elif p.x < self.ped_crossing_start_ped:
            # the pedestrian has not entered the crossing yet
            t1 = p.time_to_pos(self.ped_crossing_start_ped)
            t2 = p.time_to_pos(0)
            x1 = self.future_pos(t1)
            x2 = self.future_pos(t2)
            if x2 < self.ped_crossing_start_veh:
                # when the pedestrian will have finished crossing, the vehicle
                # will still not have reached the crossing
                pass
            elif x1 > 0:
                # when the pedestrian will have started crossing, the vehicle
                # will have passed the crossing already
                pass
            else:
                # the pedestrian and the vehicle will be on the crossing at the
                # same time. The vehicle must break.
                self.throttle(0)
                return 'Stopping because the sensor has spotted a pedestrian'

        else:
            # the pedestrian is on the crossing already
            t = p.time_to_pos(0)
            x = self.future_pos(t)
            if x < self.ped_crossing_start_veh:
                # when the pedestrian will have finished crossing, the vehicle
                # will still not have reached the crossing
                pass
            else:
                # the pedestrian and the vehicle will be on the crossing at the
                # same time. The vehicle must break.
                self.throttle(0)
                return 'Stopping because there is a pedestrian on the crossing'
        
        return None

    def check_peds(self, pedestrians):
        if self.x > self.ped_crossing_start_veh + 0.1:
            return None
        
        # check all pedestrians
        for p in filter(self.sensor.visible, pedestrians):
            s = self._check_one_ped(p)
            if s:
                return s
            
        # Since the sensor may have a small sensing range, we must slow down when
        # getting close, so as to be able to stop if a pedestrian was to step in
        if round(self.v,3) > 0:
            virtual_ped = Mobile(x0=self.sensor.fov[0], v_max=self.ped_vel, v0=self.ped_vel)
            s = self._check_one_ped(virtual_ped)
            if s:
                self.throttle(0)
                return s + ' (virtual ped)'
        
        return None
    
    
    def check_vehicles(self, all_vehicles):
        # The vehicle must stop if there is another vehicle in front
        vs = [v for v in all_vehicles if v.x > self.x]
        if len(vs)!=0:
            vs.sort(key=lambda v: v.x)
            d = vs[0].x + vs[0].dist_to_stop() + self.safety_dist
            if self.must_decelerate_to_stop_at_pos(d):
                self.throttle(0)
                return 'Stopping because there is a vehicle %.1fm in front' % d
            
        return None

        
    def update(self, all_vehicles, pedestrians):
        super(Vehicle, self).update()

        # by default accelerate to full speed
        self.throttle(self.v_max)
        
        s = self.check_vehicles(all_vehicles)
        if not s:
            s = self.check_peds(pedestrians)
        return s
