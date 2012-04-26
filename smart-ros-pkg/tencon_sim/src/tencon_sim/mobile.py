import math
import copy


def bound(x, m1, m2=None):
    '''returns x, bounded between m1 and m2. If m2 is not given, use -m1.'''
    if m2 is None:
        m2 = -m1
    m = min([m1,m2])
    M = max([m1,m2])
    return min([max([x, m]), M])


class Mobile(object):
    '''A mobile moves along one axis. Velocity can be controlled, possibly with
    some acceleration constraints.'''

    def __init__(self, x0, v_max, a=None, v0=0, sim_time_step=0.1, **kwargs):
        '''Constructor:
        @param x0 initial position
        @param v_max maximum speed
        @param a acceleration (optional). If not given, assumes instant
        (infinite) acceleration/deceleration.
        @param v0 initial velocity (defaults to 0)
        '''
        #print 'Mobile:', kwargs
        self.x = x0
        self.v_max = v_max
        self.a = a

        self.v = v0 #current speed
        self.v_target = v0 #desired speed

        self.sim_time_step = sim_time_step

    def throttle(self, v_target):
        '''Sets the desired speed.'''
        self.v_target = bound(v_target, 0, self.v_max)
        if self.a is None:
            self.v = self.v_target
            
    def acc(self, a):
        '''brake, accelerate or cruise.
        @param a: 0: cruise, -1: brake, 1: accelerate
        '''
        if a==-1:
            self.throttle(0)
        elif a==1:
            self.throttle(self.v_max)
        else:
            self.throttle(self.v)

    def update(self):
        '''Updates the velocity and position.'''
        if self.a is None:
            self.x = self.x + self.v * self.sim_time_step
        else:
            v = self.v
            if self.v < self.v_target:
                self.v = min([self.v + self.a * self.sim_time_step, self.v_target, self.v_max])
            elif self.v > self.v_target:
                self.v = max([self.v - self.a * self.sim_time_step, self.v_target, 0])
            self.x = self.x + (self.v+v)/2 * self.sim_time_step
        #print self.v, self.x

    def time_to_stop(self):
        '''Returns the time needed to stop.'''
        if self.a is None:
            return 0

        #v(t) = v0 - a * t
        #v(T) = 0 <==> T = v0 / a
        return self.v / self.a

    def dist_to_stop(self):
        '''Returns the distance needed to stop, given the current speed.'''
        if self.a is None:
            return 0

        #v(t) = v0 - a * t
        #x(t) = x0 + v0 * t - a * t^2 / 2
        #
        #v(T) = 0
        #T = v0 / a
        #x(T) - x0 = v0^2 / a - v0^2 / 2a = v0^2 / 2a

        #m = copy.deepcopy(self)
        #m.throttle(0)
        #while round(m.v,4) > 0:
            #m.update()
        #return abs(m.x-self.x)

        return math.pow(self.v, 2) / (self.a * 2)

    def must_decelerate_to_stop_at_pos(self, xstop):
        '''Whether the mobile must start decelerating to stop at the given
        position.
        '''
        #m = copy.deepcopy(self)
        #m.throttle(0)
        #while round(m.v,4) > 0:
            #m.update()
        #return m.x >= xstop
        return (self.x + self.dist_to_stop() >= xstop)

    def time_to_pos(self, y):
        '''Returns the time needed, at the current speed, to reach the given
        position.'''
        #x(t) = x0 + v0 * t
        #x(T) = y <=> T = (y-x0) / v0
        if self.v == 0:
            raise RuntimeError('Mobile is not moving')
        return (y-self.x) / self.v

    def future_pos(self, dt, a=0):
        '''Returns the position of the mobile some time into the future.
        If an acceleration command is given, take it into account, otherwise
        assume that the same speed will be maintained.
        @param dt the duration of the future to investigate
        @param a the acceleration command: -1 to brake, 0 to cruise, 1 to accelerate.
        '''
        if a==0:
            return self.x + self.v * dt
        
        if a==-1:
            if self.time_to_stop() < dt:
                return self.x + self.dist_to_stop()
            return self.x + self.v * dt - self.a * pow(dt,2) / 2

        t_acc = 0
        if self.a!=0:
            t_acc = (self.v_max-self.v)/self.a
        
        if t_acc > dt:
            return self.x + self.v * dt + self.a * pow(dt,2) / 2
        
        d_acc = 0
        if self.a != 0:
            d_acc = (pow(self.v_max,2)-pow(self.v,2)) / (2 * self.a )
        
        d_v_max = self.v_max * (dt-t_acc)
        return self.x + d_acc + d_v_max