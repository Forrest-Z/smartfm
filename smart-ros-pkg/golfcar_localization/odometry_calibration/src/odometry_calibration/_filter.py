import numpy as np

class FilterState(object):
    '''A class to represent a filter state: the value and its uncertainty.'''
    def __init__(self, x, p=0):
        self.x = x
        self.p = p

class GenericFilter(object):
    '''An abstract base class for filters.'''
    def update(self, z, r=0):
        '''Update the filter with a new observation z and the associated noise r.'''
        raise NotImplementedError()

    def get_state(self):
        '''Get the current state of the filter as a FilterState object.'''
        raise NotImplementedError()

class WeightedAverageFilter(GenericFilter):
    '''The measure is the results of the weighted average of all measures.

    This code could be optimized (i.e. keep only the average and the weight, and
    not the whole history of values), but then it would not read as a weighted
    average as explicitly.

    This is actually equivalent to the KalmanFilter, with sigma_q=0, and
    p = 1/sum(1/r)
    '''
    def __init__(self, prior=None):
        self.measures = []
        if prior is not None:
            self.measures.append(prior)

    def update(self, z, r=1):
        # the weight is inversely proportional to the variance
        self.measures.append(FilterState(z,1/r))

    def get_state(self):
        if len(self.measures)==0:
            raise RuntimeError('Filter not initialized')
        s = np.array([ [m.x, m.p] for m in self.measures])
        return FilterState(np.average(s[:,0], weights=s[:,1]), 1.0/np.sum(s[:,1]))

class KalmanFilter(GenericFilter):
    '''A Kalman Filter to estimate a constant state (A=1), with no control (B=0)
    and direct observation of the state (H=1)
    '''
    def __init__(self, prior, q=0):
        '''Initialization of the filter with the prior and the process noise.'''
        self.state = prior
        self.sigma_q = q
        #print 'Filter initialized with (%f,%f)' % (prior.x, prior.p)

    def update(self, z, r):
        old_state = self.state
        p_p = self.state.p + self.sigma_q  # projected covariance
        k   = p_p / (p_p + r)              # kalman gain
        self.state.x = (1.0-k)*self.state.x + k*z
        self.state.p = (1.0-k) * p_p
        #print 'Updating with (%f,%f): (%f,%f) --> (%f,%f). k=%f' % (z, r, old_state.x, old_state.p, self.state.x, self.state.p, k)

    def get_state(self):
        return FilterState(self.state.x, self.state.p)