'''Reads the log file written by the calibration node, and plots the data.'''

import matplotlib.pyplot as plt
import numpy as np
import sys

data = np.genfromtxt(sys.argv[1], delimiter=',', names=True, dtype=None)

def plot(name):
    idx = (data['name']==name).nonzero()[0]
    I = np.arange(len(idx))

    # separate points with a low variance
    obsp = np.sort(data['obsp'][idx])
    thr = obsp[len(obsp)/4]
    lowp = I[data['obsp'][idx] <= thr]
    highp = I[data['obsp'][idx] > thr]

    plt.figure(name)
    plt.subplot(211)
    plt.plot(lowp, data['obsx'][idx][lowp], 'g.', label='measurement (low variance)')
    plt.plot(highp, data['obsx'][idx][highp], 'r.', label='measurement (high variance)')
    #plt.plot(data['obsx'][idx], 'r.', label='measurement')
    plt.plot(data['valx'][idx], 'b', label='estimation')
    mu = np.average(data['obsx'][idx], weights=1.0/data['obsp'][idx])
    plt.plot(mu+np.zeros_like(I), 'k', label='averagd obs')
    plt.legend(loc=0)
    plt.title('value')

    plt.subplot(212)
    plt.plot(lowp, data['obsp'][idx][lowp], 'g.', label='measurement (low variance)')
    plt.plot(highp, data['obsp'][idx][highp], 'r.', label='measurement (high variance)')
    #plt.plot(data['obsp'][idx], 'r.', label='measurement')
    plt.plot(data['valp'][idx], 'b', label='estimation')
    plt.plot(thr+np.zeros_like(I), 'k', label='variance threshold')
    plt.legend(loc=0)
    plt.title('variance')

plot('size')
plot('alpha')
plot('beta')
plt.show()