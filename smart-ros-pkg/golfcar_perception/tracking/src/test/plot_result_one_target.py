#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import sys


fn = 'datalog.csv'
if len(sys.argv)==2:
    fn = sys.argv[1]

data = np.genfromtxt(fn, delimiter=',', names=True)

for name in ('gt_t', 'p_t'):
    d = data[name]
    for i in range(1, len(d)):
        if d[i] > d[i-1]+np.pi:
            d[i] -= np.pi*2
        elif d[i] < d[i-1]-np.pi:
            d[i] += np.pi*2

for name in ('gt_t', 'p_t', 'gt_w', 'p_w'):
    data[name] = np.degrees(data[name])


plt.figure()
plt.title('Trajectory')
plt.plot(data['gt_x'], data['gt_y'], 'k', label='Ground Truth')
plt.plot(data['obs_x'], data['obs_y'], 'r.', label='Measurements')
plt.plot(data['p_x'], data['p_y'], 'b', label='Estimation')
plt.legend(loc=0)

plt.figure()
plt.title('Hidden variables')
plt.subplot(311)
plt.title('theta')
plt.plot(data['time'], data['gt_t'], 'k', label='Ground Truth')
plt.plot(data['time'], data['p_t'], 'b', label='Estimation')
plt.legend(loc=0)
plt.subplot(312)
plt.title('v')
plt.plot(data['time'], data['gt_v'], 'k', label='Ground Truth')
plt.plot(data['time'], data['p_v'], 'b', label='Estimation')
plt.legend(loc=0)
plt.subplot(313)
plt.title('w')
plt.plot(data['time'], data['gt_w'], 'k', label='Ground Truth')
plt.plot(data['time'], data['p_w'], 'b', label='Estimation')
plt.legend(loc=0)

plt.show()
