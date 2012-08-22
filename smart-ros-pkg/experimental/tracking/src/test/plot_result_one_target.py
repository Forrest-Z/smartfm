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


fig = plt.figure()

ax = plt.subplot(121)
ax.set_title('Trajectory')
ax.plot(data['gt_x'], data['gt_y'], 'k', label='Ground Truth')
ax.plot(data['obs_x'], data['obs_y'], 'r.', label='Measurements')
ax.plot(data['p_x'], data['p_y'], 'b', label='Estimation')
ax.legend(loc=0)

ax = plt.subplot(322)
ax.set_title('theta')
ax.plot(data['time'], data['gt_t'], 'k', label='Ground Truth')
ax.plot(data['time'], data['p_t'], 'b', label='Estimation')
ax.legend(loc=0)

ax = plt.subplot(324)
ax.set_title('v')
ax.plot(data['time'], data['gt_v'], 'k', label='Ground Truth')
ax.plot(data['time'], data['p_v'], 'b', label='Estimation')
ax.legend(loc=0)

ax = plt.subplot(326)
ax.set_title('w')
ax.plot(data['time'], data['gt_w'], 'k', label='Ground Truth')
ax.plot(data['time'], data['p_w'], 'b', label='Estimation')
ax.legend(loc=0)

plt.tight_layout()
plt.show()
