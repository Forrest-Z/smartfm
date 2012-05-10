#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import cPickle as pickle
import numpy as np
import scipy.stats
import matplotlib.pyplot as plt
import matplotlib.mlab
from mpl_toolkits.mplot3d import Axes3D


class PlotResults:
    
    def __init__(self, filename):
        with open(filename,'r') as f:
            self.results = pickle.load(f)

        self.lps = [r['lambda_ped'] for r in self.results]
        self.lvs = [r['lambda_veh'] for r in self.results]
        self.lp = sorted(set(self.lps))
        self.lv = sorted(set(self.lvs))
        self.dt = [np.mean(r['dt']['base']) - np.mean(r['dt']['infra']) for r in self.results]
        self.t_dt = [scipy.stats.ttest_ind(r['dt']['base'], r['dt']['infra'])[0] for r in self.results]


    def plot_3D_hist(self, zvar, name='', fig=None, subplot=111):
        '''Plots the dt results as a 3D histogram.'''
        if fig is None:
            fig = plt.figure()
        ax = fig.add_subplot(subplot, projection='3d')
        ax.set_title(name)
        
        # position of the bars origin
        xpos, ypos = np.meshgrid(np.arange(len(self.lp))-0.25, np.arange(len(self.lv))-0.25)
        xpos = xpos.flatten()
        ypos = ypos.flatten()
        zpos = np.zeros_like(xpos)
    
        # size of the bars    
        dx = 0.5 * np.ones_like(xpos)
        dy = dx.copy()
        dz = np.asarray(zvar)
        
        ax.bar3d(xpos, ypos, zpos, dx, dy, dz, color='b')
        ax.set_xlabel('lambda_ped')
        ax.set_ylabel('lambda_veh')
        #TODO: put the values for lp and lv
        #ax.set_xticks(range(len(self.lp)))
        #ax.set_yticks(range(len(self.lv)))
        #ax.set_xticklabels([str(l) for l in self.lp])
        #ax.set_yticklabels([str(l) for l in self.lv])
    
        
    def plot_color(self, zvar, name='', fig=None, subplot=111):
        '''Plots the dt results as a color plot.'''
        if fig is None:
            fig = plt.figure()
        ax = fig.add_subplot(subplot)
        ax.set_title(name)
        
        def linspace(v):
            m = min(v)
            M = max(v)
            r = M-m
            return np.linspace(m-0.1*r, M+0.1*r, 500)
        
        xi = linspace(self.lp)
        yi = linspace(self.lv)
        
        zi = matplotlib.mlab.griddata(self.lps, self.lvs, zvar, xi, yi, interp='linear')
        #ax.contour(xi, yi, zi, 15, linewidths=0.5, colors='k')
        contour = ax.contourf(xi, yi, zi, 15, cmap=plt.cm.jet)
        fig.colorbar(contour) # draw colorbar
        ax.plot(self.lps, self.lvs, 'ko', ms=3)
        ax.set_xlim(min(self.lp),max(self.lp))
        ax.set_ylim(min(self.lv),max(self.lv))
        ax.set_xlabel('lambda_ped')
        ax.set_ylabel('lambda_veh')
    
    
    
if __name__=='__main__':
    results = PlotResults('result.pickle.dat')
    
    #results.plot_3D_hist(results.t_dt, 'time gain (t value)')
    #results.plot_3D_hist(results.dt, 'time gain (raw value)')
    
    #results.plot_color(results.t_dt, 'time gain (t value)')
    #results.plot_color(results.dt, 'time gain (raw value)')

    fig = plt.figure()
    results.plot_3D_hist(results.t_dt, 'time gain (t value)', fig, 221)
    results.plot_3D_hist(results.dt, 'time gain (raw value)', fig, 222)    
    results.plot_color(results.t_dt, 'time gain (t value)', fig, 223)
    results.plot_color(results.dt, 'time gain (raw value)', fig, 224)
    
    plt.show()
