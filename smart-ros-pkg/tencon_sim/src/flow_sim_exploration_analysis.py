#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import marshal
import os, os.path, tarfile, tempfile, re

import numpy as np
import scipy.stats
import matplotlib.pyplot as plt
import matplotlib.mlab
from mpl_toolkits.mplot3d import Axes3D


class PlotResults:

    def __init__(self):
        self.lps = []
        self.lvs = []
        self.dts = []

    def load_from_tar(self, tarfile):
        logdir = tempfile.mkdtemp()
        tar = tarfile.open(tarfile)
        tar.extractall(path=logdir)
        tar.close()
        self.load_from_dir(logdir)

    def load_from_dir(self, logdir):
        self.logdir = logdir
        self.logfiles = sorted(os.listdir(logdir))
        for f in self.logfiles:
            m = re.search('lv_(?P<lv>[\d\.]*)_lp_(?P<lp>[\d\.]*).dat', f)
            try:
                self.lvs.append( float(m.group('lv')) )
                self.lps.append( float(m.group('lp')) )
            except KeyError:
                print 'Skipping', f
            else:
                self.dts.append( self.extract_transit_times(self.logdir+'/'+f) )
        self.lp = sorted(set(self.lps))
        self.lv = sorted(set(self.lvs))

        self.dt = [np.mean(dt['base']) - np.mean(dt['infra']) for dt in self.dts]
        self.t_dt = [scipy.stats.ttest_ind(dt['base'], dt['infra'])[0] for dt in self.dts]

    def extract_transit_times(self, filename):
        '''Goes through the raw data and extracts the transit times of all vehicles
        in both the base and infra condition.
        '''
        with open(filename, 'rb') as f:
            data = marshal.load(f)

        # For each base/infra condition
        dts = {'base':[], 'infra':[]}
        for k in ('base','infra'):

            # maintain a record of entry times
            entry = {} #{id: t}
            # initialize with t0
            for vid in data[k][0]:
                entry[vid] = data['t'][0]

            # for each row, compare with previous row to find out
            # vehicle ids that have been added and those that have
            # been removed. Sets are useful for that.
            prev_set = set(data[k][0])
            for i in xrange(1, len(data['t'])):
                new_set = set(data[k][i])
                #print 'prev_set:', prev_set
                #print 'new_set:', new_set
                #print 'new vehicles (new_set-prev_set):', (new_set-prev_set)
                #print 'removed vehicles (prev_set-new_set):', (prev_set-new_set)
                for vid in new_set-prev_set:
                    # vehicles that have appeared
                    entry[vid] = data['t'][i]
                for vid in prev_set-new_set:
                    # vehicles that have disappeared
                    dts[k].append(data['t'][i-1] - entry[vid])
                    del entry[vid]
                prev_set = new_set

        print '%s: base=%d,%f, infra=%d,%f, dt=%f' % \
            ( os.path.basename(filename), \
            len(dts['base']), np.mean(dts['base']), \
            len(dts['infra']), np.mean(dts['infra']), \
            np.mean(dts['base'])-np.mean(dts['infra']) )
        return dts

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
    results = PlotResults()
    if os.path.isdir(sys.argv[1]):
        results.load_from_dir(sys.argv[1])
    elif os.path.isfile(sys.argv[1]):
        results.load_from_tar(sys.argv[1])


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
