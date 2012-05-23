#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, os.path, tarfile, tempfile, re, gzip

import numpy as np
import scipy.stats
import matplotlib.pyplot as plt
import matplotlib.mlab
from mpl_toolkits.mplot3d import Axes3D


class SimState:
    '''Holds the state of the simulation at a given instant: time step, pos and
    vel of each mobile.
    '''
    def __init__(self):
        self.t = 0
        self.m = {'base':{}, 'infra':{}, 'peds':{}}

    def __str__(self):
        s = ['t='+str(round(self.t,2))]
        for k in ['peds', 'base', 'infra']:
            tmp = []
            for mid, mval in self.m[k].items():
                tmp.append( '(' + ', '.join([str(mid)] + [str(round(mval[kk],2)) for kk in ['pos', 'vel']]) + ')' )
                s.append( k + '=[' + ', '.join(tmp) + ']' )
        return ', '.join(s)


def parseSimStateStr(s):
    '''Parses a string with the following format and returns the corresponding
    SimState object:
    "t=1.2, peds=[(2, -3, 1), (3, -5, 1)], base=[], infra=[]"
    '''
    state = SimState()
    m = re.search('t=([\-\d\.]*)', s)
    state.t = float(m.group(1))
    for k in ('peds', 'base', 'infra'):
        m = re.search(k+'=\[([^\]]*)\]', s)
        if m.group(1)!='':
            mm = re.findall('\(([\-\d\.]*), ([\-\d\.]*), ([\-\d\.]*)\)', m.group(1))
            for mmm in mm:
                state.m[k][int(mmm[0])] = {'pos': float(mmm[1]), 'vel': float(mmm[2])}
    return state



class PlotResults:

    def __init__(self):
        self.lps = []
        self.lvs = []
        self.dts = []

    def load_from_tar(self, tarfn):
        logdir = tempfile.mkdtemp()
        tar = tarfile.open(tarfn)
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
        # open the file (it's zipped)
        f = gzip.GzipFile(filename,'r')

        # store transit times here
        dts = {'base':[], 'infra':[]}

        # maintain a record of entry times
        entry = {'base':{}, 'infra':{}} #{id: t}

        # initialize entry time with t0
        line = f.readline()
        state = parseSimStateStr(line)
        for k in ('base','infra'):
            for vid in state.m[k]:
                entry[k][vid] = state.t
        prev_time = state.t

        # iterate other lines
        for line in f:
            state = parseSimStateStr(line)
            #print line, state
            for k in ('base','infra'):
                todel = []
                for vid in entry[k]:
                    if vid not in state.m[k]:
                        # vehicle has disappeared
                        dts[k].append(prev_time - entry[k][vid])
                        todel.append(vid)
                for vid in todel:
                    del entry[k][vid]

                for vid in state.m[k]:
                    if vid not in entry[k]:
                        # vehicle has appeared
                        entry[k][vid] = state.t
            prev_time = state.t

        f.close()
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
