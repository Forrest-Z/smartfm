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
        self.data = []
        self.logfiles = []

    def load_from_tar(self, tarfn):
        logdir = tempfile.mkdtemp(prefix='flow_sim_')
        tar = tarfile.open(tarfn)
        print 'Extracting data files to', logdir
        tar.extractall(path=logdir)
        tar.close()
        self.load_from_dir(logdir)

    def load_from_dir(self, logdir):
        self.logdir = logdir

        for f in sorted(os.listdir(logdir)):
            # open the file and extract some useful data
            self.extract_data( os.path.join(self.logdir,f) )

        self.lv = sorted(set([d['lv'] for d in self.data]))
        self.lp = sorted(set([d['lp'] for d in self.data]))
        self.dt = [d['dt'] for d in self.data]
        self.dt_t = [d['dt_t'] for d in self.data]

    def parse_parameter_section(self, f):
        params = {}
        line = f.readline()
        while '<parameters>' not in line:
            line = f.readline()
        line = f.readline()
        while '</parameters>' not in line:
            tokens = line.strip().split('=')
            if len(tokens)!=2:
                print 'Warning: failed parsing parameter line:', line.strip()
            else:
                params[tokens[0]] = float(tokens[1])
            line = f.readline()
        return params

    def extract_data(self, filename):
        '''Goes through the raw data and extracts meaningful data. Entries in
        the list below describe the keys available in the data dictionnary
        created by this function. Entries with a * is a dictionnary with 'base'
        and 'infra' key.
        - lv: the lambda_veh value
        - lp: the lambda_ped value
        - dts*: the transit times of vehicles, which is the time taken for each
          vehicle to transit from the launch point to after the pedestrian
          crossing. It is returned as a list of transit times.
        - dt: difference of mean transit times between infra and base
        - dt_t, dt_p: t and p value of the t-test between infra and base transit
          times
        - stability_p_val*: a stability test based on the p value of the t-test
          between transit times at beginning and end of simulation.
        - is_stable*: a stability test based on whether vehicles at the back of
          the queue are moving or not.
        '''

        m = re.search('lv_(?P<lv>[\d\.]*)_lp_(?P<lp>[\d\.]*).dat', filename)
        if not m.groupdict().has_key('lv') or not m.groupdict().has_key('lp'):
            # only process files whose name matches the pattern
            return

        self.logfiles.append(filename)

        # a dictionnary to hold the extracted data
        data = {}
        data['lv'] = float(m.group('lv'))
        data['lp'] = float(m.group('lp'))
        self.data.append(data)

        # open the file (it's zipped)
        if filename.endswith('gz'):
            f = gzip.GzipFile(filename, 'rb')
        else:
            f = open(filename, 'rb')

        # parse the parameters
        data['params'] = self.parse_parameter_section(f)

        # store transit times here
        data['dts'] = {'base':[], 'infra':[]}

        # maintain a record of entry times (not recorded)
        entry = {'base':{}, 'infra':{}} #{id: t}

        # stability test
        data['is_stable'] = {'base': True, 'infra': True}

        # initialize entry time with t0
        line = f.readline()
        state = parseSimStateStr(line)
        for k in ('base','infra'):
            for vid in state.m[k]:
                entry[k][vid] = state.t
        prev_time = state.t

        # iterate over all lines
        for line in f:
            state = parseSimStateStr(line)
            #print line, state
            for k in ('base','infra'):
                todel = []
                for vid in entry[k]:
                    if vid not in state.m[k]:
                        # vehicle has disappeared
                        data['dts'][k].append(prev_time - entry[k][vid])
                        todel.append(vid)
                for vid in todel:
                    del entry[k][vid]

                count = 0
                for vid in state.m[k]:
                    if data['is_stable'][k] and state.m[k][vid]['pos']<-70 and \
                        state.m[k][vid]['vel']<0.5:
                        count += 1
                        if count > 10:
                            data['is_stable'][k] = False
                    if vid not in entry[k]:
                        # vehicle has appeared
                        entry[k][vid] = state.t
            prev_time = state.t

        f.close()
        data['dt'] = np.mean(data['dts']['base'][30:]) - np.mean(data['dts']['infra'][30:])
        data['dt_t'], data['dt_p'] = scipy.stats.ttest_ind(data['dts']['base'][30:], data['dts']['infra'][30:])

        # compute the stability metric: t-test between first half and
        # second half of the transit time data. If above 0.05, then it is
        # deemed stable.
        data['stability_p_val'] = {}
        for k in ('base', 'infra'):
            n1 = int(len(data['dts'][k]) * 0.1)
            n2 = int(len(data['dts'][k]) * 0.55)
            t, p =  scipy.stats.ttest_ind(data['dts'][k][n1:n2],data['dts'][k][n2:])
            data['stability_p_val'][k] = p

        print '%s: base=%d,%.2f,%.2f, infra=%d,%.2f,%.2f, dt=%2f, dt_t=%.2f, dt_p=%.2f' % \
            ( os.path.basename(filename), \
            len(data['dts']['base'][30:]), np.mean(data['dts']['base'][30:]), data['stability_p_val']['base'], \
            len(data['dts']['infra'][30:]), np.mean(data['dts']['infra'][30:]), data['stability_p_val']['infra'], \
            data['dt'], data['dt_t'], data['dt_p'] )

    def print_data(self):
        print 'lv, lp, base_time, base_stab_p, base_is_stable, infra_time, infra_stab_p, infra_is_stable, t_val, p_val'
        for d in self.data:
            print '%.02f, %.02f, %.02f, %.02f, %d, %.02f, %.02f, %d, %.02f, %.02f' % ( \
                d['lv'], d['lp'], \
                np.mean(d['dts']['base'][30:]), d['stability_p_val']['base'], \
                d['is_stable']['base'], \
                np.mean(d['dts']['infra'][30:]), d['stability_p_val']['infra'], \
                d['is_stable']['infra'], \
                d['dt_t'], d['dt_p'])

    def print_data_table_tex(self):
        print '\\begin{tabular}{|c|%s}' % (len(self.lv) * 'cc|')
        print '\\hline'
        print '\\multirow{2}{*}{$\\lambda_p$} & \\multicolumn{%d}{|c|}{$\\lambda_v$}' % (2*len(self.lv))
        print '\\\\ \\cline{2-%d}' % (2*len(self.lv)+1)
        print ' & ' + ' & '.join(['\\multicolumn{2}{|c|}{%.2f}' % lv for lv in self.lv])
        print '\\\\ \\hline'
        for lp in self.lp:
            s = '%.2f' % lp
            for lv in self.lv:
                for k in ('base','infra'):
                    for d in self.data:
                        if d['lp']==lp and d['lv']==lv:
                            if d['is_stable'][k]:
                                s = s + ' & %d' % int( np.mean(d['dts'][k][30:]) )
                            else:
                                s = s + ' & - '
            print s + '\\\\'
        print '\\hline'
        print '\\end{tabular}'

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

        lps = [d['lp'] for d in self.data]
        lvs = [d['lv'] for d in self.data]

        zi = matplotlib.mlab.griddata(lps, lvs, zvar, xi, yi, interp='linear')
        #ax.contour(xi, yi, zi, 15, linewidths=0.5, colors='k')
        contour = ax.contourf(xi, yi, zi, 15, cmap=plt.cm.jet)
        fig.colorbar(contour) # draw colorbar
        ax.plot(lps, lvs, 'ko', ms=3)
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


    print '-'*79
    results.print_data()
    results.print_data_table_tex()

    #results.plot_3D_hist(results.dt_t, 'time gain (t value)')
    #results.plot_3D_hist(results.dt, 'time gain (raw value)')

    #results.plot_color(results.dt_t, 'time gain (t value)')
    #results.plot_color(results.dt, 'time gain (raw value)')

    fig = plt.figure()
    results.plot_3D_hist(results.dt_t, 'time gain (t value)', fig, 221)
    results.plot_3D_hist(results.dt, 'time gain (raw value)', fig, 222)
    results.plot_color(results.dt_t, 'time gain (t value)', fig, 223)
    results.plot_color(results.dt, 'time gain (raw value)', fig, 224)

    plt.show()
