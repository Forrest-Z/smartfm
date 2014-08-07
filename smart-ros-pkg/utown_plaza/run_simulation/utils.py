#!/usr/bin/env python
import time
import platform
import argh
import librato

librato_api = librato.connect('haoyu@nus.edu.sg', 'f92df24c4d0efba6525a1c4d2b577ab2b8bb9c28fc9e3794bde4d8841d3629b4')

def report(name, v=1):
    machine = platform.node()
    librato_api.submit(name, v, source=machine)

def watchdog(t, *_):
    time.sleep(float(t))
    report('timeout')

def count(*_):
    report('sims')

if __name__=='__main__':
    argh.dispatch_commands([watchdog, count])

