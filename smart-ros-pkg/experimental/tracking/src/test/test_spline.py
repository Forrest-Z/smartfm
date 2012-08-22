#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate


#-----------------------------------------------------------------------------------
# test 1: using scipy.interpolate
#
# this does not work as it is meant to interpolate a mathematic function where
# the x axis is monotonically increasing

#waypoints = np.array([[0,0], [20,50], [30,50], [50,25], [100,50], [50,100], [25,75], [15,85], [50,150], [150,100]])
#x = waypoints[:,0]
#y = waypoints[:,1]

#f_interp = interpolate.UnivariateSpline(x, y, s=0)

##plt.plot(x, yinterp, label='Interpolated')
#plt.plot(x, y, '-bo', label='Original')
#plt.legend()
#plt.show()


#-----------------------------------------------------------------------------------
# test 2:
# taken from http://en.literateprograms.org/Cubic_spline_(Python)
# The curve is drawn as a set of segments, where each segment is defined by 4
# control points. The first and last control points are the extremities of the
# segment, and the second and third points are attractors to curve the segment.
#
# The result is printed to stdout as PostScript. To view it, redirect to a .ps file,
# or pipe into GS: ./test_spline.py | gs -dTTYPAUSE -_
#
# The main problem with this code is that the control points have to be defined
# too explicitly, and that there is no smooth transition between segments

N = 2**8 # max number of points used to render each segment of the spline
s = np.arange(N+1)/float(N)
z = s[::-1]
b = np.transpose(np.array((z*z*z, 3*z*z*s, 3*z*s*s, s*s*s)))
def cubicspline(c,t): return np.dot(b[t],c)


# control points.
# each line is a segment defined by 4 control points
# the segment always passes through the first and last control point
# if the segment is made of 2 double control points, then its a straight line between them
# The 2 middle points are used to control the curvature

# This draws a capital L.
cs = np.reshape(np.array([
        533,179, 533,179, 483,0, 483,0,
        483,0, 483,0, 28,0, 28,0,
        28,0, 28,0, 28,24, 28,24,
        28,24, 28,24, 81,24, 81,24,
        81,24, 109,24, 117,53, 117,76,
        117,76, 117,76, 117,600, 117,600,
        117,600, 117,618, 99,637, 81,637,
        81,637, 81,637, 28,637, 28,637,
        28,637, 28,637, 28,661, 28,661,
        28,661, 28,661, 259,661, 259,661,
        259,661, 259,661, 259,637, 259,637,
        259,637, 259,637, 206,637, 206,637,
        206,637, 187,637, 172,618, 172,600,
        172,600, 172,600, 172,73, 172,73,
        172,73, 172,50, 184,23, 210,23,
        210,23, 210,23, 330,23, 330,23,
        330,23, 416,23, 495,99, 518,179,
        518,179, 518,179, 533,179, 533,179,
        ]),(-1,4,2))

# This is a stripped down version for experimentation purpose
#cs = np.reshape(np.array([
        #533,179, 483,0, 28,0, 28,24,
        #81,24, 117,76, 117,600, 81,637,
        #28,637, 28,661, 259,661, 259,661,
        #259,661, 259,661, 259,637, 206,637,
        #172,600, 172,73, 210,23, 330,23
        #]),(-1,4,2))

print "%!"
print "20 200 translate"
print ".8 .8 scale"
print "%d %d moveto" % tuple(cs[0][0])

# Number of points to render for each segment (controls the resolution)
# With M=2 the segments are rendered as straight lines between the first and last
# point of the segment
M = N
assert M>=2 and M<=N
T = [int(t) for t in np.linspace(0,N,M)]

for (x,y) in [cubicspline(c,t) for c in cs for t in T]:
        print x,y,"lineto"

print "stroke showpage"