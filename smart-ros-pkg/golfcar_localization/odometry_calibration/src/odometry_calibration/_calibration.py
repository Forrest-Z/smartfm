# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('odometry_calibration')
import rospy

from ._segment_extraction import *


import numpy as np
import threading

class CalibrationSegment:
    '''Holds information about a segment that is useful for calibration.

    Attributes:
        length: the length of the segment
        orientation_sigma: the standard deviation of orientations on that segment
        rotation: the total rotation of the segment (last orientation - first orientation)
        left: the difference of encoder counts on the left side
        right: the difference of encoder counts on the right side
        alpha: the left correction factor
        size: the size of the wheels
        beta: the distance between the wheels
    '''

    def __init__(self, seg):
        '''Constructed from a Segment'''
        self.poses = seg.poses # this is only needed for visualization at this time
        self.length = seg.length
        self.orientation_range = np.std(seg.orientations)
        self.rotation = seg.orientations[-1] - seg.orientations[0]

    def get_encoder_counts(self, encoder_counts, i0=0):
        left = 0
        right = 0
        try:
            while encoder_counts[i0].stamp < self.poses[0].stamp:
                i0 += 1
            i1 = i0
            while encoder_counts[i1].stamp < self.poses[-1].stamp:
                left += encoder_counts[i1].d_left
                right += encoder_counts[i1].d_right
                i1 += 1
        except IndexError:
            print 'Could not find matching encoder messages'
            raise
        self.left = left
        self.right = right
        return i1

    def common_probas(self):
        self.p_length = 1-np.exp(-self.length/10.0)
        self.p_cv = np.exp(-np.mean([p.cvmax for p in self.poses])/0.1)

    def calibrate_straight(self):
        self.common_probas()
        self.p_orientation = np.exp(-self.orientation_range/np.radians(5))
        self.alpha = self.right / self.left
        self.size = self.length / self.right

    def calibrate_curves(self, alpha, size):
        self.common_probas()
        self.beta = (self.right-self.left*alpha)*size/self.rotation

class CalibrationTool:
    '''Maintains information about extracted segments for optimal use of them all.'''

    def __init__(self):
        self.segment_classifier = SegmentClassifier()
        self.encoder_counts = []
        self.curved_segments = []
        self.straight_segments = []

        self.last_cleanup = None

        # a mutex to protect access to encoder_counts
        self.mutex = threading.Lock()

        # initial values. only needed for debugging (the algorithm does not need
        # to be seeded)
        self.left_correction_factor = 1.0101
        self.wheel_size = 1.3168544

    def append(self, msg):
        with self.mutex:
            self.encoder_counts.append(msg)

    def calibrate_straight(self):
        if len(self.straight_segments)>0 and len(self.segment_classifier.straight_segments)>0:
            self.straight_segments.pop()
        i = 0
        for seg in self.segment_classifier.straight_segments:
            cseg = CalibrationSegment(seg)
            try:
                i = cseg.get_encoder_counts(self.encoder_counts, i)
            except IndexError:
                continue
            cseg.calibrate_straight()
            self.straight_segments.append(cseg)

        if len(self.segment_classifier.straight_segments)>0:
            self.segment_classifier.straight_segments = self.segment_classifier.straight_segments[-1:]

        if len(self.straight_segments)>0:
            p = np.array([np.power(cseg.p_length*cseg.p_orientation*cseg.p_cv, 1.0/3) for cseg in self.straight_segments])
            xa = np.array([cseg.alpha for cseg in self.straight_segments])
            xs = np.array([cseg.size for cseg in self.straight_segments])
            #print 'p_length:', [cseg.p_length for cseg in self.straight_segments]
            #print 'p_orientation:', [cseg.p_orientation for cseg in self.straight_segments]
            #print 'ps:', p
            #print 'alphas:', xa
            #print 'sizes:', xs
            ua = np.average(xa, weights=p)
            us = np.average(xs, weights=p)
            pa = np.mean(np.exp(-np.abs(xa/ua-1))*p)
            ps = np.mean(np.exp(-np.abs(xs/us-1))*p)
            self.left_correction_factor = ua
            self.wheel_size = us
            self.p_alpha = pa
            self.p_size = ps
            print 'left_correction_factor=%f, confidence=%d%%' % (ua, int(pa*100))
            print 'wheel_size=%f, confidence=%d%%' % (us, int(ps*100))

    def calibrate_curves(self):
        if len(self.curved_segments)>0 and len(self.segment_classifier.curved_segments)>0:
            self.curved_segments.pop()
        i = 0
        for seg in self.segment_classifier.curved_segments:
            cseg = CalibrationSegment(seg)
            try:
                i = cseg.get_encoder_counts(self.encoder_counts, i)
            except IndexError:
                continue
            cseg.calibrate_curves(self.left_correction_factor, self.wheel_size)
            self.curved_segments.append(cseg)

        if len(self.segment_classifier.curved_segments)>0:
            self.segment_classifier.curved_segments = self.segment_classifier.curved_segments[-1:]

        if len(self.curved_segments)>0:
            p = np.array([np.power(cseg.p_length*cseg.p_cv,1.0/2) for cseg in self.curved_segments])
            xb = np.array([cseg.beta for cseg in self.curved_segments])
            #print 'p_length:', p
            #print 'betas:', xb
            ub = np.sum(p*xb)/p.sum()
            pb = np.mean(np.exp(-np.abs(xb/ub-1))*p)
            self.dist_btw_wheels = ub
            print 'dist_btw_wheels=%f, confidence=%d%%' % (ub, int(pb*100))

    def remove_encoder_counts(self):
        with self.mutex:
            if len(self.encoder_counts)==0:
                return

            if self.last_cleanup is None:
                self.last_cleanup = rospy.Time.now()
                return

            ts = [self.last_cleanup]
            self.last_cleanup = rospy.Time.now()
            if len(self.segment_classifier.amcl_poses)>0:
                ts.append( self.segment_classifier.amcl_poses[0].stamp )
            if len(self.segment_classifier.straight_segments)>0:
                ts.append( self.segment_classifier.straight_segments[0].poses[0].stamp )
            if len(self.segment_classifier.curved_segments)>0:
                ts.append( self.segment_classifier.curved_segments[0].poses[0].stamp )
            t = min(ts)

            i = len(self.encoder_counts)-1
            while i>=0 and self.encoder_counts[i].stamp>=t:
                i -= 10
            if i>0:
                #print 'removing', i, 'encoder_counts values:', t.to_sec()
                self.encoder_counts = self.encoder_counts[i:]

    def calibrate(self):
        self.segment_classifier.classify()
        self.calibrate_straight()
        self.calibrate_curves()
        self.remove_encoder_counts()