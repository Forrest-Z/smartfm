# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('odometry_calibration')
import rospy

from ._segment_extraction import *
from ._filter import FilterState, KalmanFilter


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

        alpha: the left correction factor and its variance (as a FilterState object)
        size: the size of the wheels and its variance (as a FilterState object)
        beta: the distance between the wheels and its variance (as a FilterState object)
    '''

    def __init__(self, seg):
        '''Constructed from a Segment'''
        self.poses = seg.poses
        self.length = seg.length
        self.orientation_range = np.std(seg.orientations)
        self.rotation = seg.orientations[-1] - seg.orientations[0]
        self.amcl_var = np.mean([p.cvmax for p in seg.poses])

    def get_encoder_counts(self, encoder_counts, i0=0):
        '''Searches the list of encoder messages for those matching this segment.
        Then computes the left and right encoder deltas.
        i0 is the search start index. The index of the last matching encoder message
        is returned.
        '''
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

    def calibrate_straight(self):
        '''Computes the alpha and size parameters and their associated variances
        sigma_alpha and sigma_size.
        '''

        # Note that the sigma formulas below are empirical

        # the total variance is inversely proportional to the length of the segment,
        # and proportional to the AMCL variance and the variance of the orientation.
        sigmas = [1.0/self.length, self.amcl_var, self.orientation_range]
        #print self.length, sigmas

        self.alpha = FilterState(self.right / self.left,
                            np.dot(sigmas, [10, 10, 1.0/np.radians(5)]) * 0.01)

        self.size = FilterState(self.length / self.right,
                            np.dot(sigmas, [10, 10, 1.0/np.radians(5)]) * 1)

        #print 'sigma_alpha=%f, sigma_size=%f' % (self.alpha.p, self.size.p)

    def calibrate_curves(self, alpha, size):
        '''Computes the beta parameter and its variance sigma_beta.
        The alpha and size arguments are FilterState objects (value and variance).
        '''

        # Note that the sigma formula is empirical
        sigmas = np.array([
            [1.0/self.length,        1             ],
            [self.amcl_var,          10            ],
            [1.0/abs(self.rotation), np.radians(60)],
            [alpha.p,                1             ],
            [size.p,                 1             ]])
        #print self.length, sigmas[:,0]

        self.beta = FilterState((self.right-self.left*alpha.x) * size.x / self.rotation,
                            np.dot(sigmas[:,0], sigmas[:,1]) * .3)
        #print 'sigma_beta=%f' % self.beta.p


class CalibrationTool:
    '''Collects encoder counts messages and AMCL pose messages. Extracts straigt and
    curve segments. Computes the corresponding odometry parameters.
    '''

    def __init__(self):
        self.segment_classifier = SegmentClassifier()
        self.encoder_counts = []

        self.debug_viz = False
        self.debug_filter_fn = None
        self.segments_viz = [] #holds the curved segments for visualization purpose

        # from time to time, cleanup the list of encoder messages.
        # See remove_encoder_counts()
        self.last_cleanup = None

        # a mutex to protect access to encoder_counts
        self.mutex = threading.Lock()

        # Filters for the estimation of the odometry parameters, initialized with
        # a prior
        # TODO: write to a file the last computed values and use as a prior
        self.left_correction_factor_filter = KalmanFilter(prior=FilterState(1.012,0.001), q=0.00001)
        self.wheel_size_filter = KalmanFilter(prior=FilterState(1.316,0.1), q=0.01)
        self.dist_btw_wheels_filter = KalmanFilter(prior=FilterState(1.017,0.02), q=0.000001)

    def turn_on_filter_log(self, filename):
        self.debug_filter_fn = filename
        f = open(filename, 'w')
        f.write('name, obsx, obsp, valx, valp\n')
        f.close()

    def update_filter(self, f, o, name):
        f.update(o.x, o.p)
        if self.debug_filter_fn:
            s = f.get_state()
            with open(self.debug_filter_fn, 'a') as logfile:
                logfile.write('%s, %f, %f, %f, %f\n' % (name, o.x, o.p, s.x, s.p))

    def append(self, msg):
        '''Appends an encoder message.'''
        with self.mutex:
            self.encoder_counts.append(msg)

    def calibrate_straight(self):
        if len(self.segment_classifier.segments)==0:
            return

        # Go through all the new segments, and precompute the parameters
        segments = []
        i = 0
        for seg in self.segment_classifier.segments[-1:]:
            if seg.type!=Segment.STRAIGHT:
                continue
            cseg = CalibrationSegment(seg)
            try:
                i = cseg.get_encoder_counts(self.encoder_counts, i)
            except IndexError:
                continue
            segments.append(cseg)

        # Actual calibration. Use Kalman filters to estimate the true values from
        # the multiple measurements.
        if len(segments)>0:

            # updates with all measurements and their variance
            for cseg in segments:
                cseg.calibrate_straight()
                self.update_filter(self.left_correction_factor_filter, cseg.alpha, 'alpha')
                self.update_filter(self.wheel_size_filter, cseg.size, 'size')

            # final report
            left_correction_factor = self.left_correction_factor_filter.get_state()
            wheel_size = self.wheel_size_filter.get_state()
            print 'left_correction_factor=%f, variance=%f' % (left_correction_factor.x, left_correction_factor.p)
            print 'wheel_size=%f, variance=%f' % (wheel_size.x, wheel_size.p)
            self.left_correction_factor = left_correction_factor.x
            self.wheel_size = wheel_size.x

    def calibrate_curves(self):
        # See comments in calibrate_straight as it is almost the same algorithm

        if len(self.segment_classifier.segments)==0:
            return

        segments = []
        i = 0
        for seg in self.segment_classifier.segments[-1:]:
            if seg.type!=Segment.CURVED:
                continue
            cseg = CalibrationSegment(seg)
            try:
                i = cseg.get_encoder_counts(self.encoder_counts, i)
            except IndexError:
                continue
            segments.append(cseg)

        if len(segments)>0:
            for cseg in segments:
                alpha = self.left_correction_factor_filter.get_state()
                size = self.wheel_size_filter.get_state()
                cseg.calibrate_curves(alpha, size)
                self.update_filter(self.dist_btw_wheels_filter, cseg.beta, 'beta')
            dist_btw_wheels = self.dist_btw_wheels_filter.get_state()
            self.dist_btw_wheels = dist_btw_wheels.x
            print 'dist_btw_wheels=%f, variance=%f' % (dist_btw_wheels.x, dist_btw_wheels.p)

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
            if len(self.segment_classifier.segments)>0:
                ts.append( self.segment_classifier.segments[0].poses[0].stamp )
            t = min(ts)

            i = len(self.encoder_counts)-1
            while i>=0 and self.encoder_counts[i].stamp>=t:
                i -= 10
            if i>0:
                #print 'removing', i, 'encoder_counts values:', t.to_sec()
                self.encoder_counts = self.encoder_counts[i:]

    def calibrate(self):
        '''Main calibration function.
        - searches the trajectory for suitable segments
        - extracts the relevant information from these segments
        - performs calibration
        - cleans up the list of encoder messages
        '''
        self.segment_classifier.classify()
        self.calibrate_straight()
        self.calibrate_curves()
        self.remove_encoder_counts()

        S = self.segment_classifier.segments
        for i in range(len(S)):
            if i==0:
                if len(self.segments_viz)>0 and self.segments_viz[-1][1][0]==S[0].poses[0]:
                    self.segments_viz[-1][1] = S[0].poses
            else:
                self.segments_viz.append([S[i].type, S[i].poses])
        if len(self.segment_classifier.segments)>0:
            self.segment_classifier.segments = self.segment_classifier.segments[-1:]