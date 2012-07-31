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

        alpha: the left correction factor
        size: the size of the wheels
        beta: the distance between the wheels
    '''

    def __init__(self, seg):
        '''Constructed from a Segment'''
        self.poses = seg.poses # this is only needed for visualization
        self.length = seg.length
        self.orientation_range = np.std(seg.orientations)
        self.rotation = seg.orientations[-1] - seg.orientations[0]

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
        sigmas = [1.0/self.length, np.mean([p.cvmax for p in self.poses]), self.orientation_range]
        print self.length, sigmas

        self.alpha = self.right / self.left
        self.sigma_alpha = np.dot(sigmas, [10, 1.0/3, 1.0/np.radians(5)]) * 0.001

        self.size = self.length / self.right
        self.sigma_size = np.dot(sigmas, [10, 1.0/3, 1.0/np.radians(5)]) * 0.1

        print 'sigma_alpha=%f, sigma_size=%f' % (self.sigma_alpha, self.sigma_size)

    def calibrate_curves(self, alpha, size):
        '''Computes the beta parameter and its variance sigma_beta.
        The alpha and size arguments are FilterState objects (value and variance).
        '''

        # Note that the sigma formula is empirical
        sigmas = [1.0/self.length, np.mean([p.cvmax for p in self.poses])]
        sigmas.extend([self.orientation_range, alpha.p, size.p])
        print self.length, sigmas

        self.beta = (self.right-self.left*alpha.x) * size.x / self.rotation
        self.sigma_beta = np.dot(sigmas, [10, 1.0/3, 0, 1, 1]) * 0.1
        print 'sigma_beta=%f' % self.sigma_beta


class CalibrationTool:
    '''Collects encoder counts messages and AMCL pose messages. Extracts straigt and
    curve segments. Computes the corresponding odometry parameters.
    '''

    def __init__(self):
        self.segment_classifier = SegmentClassifier()
        self.encoder_counts = []
        self.curved_segments = []
        self.straight_segments = []

        # from time to time, cleanup the list of encoder messages.
        # See remove_encoder_counts()
        self.last_cleanup = None

        # a mutex to protect access to encoder_counts
        self.mutex = threading.Lock()

        # initial values, needed so that dist_btw_wheels can be computed when
        # no straight segment has been extracted yet.
        # TODO: write to a file the last computed values and use as a prior
        self.left_correction_factor_filter = KalmanFilter(FilterState(1.0101,0.001))
        self.wheel_size_filter = KalmanFilter(FilterState(1.3168544,0.01))

    def append(self, msg):
        '''Appends an encoder message.'''
        with self.mutex:
            self.encoder_counts.append(msg)

    def calibrate_straight(self):
        # self.straight_segments is the list of precomputed segment information.
        # Since the last segment in self.segment_classifier.straight_segments
        # may have been extended since last time, we will recompute its parameters
        if len(self.straight_segments)>0 and len(self.segment_classifier.straight_segments)>0:
            self.straight_segments.pop()

        # Go through all the new segments, and precompute the parameters
        i = 0
        for seg in self.segment_classifier.straight_segments:
            cseg = CalibrationSegment(seg)
            try:
                i = cseg.get_encoder_counts(self.encoder_counts, i)
            except IndexError:
                continue
            cseg.calibrate_straight()
            self.straight_segments.append(cseg)

        # clear the segments that have been processed. Keep only the last one,
        # as it might be updated later.
        if len(self.segment_classifier.straight_segments)>0:
            self.segment_classifier.straight_segments = self.segment_classifier.straight_segments[-1:]

        # Actual calibration. Use Kalman filters to estimate the true values from
        # the multiple measurements.
        if len(self.straight_segments)>0:

            # Use the first measurement as the prior
            cseg = self.straight_segments[0]
            self.left_correction_factor_filter = KalmanFilter(FilterState(cseg.alpha, cseg.sigma_alpha), 0.01)
            self.wheel_size_filter = KalmanFilter(FilterState(cseg.size, cseg.sigma_size), 0.1)

            # updates with all measurements and their variance
            for cseg in self.straight_segments[1:]:
                self.left_correction_factor_filter.update(cseg.alpha, cseg.sigma_alpha)
                self.wheel_size_filter.update(cseg.size, cseg.sigma_size)

            # final report
            left_correction_factor = self.left_correction_factor_filter.get_state()
            wheel_size = self.wheel_size_filter.get_state()
            print 'left_correction_factor=%f, variance=%f' % (left_correction_factor.x, left_correction_factor.p)
            print 'wheel_size=%f, variance=%f' % (wheel_size.x, wheel_size.p)
            self.left_correction_factor = left_correction_factor.x
            self.wheel_size = wheel_size.x

    def calibrate_curves(self):
        # See comments in calibrate_straight as it is almost the same algorithm

        if len(self.curved_segments)>0 and len(self.segment_classifier.curved_segments)>0:
            self.curved_segments.pop()
        i = 0
        for seg in self.segment_classifier.curved_segments:
            cseg = CalibrationSegment(seg)
            try:
                i = cseg.get_encoder_counts(self.encoder_counts, i)
            except IndexError:
                continue
            self.curved_segments.append(cseg)

        if len(self.segment_classifier.curved_segments)>0:
            self.segment_classifier.curved_segments = self.segment_classifier.curved_segments[-1:]

        if len(self.curved_segments)>0:
            cseg = self.curved_segments[0]
            cseg.calibrate_curves(self.left_correction_factor_filter.get_state(), self.wheel_size_filter.get_state())
            self.dist_btw_wheels_filter = KalmanFilter(FilterState(cseg.beta, cseg.sigma_beta), 0.1)
            for cseg in self.curved_segments[1:]:
                cseg.calibrate_curves(self.left_correction_factor_filter.get_state(), self.wheel_size_filter.get_state())
                self.dist_btw_wheels_filter.update(cseg.beta, cseg.sigma_beta)
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