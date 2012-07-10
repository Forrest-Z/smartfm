#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('phidget_encoders')
import rospy
import tf.transformations

from phidget_encoders.msg import EncoderCounts
from geometry_msgs.msg import PoseWithCovarianceStamped, Point32
from sensor_msgs.msg import PointCloud

import math
import numpy as np
import threading

def distance(p0,p1):
    '''Computes the distance between 2 points.'''
    return math.sqrt(math.pow(p1.x-p0.x,2)+math.pow(p1.y-p0.y,2))

def anorm(a):
    '''Normalizes the angle between ]-pi, pi]'''
    return math.atan2(math.sin(a),math.cos(a))

class Pose:
    '''Simple pose representation.'''
    def __init__(self):
        self.stamp = 0
        self.x = 0
        self.y = 0
        self.th = 0
        self.cvmax = 0 #max covariance

def PoseFromAMCLMsg(msg):
    '''Returns a Pose constructed from a PoseWithCovarianceStamped.'''
    pose = Pose()
    pose.stamp = msg.header.stamp
    pose.x = msg.pose.pose.position.x
    pose.y = msg.pose.pose.position.y
    o = msg.pose.pose.orientation
    q = [o.x, o.y, o.z, o.w]
    roll, pitch, pose.th = tf.transformations.euler_from_quaternion(q)
    pose.cvmax = np.abs(msg.pose.covariance).max()
    return pose

class Segment:
    '''Represents a segment as a sequence of Poses.

    Attributes:
        poses: the sequence of AMCL poses

    Attributes available only after compute_geometry() has been called:
        lengths: array of inter-point distances
        length: the sum of lengths, i.e. the total length of the segment
        dths: array of orientation differences
        curvatures: array of curvature values
        curvature: the mean curvature
        orientations: the orientation values, but normalized to avoid jumping from
            pi to -pi
        orientation: the mean orientation
    '''

    def __init__(self, poses=[]):
        self.poses = poses

    def compute_geometry(self):
        '''Computes the geometrical attributes.'''
        self.lengths = []
        self.curvatures = []
        self.dths = []
        self.orientations = []

        if len(self.poses)==0:
            return

        for i in range(1, len(self.poses)):
            p0 = self.poses[i-1]
            p1 = self.poses[i]
            d = distance(p0,p1)
            self.lengths.append(d)
            dth = anorm(p1.th-p0.th)
            self.dths.append(dth)
            self.curvatures.append(dth/d)

        self.length = np.sum(self.lengths)
        self.curvature = np.mean(self.curvatures)

        th = self.poses[0].th
        self.orientations = [th]
        for dth in self.dths:
            th += dth
            self.orientations.append(th)
        self.orientation = np.mean(self.orientations)
        self.rotation = self.orientations[-1] - self.orientations[0]


class SegmentClassifier:
    '''Holds the running pool of poses given by AMCL, and extracts straight and
    curved segments from it.

    Attributes:
        amcl_poses: the pool of AMCL poses
        straight_segments: extracted straight segments
        curved_segments: extracted curved segments (with constant curvature)
        orientation_threshold: when searching for straight lines, include a new
            point to the segment only if its orientation is equal to the mean
            orientation plus or minus this threshold
        min_pts_per_seg: only consider segments with at least that many points
        min_curvature: only consider segments as curved if the curvature is larger
            than this threshold
    '''

    def __init__(self):
        # pool of poses to process: AMCL poses are accumulated here. Periodically
        # it will be searched for segments
        self.amcl_poses = []

        # the output of the extraction algorithms
        self.straight_segments = []
        self.curved_segments = []

        # some thresholds for the algorithms
        self.orientation_threshold = math.radians(1)
        self.min_pts_per_seg = 5
        self.min_curvature = math.radians(3)

        # a mutex to protect access to amcl_poses
        self.mutex = threading.Lock()

    def append(self, p):
        '''Appends a pose to the pool of AMCL poses.'''
        #print p.x, p.y, p.th
        with self.mutex:
            # check whether it's the same pose as before. If it is, simply update
            # the stamp
            if len(self.amcl_poses)>0:
                prev = self.amcl_poses[-1]
                if (p.x,p.y,p.th)==(prev.x,prev.y,prev.th):
                    #print 'updating instead of appending'
                    prev.stamp = p.stamp
                    return
            self.amcl_poses.append(p)

    def search_for_segment(self, a, testfn, min_pts_per_seg=0):
        '''Searches sequences of similar values and returns their
        begining and end index.

        Args:
            a: the array of values to search
            testfn: the function used to decide on similarity. Takes the array of
                values as input, and returns whether they are similar enough or not.
            min_pts_per_seg: minimal number of points required to make a segment

        Returns:
            segs: array of segment indexes boundaries [b,e], where b is the index of
                the first point of the segment, and e the end (excluded). Therefore,
                a[b:e] is the resulting segment.
        '''
        # the output
        segs = []

        #initialize with only one point
        b = 0
        e = b+1

        while e <= len(a):
            # if the next point (if any) does not belong to the segment, then
            # terminate the segment here
            if e>=len(a) or not testfn(a[b:e]):
                # only keep segments that are long enough
                if e-b >= min_pts_per_seg:
                    segs.append([b,e])
                    # start a new segment at the end of this one
                    b = e
                else:
                    # start a new segment at the begining of this one
                    b += 1
                e = b+1
            else:
                # proceed with the next point
                e += 1

        return segs

    def merge(self, seg_p, seg):
        '''Helper function to merge segments

        Args:
            seg_p: the previous segment
            seg: the current segment

        Returns:
            a boolean indicating whether the merge was successful
        '''

        # distance check: the distance between the 2 segments must match the inter
        # point distance in seg_p
        p0 = seg_p.poses[-1]
        p1 = seg.poses[0]
        d = distance(p0, p1)
        t = max(seg_p.lengths) * 1.5
        if d > t:
            #print 'distances don\'t match %f>%f' %(d,t)
            return False

        # merge
        #print 'merging'
        if (p0.x,p0.y,p0.th)==(p1.x,p1.y,p1.th):
            # This is a weird case. I am not sure why this happens... It is
            # important to guard against it though, otherwise curvature computation
            # will fail as the inter point distance will be 0.
            print 'double point', (p0.x,p0.y,p0.th)
            seg_p.poses.extend(seg.poses[1:])
        else:
            seg_p.poses.extend(seg.poses)
        seg_p.compute_geometry()
        return True

    def try_merge_straight(self, seg):
        #print 'trying to merge (straight)'
        if len(self.straight_segments)==0:
            #print 'new segment'
            return False
        seg_p = self.straight_segments[-1]

        # compare orientations
        d = abs(anorm(seg.orientation-seg_p.orientation))
        if d > self.orientation_threshold:
            #print 'orientations don\'t match: %f>%f' % (d,self.orientation_threshold)
            return False

        return self.merge(seg_p, seg)

    def try_merge_curved(self, seg):
        #print 'trying to merge (curved)'
        if len(self.curved_segments)==0:
            #print 'new segment'
            return False
        seg_p = self.curved_segments[-1]

        # compare curvatures
        if np.sign(seg.curvature)!=np.sign(seg_p.curvature):
            #print 'curvatures don\'t match'
            return False

        return self.merge(seg_p, seg)

    def classify(self):
        #print 'classifying'
        with self.mutex:
            self.searched = Segment(self.amcl_poses)
            self.amcl_poses = []

        self.searched.compute_geometry()

        # Search for straigt segments
        testfn = lambda v: max(v)-min(v)<=self.orientation_threshold
        straigt_seg_idx = self.search_for_segment(self.searched.orientations,
                                testfn, self.min_pts_per_seg)
        for si in straigt_seg_idx:
            seg = Segment( self.searched.poses[si[0]:si[1]] )
            seg.compute_geometry()
            if not self.try_merge_straight(seg):
                self.straight_segments.append(seg)

        # Search for curved segments
        def testfn(a):
            ar = np.asarray(a)
            return np.all(ar>0) or np.all(ar<0)
        curved_seg_idx = self.search_for_segment(self.searched.dths, testfn,
                                self.min_pts_per_seg)
        for si in curved_seg_idx:
            si[1] += 1
            seg = Segment( self.searched.poses[si[0]:si[1]] )
            seg.compute_geometry()
            if abs(seg.curvature) > self.min_curvature:
                if not self.try_merge_curved(seg):
                    self.curved_segments.append(seg)

        # keep the last few points for future analysis
        I = straigt_seg_idx+curved_seg_idx
        n = len(self.searched.poses)-self.min_pts_per_seg
        last = max([i[1] for i in I] + [0, n])
        #print 'keeping %d points' % (len(self.searched.poses)-last)
        with self.mutex:
            self.amcl_poses = self.searched.poses[last:]+self.amcl_poses

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
        self.p_orientation = np.exp(-self.orientation_range/math.radians(5))
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

        self.calibration_period = rospy.Duration(5)

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
        ts = [rospy.Time.now() - self.calibration_period]
        if len(self.segment_classifier.amcl_poses)>0:
            ts.append( self.segment_classifier.amcl_poses[0].stamp )
        if len(self.segment_classifier.straight_segments)>0:
            ts.append( self.segment_classifier.straight_segments[0].poses[0].stamp )
        if len(self.segment_classifier.curved_segments)>0:
            ts.append( self.segment_classifier.curved_segments[0].poses[0].stamp )
        t = min(ts)

        with self.mutex:
            if len(self.encoder_counts)==0:
                return
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

class OdometryCalibrationNode:
    '''From encoder counts and AMCL trajectory, calibrate the odometry.

    To derive odometry from encoder we need 2 parameters: the size of the wheels and
    the distance between the wheels. Additionaly, a correction factor is introduced
    to take care of the fact that the 2 wheels do not have exactly the same size.
    Those 3 parameters can be tuned if we know the true trajectory of the vehicle.
    This can be obtained from AMCL when the laser sees enough features to localize
    the car better than the odometry. This node extracts portions of the trajectory
    that are straight segments and those that are segments of constant curvatures,
    and uses them to calibrate the 3 parameters above.
    '''

    def __init__(self):
        self.calibration_tool = CalibrationTool()

        self.encoders_sub = rospy.Subscriber('encoder_counts', EncoderCounts, self.encoders_callback)
        self.amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        # publish the extracted segments as a point cloud (for debugging)
        self.segments_pub = rospy.Publisher('calibration_segments', PointCloud)

        # every 5 seconds, analyze the trajectory and attempt to calibrate
        self.timer = rospy.Timer(self.calibration_tool.calibration_period, self.timer_callback)

        # do it straight away for debugging purpose
        self.timer_callback(0)

    def encoders_callback(self, msg):
        self.calibration_tool.append(msg)

    def amcl_callback(self, msg):
        self.calibration_tool.segment_classifier.append( PoseFromAMCLMsg(msg) )

    def timer_callback(self, dummy):
        self.calibration_tool.calibrate()

        # publish the segments (for visualization purpose)
        msg = PointCloud()
        msg.header.frame_id = 'map'
        d = {1: self.calibration_tool.straight_segments,
             -1: self.calibration_tool.curved_segments}
        for (z, segments) in d.iteritems():
            for i in range(len(segments)):
                for p in segments[i].poses:
                    msg.points.append( Point32(p.x, p.y, z*(i+1)) )
        self.segments_pub.publish(msg)




if __name__=='__main__':
    rospy.init_node('odo_cal_node')
    node = OdometryCalibrationNode()
    rospy.spin()