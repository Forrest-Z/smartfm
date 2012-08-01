# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('odometry_calibration')
import rospy
import tf.transformations

import numpy as np
import threading


def distance(p0,p1):
    '''Computes the distance between 2 points.'''
    return np.sqrt(np.square(p1.x-p0.x)+np.square(p1.y-p0.y))

def anorm(a):
    '''Normalizes the angle between ]-pi, pi]'''
    return np.math.atan2(np.sin(a), np.cos(a))


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

    STRAIGHT = 0
    CURVED = 1

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
        self.segments = []

        # some thresholds for the algorithms
        self.orientation_threshold = np.radians(1)
        self.min_pts_per_seg = 5
        self.min_curvature = np.radians(3)

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
        if len(self.segments)==0 or self.segments[-1].type!=Segment.STRAIGHT:
            #print 'new segment'
            return False
        seg_p = self.segments[-1]

        # compare orientations
        d = abs(anorm(seg.orientation-seg_p.orientation))
        if d > self.orientation_threshold:
            #print 'orientations don\'t match: %f>%f' % (d,self.orientation_threshold)
            return False

        return self.merge(seg_p, seg)

    def try_merge_curved(self, seg):
        #print 'trying to merge (curved)'
        if len(self.segments)==0 or self.segments[-1].type!=Segment.CURVED:
            #print 'new segment'
            return False
        seg_p = self.segments[-1]

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
                seg.type = Segment.STRAIGHT
                self.segments.append(seg)

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
                    seg.type = Segment.CURVED
                    self.segments.append(seg)

        # keep the last few points for future analysis
        I = straigt_seg_idx+curved_seg_idx
        n = len(self.searched.poses)-self.min_pts_per_seg
        last = max([i[1] for i in I] + [0, n])
        #print 'keeping %d points' % (len(self.searched.poses)-last)
        with self.mutex:
            self.amcl_poses = self.searched.poses[last:]+self.amcl_poses