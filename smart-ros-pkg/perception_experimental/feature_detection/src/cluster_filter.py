#!/usr/bin/env python

'''A node to filter clusters according to their size and distance.'''

# Implementation inspired from rosbag filter (see rosbag/src/rosbag/rosbag_main.py)


import roslib; roslib.load_manifest('feature_detection')
import rospy

import sys
import optparse

from feature_detection.msg import clusters as ClustersMsg


def get_filter(argv):

    # Returns a function that implements the filter.
    # expr is evaluated to return a boolean according to conditions on 'width'
    # and 'dist'.
    def expr_eval(expr):
        def eval_fn(width, depth, dist):
            return eval(expr)
        return eval_fn

    parser = optparse.OptionParser(usage="""cluster_filter EXPRESSION

    EXPRESSION can be any Python-legal expression.

    The following variables are available:
    * width: the width of the cluster
    * depth: the depth of the cluster
    * dist: the distance between the cluster and the observer""",
        description='Filter clusters according to size and distance.')

    options, args = parser.parse_args(argv)
    if len(args) == 0:
        parser.error('You must specify an expression.')
    elif len(args) > 1:
        parser.error("Too many arguments.")

    return expr_eval(args[0])


class cluster_filter:

    def __init__(self, filter_fn):
        self.sub = rospy.Subscriber('cluster_filter_in', ClustersMsg, self.callback)
        self.pub = rospy.Publisher('cluster_filter_out', ClustersMsg)


    def callback(self, msg_in):
        msg_out = ClustersMsg()
        msg_out.header = msg_in.header

        for cluster in msg_in.clusters:
            dist = cluster.centroid.x # a frame transform is required here...
            if filter_fn(cluster.width, cluster.depth, dist):
                msg_out.clusters.append(cluster)

        if len(msg_out.clusters) > 0:
            self.pub.publish(msg_out)



rospy.init_node('cluster_filter')
filter_fn = get_filter( rospy.myargv()[1:] )
node = cluster_filter(filter_fn)
rospy.spin()
