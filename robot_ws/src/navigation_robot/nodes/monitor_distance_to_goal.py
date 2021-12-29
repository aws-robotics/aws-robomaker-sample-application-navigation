#!/usr/bin/env python

# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0

from itertools import izip
import time

from nav_msgs.msg import Path
import numpy as np
import rospy
from std_msgs.msg import Float32, Header


class MonitorDistanceToGoal:

    def __init__(self):
        self.scan_sub = rospy.Subscriber(
            '/move_base/NavfnROS/plan', Path, callback=self.report_metric
        )
        self.metric_pub = rospy.Publisher('/distance_to_goal', Float32, queue_size=1)

    def calc_path_distance(self, msg):
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        array = np.array(points, dtype=np.dtype('f8', 'f8'))
        return sum((np.linalg.norm(p0 - p1) for p0, p1 in izip(array[:-2], array[1:])))

    def report_metric(self, msg):
        if not msg.poses:
            rospy.loginfo('Path empty, not calculating distance')
            return

        distance = self.calc_path_distance(msg)
        rospy.loginfo('Distance to goal: %s', distance)

        self.metric_pub.publish(distance)


def main():
    rospy.init_node('monitor_goal_to_distance')
    try:
        monitor = MonitorDistanceToGoal()
        if (monitor):
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
