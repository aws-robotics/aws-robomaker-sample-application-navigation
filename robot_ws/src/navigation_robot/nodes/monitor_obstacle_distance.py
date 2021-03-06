#!/usr/bin/env python

# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0

import time

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Header


class MonitorNearestObstacle:

    def __init__(self):
        self.scan_sub = rospy.Subscriber('scan', LaserScan, callback=self.report_metric)
        self.metrics_pub = rospy.Publisher('/min_obstacle_distance', Float32, queue_size=1)

    def filter_scan(self, msg):
        rospy.loginfo(
            'Filtering scan values in value range (%s,%s)', msg.range_min, msg.range_max
        )
        return [
            msg.ranges[i]
            for i in range(360)
            if msg.ranges[i] >= msg.range_min and msg.ranges[i] <= msg.range_max
        ]

    def report_metric(self, msg):
        filtered_scan = self.filter_scan(msg)
        if not filtered_scan:
            rospy.loginfo(
                'No obstacles with scan range (%s,%s)', msg.range_min, msg.range_max
            )
            return

        min_distance = min(filtered_scan)
        rospy.loginfo('Nearest obstacle: %s', min_distance)

        self.metrics_pub.publish(min_distance)


def main():
    rospy.init_node('monitor_obstacle_distance')
    try:
        monitor = MonitorNearestObstacle()
        if (monitor):
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
