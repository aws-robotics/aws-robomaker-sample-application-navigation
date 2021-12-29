#!/usr/bin/env python

# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0

import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import Float32, Header


class Monitor:

    def __init__(self, data_topic, data_msg, metric_topic, transform):
        self.metrics_pub = rospy.Publisher(metric_topic, Twist, queue_size=1)
        self.topic_sub = rospy.Subscriber(data_topic, data_msg, self.callback)
        self.transform = transform

    def callback(self, message):
        twist = self.transform(message)
        rospy.loginfo('Robot speed: %s', twist)
        self.metrics_pub.publish(twist)


def odom_to_speed(odom):
    return odom.twist.twist


def main():
    rospy.init_node('speed_monitor')
    monitor = Monitor(data_topic='/odom',
                      data_msg=Odometry,
                      metric_topic='/robot_speed',
                      transform=odom_to_speed)
    if (monitor):
        rospy.spin()


if __name__ == '__main__':
    main()
