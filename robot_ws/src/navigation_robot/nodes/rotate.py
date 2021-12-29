#!/usr/bin/env python

# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0

from geometry_msgs.msg import Twist

import rospy


class Rotator:

    def __init__(self):
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def rotate_forever(self):
        self.twist = Twist()

        direction = 1
        angular_speed = 0.2
        r = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            self.twist.angular.z = direction * angular_speed
            self._cmd_pub.publish(self.twist)
            rospy.loginfo('Rotating Robot: %s', self.twist)
            r.sleep()


def main():
    rospy.init_node('rotate')
    try:
        rotator = Rotator()
        rotator.rotate_forever()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
